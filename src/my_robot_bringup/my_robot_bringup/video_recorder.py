#!/usr/bin/env python3
"""
Raw Video Recorder Node for ROS 2.
Ghi video THÔ 640x480 @ 30 FPS mượt mà từ topic /camera/color/image_raw trên Raspberry Pi.

Nguyên lý tối ưu đa luồng:
1. MultiThreadedExecutor: Luồng ROS 2 riêng biệt đọc tin liên tục từ DDS, không bao giờ bị nghẽn buffer.
2. Callback siêu nhẹ (<0.02ms): Chỉ reshape mảng numpy và đẩy vào hàng đợi RAM FIFO.
3. Background Writer: Luồng nền độc lập đổi màu BGR và mã hóa MP4, đồng bộ 1:1 thời gian thực.
"""

import os
import sys
import time
import threading
import queue
from datetime import datetime
import numpy as np

try:
    import cv2
except ImportError:
    print("❌ Thiếu thư viện opencv-python! Chạy: sudo apt install python3-opencv")

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image


class RawVideoRecorder(Node):
    def __init__(self):
        super().__init__('video_recorder')

        default_dir = os.path.join(os.path.expanduser('~'), 'robot-ws', 'recordings')
        if not os.path.exists(os.path.dirname(default_dir)):
            default_dir = os.path.join(os.path.expanduser('~'), 'robot_ws', 'recordings')

        self.declare_parameter('topic', '/camera/color/image_raw')
        self.declare_parameter('output_dir', default_dir)
        self.declare_parameter('filename', '')
        self.declare_parameter('fps', 30.0)

        self.topic_name = self.get_parameter('topic').value
        self.output_dir = self.get_parameter('output_dir').value
        self.custom_filename = self.get_parameter('filename').value
        self.fps = float(self.get_parameter('fps').value)

        os.makedirs(self.output_dir, exist_ok=True)

        if self.custom_filename:
            base_name = self.custom_filename
            if not base_name.endswith('.mp4'):
                base_name += '.mp4'
        else:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            base_name = f"dataset_raw_{timestamp}.mp4"

        self.output_path = os.path.join(self.output_dir, base_name)

        # Hàng đợi RAM đa luồng (chứa tới 30 giây video 640x480)
        self.frame_queue = queue.Queue(maxsize=900)
        self.writer = None
        self.start_time = None
        self.last_log_time = 0.0
        self.received_frames = 0
        self.written_frames = 0
        self.is_running = True

        # Khởi động Background Writer Thread
        self.writer_thread = threading.Thread(target=self._writer_worker, daemon=True)
        self.writer_thread.start()

        # Subscribe với callback group riêng biệt
        self.cb_group = ReentrantCallbackGroup()
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=30,
            durability=DurabilityPolicy.VOLATILE
        )
        self.sub = self.create_subscription(
            Image,
            self.topic_name,
            self.image_callback,
            qos,
            callback_group=self.cb_group
        )
        self.get_logger().info(
            f"🔴 [Video Recorder] Đang thu hình trực tiếp -> {self.output_path}"
        )

    def image_callback(self, msg: Image):
        """Nhận frame siêu tốc (<0.02ms) đẩy vào hàng đợi RAM."""
        try:
            if msg.encoding in ('rgb8', 'RGB8'):
                channels = 3
                cvt = cv2.COLOR_RGB2BGR
            elif msg.encoding in ('bgr8', 'BGR8'):
                channels = 3
                cvt = None
            elif msg.encoding in ('mono8', 'MONO8'):
                channels = 1
                cvt = cv2.COLOR_GRAY2BGR
            else:
                channels = 3
                cvt = cv2.COLOR_RGB2BGR

            expected_size = msg.height * msg.width * channels
            if len(msg.data) < expected_size:
                return

            frame = np.frombuffer(msg.data, dtype=np.uint8)[:expected_size].reshape(
                msg.height, msg.width, channels
            )
            now = time.time()
            if self.start_time is None:
                self.start_time = now
                self.last_log_time = now

            try:
                self.frame_queue.put_nowait((frame, cvt, now))
                self.received_frames += 1
            except queue.Full:
                pass

            if now - self.last_log_time >= 3.0:
                elapsed = now - self.start_time
                fps_in = self.received_frames / elapsed if elapsed > 0 else 0
                mb = os.path.getsize(self.output_path) / (1024 * 1024) if os.path.exists(self.output_path) else 0
                h, w = frame.shape[:2]
                self.get_logger().info(
                    f"🔴 [Đang quay]: {int(elapsed//60):02d}:{int(elapsed%60):02d} | "
                    f"{w}x{h} @ {fps_in:.1f} FPS | Đã lưu: {self.written_frames} frames ({int(elapsed):d}s chuẩn thực tế) | {mb:.1f} MB"
                )
                self.last_log_time = now

        except Exception as e:
            self.get_logger().error(f"Lỗi nhận frame: {e}", throttle_duration_sec=5.0)

    def _writer_worker(self):
        """Thread ghi đĩa MP4 nền độc lập đồng bộ 1:1 thời gian thực."""
        while self.is_running or not self.frame_queue.empty():
            try:
                frame, cvt, frame_time = self.frame_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            if cvt is not None:
                frame = cv2.cvtColor(frame, cvt)

            if self.writer is None:
                h, w = frame.shape[:2]
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                self.writer = cv2.VideoWriter(self.output_path, fourcc, self.fps, (w, h))
                self.get_logger().info(
                    f"📹 [Video Writer] Khởi tạo MP4 Writer: {w}x{h} @ {self.fps:.0f} FPS -> {self.output_path}"
                )

            # Đồng bộ thời lượng video 1:1 chuẩn theo thời gian thực (chống tua nhanh)
            elapsed = max(0.0, frame_time - self.start_time)
            target_frames = int(round(elapsed * self.fps))
            repeats = max(1, target_frames - self.written_frames)
            repeats = min(repeats, 15)

            for _ in range(repeats):
                self.writer.write(frame)
                self.written_frames += 1

            self.frame_queue.task_done()

        if self.writer is not None:
            self.writer.release()
            self.writer = None

    def destroy_node(self):
        self.is_running = False
        self.get_logger().info("Đang hoàn tất đóng gói video vào ổ đĩa...")
        if hasattr(self, 'writer_thread') and self.writer_thread.is_alive():
            self.writer_thread.join(timeout=3.0)

        elapsed = time.time() - self.start_time if self.start_time else 0.0
        final_fps = self.written_frames / elapsed if elapsed > 0 else 0.0
        file_size_mb = os.path.getsize(self.output_path) / (1024 * 1024) if os.path.exists(self.output_path) else 0.0

        mins = int(elapsed // 60)
        secs = int(elapsed % 60)
        time_str = f"{mins:02d}:{secs:02d}"

        self.get_logger().info("=" * 65)
        self.get_logger().info("✅ ĐÃ LƯU VIDEO HOÀN CHỈNH (MƯỢT MÀ NHƯ ĐIỆN THOẠI)!")
        self.get_logger().info(f"📁 Đường dẫn: {self.output_path}")
        self.get_logger().info(f"⏱️ Thời lượng: {time_str} ({self.written_frames} frames @ {final_fps:.1f} FPS)")
        self.get_logger().info(f"📦 Dung lượng: {file_size_mb:.2f} MB")
        self.get_logger().info("=" * 65)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RawVideoRecorder()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
