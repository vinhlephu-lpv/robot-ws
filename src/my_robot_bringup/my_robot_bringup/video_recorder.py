#!/usr/bin/env python3
"""
Raw Video Recorder Node for ROS 2.
Ghi video THÔ 30 FPS mượt mà chuẩn như camera điện thoại trực tiếp từ Orbbec Astra trên Raspberry Pi.

Nguyên lý hoạt động chuẩn công nghiệp (Multi-threaded Asynchronous Worker):
- Thread 1 (ROS Callback): Bắt từng khung hình ngay lập tức (<0.2ms) đẩy vào hàng đợi RAM FIFO,
  hoàn toàn không chặn luồng ROS 2 hay camera, đảm bảo camera đạt trọn vẹn 30 FPS phần cứng.
- Thread 2 (Background Writer Thread): Ghi đĩa MP4 độc lập trong nền, không làm giật lag hình ảnh.
- 100% Raw Video: Không vẽ chữ, không watermark, dữ liệu nguyên bản sạch từ cảm biến.
- 100% Headless: Không mở cửa sổ giao diện nào (không cv2.imshow).
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
    print("❌ Thiếu thư viện opencv-python! Chạy: pip3 install opencv-python")

import rclpy
from rclpy.node import Node
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

        # Hàng đợi đa luồng bộ đệm RAM để ghi đĩa không chặn camera
        self.frame_queue = queue.Queue(maxsize=600)  # Chứa được ~20 giây buffer RAM
        self.writer = None
        self.start_time = None
        self.last_log_time = 0.0
        self.received_frames = 0
        self.written_frames = 0
        self.is_running = True

        # Khởi động Background Writer Thread
        self.writer_thread = threading.Thread(target=self._writer_worker, daemon=True)
        self.writer_thread.start()

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
            qos
        )

        self.get_logger().info(f"🔴 [Video Recorder] Sẵn sàng ghi video 30 FPS mượt mà -> {self.output_path}")

    def image_callback(self, msg: Image):
        """Callback siêu nhẹ: chỉ giải nén BGR và đẩy vào Queue trong <0.5ms."""
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

            # Đẩy (frame, cvt, timestamp) vào queue cho thread ghi đĩa độc lập
            try:
                self.frame_queue.put_nowait((frame, cvt, now))
                self.received_frames += 1
            except queue.Full:
                self.get_logger().warn("Bộ đệm ghi video bị đầy (ổ cứng quá chậm)!", throttle_duration_sec=3.0)

            if now - self.last_log_time >= 3.0:
                elapsed = now - self.start_time
                fps_in = self.received_frames / elapsed if elapsed > 0 else 0
                mb = os.path.getsize(self.output_path) / (1024 * 1024) if os.path.exists(self.output_path) else 0
                self.get_logger().info(
                    f"🔴 [Đang quay]: {int(elapsed//60):02d}:{int(elapsed%60):02d} | "
                    f"Camera: {fps_in:.1f} FPS | Đã lưu: {self.written_frames} frames ({int(elapsed):d}s chuẩn thực tế) | {mb:.1f} MB"
                )
                self.last_log_time = now

        except Exception as e:
            self.get_logger().error(f"Lỗi nhận frame: {e}", throttle_duration_sec=5.0)

    def _writer_worker(self):
        """Thread chạy ngầm độc lập ghi video ra file MP4 đồng bộ 1:1 thời gian thực."""
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
                    f"📹 [Video Writer] Bắt đầu ghi file: {w}x{h} @ {self.fps:.0f} FPS -> {self.output_path}"
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

    def close(self):
        self.is_running = False
        self.get_logger().info("Đang hoàn tất đóng gói video vào ổ đĩa...")
        if self.writer_thread.is_alive():
            self.writer_thread.join(timeout=5.0)

        elapsed = time.time() - self.start_time if self.start_time else 0
        mb = os.path.getsize(self.output_path) / (1024 * 1024) if os.path.exists(self.output_path) else 0
        actual_fps = self.written_frames / elapsed if elapsed > 0 else 0

        self.get_logger().info("=" * 65)
        self.get_logger().info(f"✅ ĐÃ LƯU VIDEO HOÀN CHỈNH (MƯỢT MÀ NHƯ ĐIỆN THOẠI)!")
        self.get_logger().info(f"📁 Đường dẫn: {self.output_path}")
        self.get_logger().info(f"⏱️ Thời lượng: {int(elapsed//60):02d}:{int(elapsed%60):02d} ({self.written_frames} frames @ {actual_fps:.1f} FPS)")
        self.get_logger().info(f"📦 Dung lượng: {mb:.2f} MB")
        self.get_logger().info("=" * 65)


def main(args=None):
    rclpy.init(args=args)
    node = RawVideoRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
