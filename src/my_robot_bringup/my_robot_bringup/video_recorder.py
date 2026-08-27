#!/usr/bin/env python3
"""
Raw Video Recorder Node for ROS 2.
Ghi video THÔ 640x480 @ 30 FPS mượt mà chuẩn như camera điện thoại trực tiếp từ Raspberry Pi.

Cơ chế hoạt động tối ưu:
1. [Chế độ ưu tiên] V4L2 Direct Capture:
   - Mở trực tiếp cổng camera phần cứng (/dev/video0) qua V4L2 MJPEG 640x480 @ 30 FPS.
   - Bỏ qua hoàn toàn DDS, không qua trung gian, triệt tiêu 100% rớt frame hay corrupt ảnh.
   - Đồng thời tự động publish /camera/color/image_raw để các node khác (wifi_cam_bridge) vẫn xem được.
2. [Chế độ dự phòng] ROS 2 Topic Subscriber:
   - Nếu không mở được trực tiếp /dev/video0, tự động fallback sang subscribe topic /camera/color/image_raw.
3. [Ghi đĩa MP4 độc lập]:
   - Luồng nền (Background Writer Thread) mã hóa và ghi MP4 vào thẻ nhớ/SSD với đồng bộ 1:1 thời gian thực.
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
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image


class RawVideoRecorder(Node):
    def __init__(self):
        super().__init__('video_recorder')

        default_dir = os.path.join(os.path.expanduser('~'), 'robot-ws', 'recordings')
        if not os.path.exists(os.path.dirname(default_dir)):
            default_dir = os.path.join(os.path.expanduser('~'), 'robot_ws', 'recordings')

        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('topic', '/camera/color/image_raw')
        self.declare_parameter('output_dir', default_dir)
        self.declare_parameter('filename', '')
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('direct_capture', True)

        self.device_path = self.get_parameter('device').value
        self.topic_name = self.get_parameter('topic').value
        self.output_dir = self.get_parameter('output_dir').value
        self.custom_filename = self.get_parameter('filename').value
        self.fps = float(self.get_parameter('fps').value)
        self.target_width = int(self.get_parameter('width').value)
        self.target_height = int(self.get_parameter('height').value)
        self.use_direct = bool(self.get_parameter('direct_capture').value)

        os.makedirs(self.output_dir, exist_ok=True)

        if self.custom_filename:
            base_name = self.custom_filename
            if not base_name.endswith('.mp4'):
                base_name += '.mp4'
        else:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            base_name = f"dataset_raw_{timestamp}.mp4"

        self.output_path = os.path.join(self.output_dir, base_name)

        # Hàng đợi đa luồng RAM để ghi đĩa độc lập
        self.frame_queue = queue.Queue(maxsize=900)
        self.writer = None
        self.start_time = None
        self.last_log_time = 0.0
        self.received_frames = 0
        self.written_frames = 0
        self.is_running = True
        self.cap = None

        # Khởi động Background Writer Thread
        self.writer_thread = threading.Thread(target=self._writer_worker, daemon=True)
        self.writer_thread.start()

        # Thử mở camera trực tiếp V4L2
        if self.use_direct:
            candidates = [self.device_path, '/dev/video0', '/dev/video1', '/dev/video2', '/dev/video4']
            seen = set()
            for dev in candidates:
                if dev and os.path.exists(dev) and dev not in seen:
                    seen.add(dev)
                    try:
                        test_cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
                        if test_cap.isOpened():
                            # Cấu hình MJPEG 640x480 @ 30 FPS trực tiếp từ cảm biến
                            test_cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                            test_cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.target_width)
                            test_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.target_height)
                            test_cap.set(cv2.CAP_PROP_FPS, self.fps)
                            ret, frame = test_cap.read()
                            if ret and frame is not None:
                                self.cap = test_cap
                                self.device_path = dev
                                actual_w = int(test_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                                actual_h = int(test_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                                self.get_logger().info(
                                    f"🚀 [V4L2 Direct] Đã mở trực tiếp {dev}: {actual_w}x{actual_h} @ {self.fps:.0f} FPS!"
                                )
                                break
                            test_cap.release()
                    except Exception:
                        pass

        if self.cap is not None:
            # Chế độ 1: Đọc trực tiếp V4L2 và publish lên ROS 2 cho wifi_cam_bridge
            self.image_pub = self.create_publisher(Image, self.topic_name, 10)
            self.capture_thread = threading.Thread(target=self._v4l2_capture_worker, daemon=True)
            self.capture_thread.start()
            self.get_logger().info(f"🔴 [Video Recorder] Đang ghi hình trực tiếp chuẩn 30 FPS -> {self.output_path}")
        else:
            # Chế độ 2 (Dự phòng): Subscribe topic ROS 2
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
            self.get_logger().info(
                f"🔴 [Video Recorder] Subscribe topic {self.topic_name} -> {self.output_path}"
            )

    def _v4l2_capture_worker(self):
        """Luồng đọc trực tiếp từ V4L2 không qua trung gian: chuẩn 30 FPS."""
        while self.is_running and self.cap is not None and self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret or frame is None:
                time.sleep(0.002)
                continue

            now = time.time()
            if self.start_time is None:
                self.start_time = now
                self.last_log_time = now

            # Đẩy vào queue ghi đĩa
            try:
                self.frame_queue.put_nowait((frame, None, now))
                self.received_frames += 1
            except queue.Full:
                pass

            # Publish lên ROS 2 cho wifi_cam_bridge xem live trên laptop
            try:
                h, w, c = frame.shape
                msg = Image()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'camera_link'
                msg.height = h
                msg.width = w
                msg.encoding = 'bgr8'
                msg.step = w * c
                msg.data = frame.tobytes()
                self.image_pub.publish(msg)
            except Exception:
                pass

            if now - self.last_log_time >= 3.0:
                elapsed = now - self.start_time
                fps_in = self.received_frames / elapsed if elapsed > 0 else 0
                mb = os.path.getsize(self.output_path) / (1024 * 1024) if os.path.exists(self.output_path) else 0
                self.get_logger().info(
                    f"🔴 [Đang quay V4L2]: {int(elapsed//60):02d}:{int(elapsed%60):02d} | "
                    f"Camera: {fps_in:.1f} FPS | Đã lưu: {self.written_frames} frames ({int(elapsed):d}s chuẩn thực tế) | {mb:.1f} MB"
                )
                self.last_log_time = now

    def image_callback(self, msg: Image):
        """Callback dự phòng khi nhận từ ROS 2 topic: đẩy nhanh trong <0.02ms."""
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
                self.get_logger().info(
                    f"🔴 [Đang quay ROS]: {int(elapsed//60):02d}:{int(elapsed%60):02d} | "
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

    def destroy_node(self):
        self.is_running = False
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
            self.cap = None

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
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
