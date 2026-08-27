#!/usr/bin/env python3
"""
Camera Recorder & Frame Extractor Node for RViz.
Mở camera USB trên Laptop, hiển thị trực tiếp trên RViz (kèm mô hình xe 3D),
đồng thời tự động quay video MP4 và tách sẵn từng frame ảnh vào bộ dataset (imgs/ và videos/ riêng biệt).
"""

import os
import sys
import time
import csv
import glob
import threading
from datetime import datetime
import numpy as np

try:
    import cv2
except ImportError:
    print("❌ Thiếu thư viện opencv-python! Chạy: pip3 install opencv-python")

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist


class CameraRecorderNode(Node):
    def __init__(self):
        super().__init__('camera_recorder_node')

        # Thư mục gốc dataset chuẩn (lưu vào robot-ws/dataset hoặc robot_ws/dataset)
        ws_candidates = [
            os.path.join(os.path.expanduser('~'), 'robot-ws', 'dataset'),
            os.path.join(os.path.expanduser('~'), 'robot_ws', 'dataset'),
            os.path.join(os.path.expanduser('~'), 'Màn hình nền', 'robot_ws', 'dataset'),
        ]
        default_dataset_dir = ws_candidates[0]
        for cand in ws_candidates:
            if os.path.exists(os.path.dirname(cand)):
                default_dataset_dir = cand
                break

        self.declare_parameter('device', 'auto')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('topic', '/camera/color/image_raw')
        self.declare_parameter('record_name', '')
        self.declare_parameter('extract_interval', 0.2)  # Mỗi 0.2s lưu 1 ảnh (5 fps) để tránh trùng lặp
        self.declare_parameter('output_dir', default_dataset_dir)

        self.device_param = self.get_parameter('device').value
        self.target_width = int(self.get_parameter('width').value)
        self.target_height = int(self.get_parameter('height').value)
        self.target_fps = float(self.get_parameter('fps').value)
        self.topic_name = self.get_parameter('topic').value
        self.custom_name = self.get_parameter('record_name').value
        self.extract_interval = float(self.get_parameter('extract_interval').value)
        self.dataset_root = self.get_parameter('output_dir').value

        # Tên phiên thu thập dữ liệu
        if self.custom_name:
            self.session_name = self.custom_name.replace('.mp4', '').replace(' ', '_')
        else:
            self.session_name = f"dataset_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

        # 1. Thư mục lưu video riêng biệt: dataset/videos/<session_name>.mp4
        self.video_dir = os.path.join(self.dataset_root, 'videos')
        os.makedirs(self.video_dir, exist_ok=True)
        self.video_path = os.path.join(self.video_dir, f"{self.session_name}.mp4")

        # 2. Thư mục lưu ảnh tách frame riêng biệt: dataset/imgs/<session_name>/frame_00001.jpg
        self.imgs_dir = os.path.join(self.dataset_root, 'imgs', self.session_name)
        os.makedirs(self.imgs_dir, exist_ok=True)

        # File nhãn CSV lưu góc lái và tốc độ đồng bộ với từng ảnh (cho train CNN)
        self.csv_path = os.path.join(self.imgs_dir, 'labels.csv')
        self.csv_file = open(self.csv_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['frame_file', 'timestamp', 'linear_x', 'angular_z'])

        # ROS 2 Publisher
        self.image_pub = self.create_publisher(Image, self.topic_name, 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/color/camera_info', 10)

        # Subscribe /cmd_vel để gán nhãn góc lái cho ảnh
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self._cmd_callback, 10)

        # Mở Camera USB
        self.cap = self._open_camera()
        if self.cap is None or not self.cap.isOpened():
            self.get_logger().error("❌ KHÔNG THỂ MỞ CAMERA USB! Vui lòng kiểm tra lại dây cắm USB.")
            sys.exit(1)

        self.writer = None
        self.total_video_frames = 0
        self.total_extracted_images = 0
        self.last_extract_time = 0.0
        self.start_time = None
        self.last_log_time = 0.0
        self.is_running = True

        # Khởi động luồng đọc camera & ghi đĩa liên tục 30 FPS
        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()

        self.get_logger().info("=" * 65)
        self.get_logger().info(f"🚀 [RViz Camera Recorder] KHỞI ĐỘNG THÀNH CÔNG!")
        self.get_logger().info(f"📹 Topic phát hình lên RViz: {self.topic_name}")
        self.get_logger().info(f"🎞️ Đường dẫn Video riêng:   {self.video_path}")
        self.get_logger().info(f"🖼️ Thư mục Ảnh riêng:       {self.imgs_dir}/")
        self.get_logger().info(f"⏱️ Chu kỳ tách ảnh:         Mỗi {self.extract_interval}s/ảnh (tránh trùng lặp)")
        self.get_logger().info("=" * 65)

    def _open_camera(self):
        """Tự động dò tìm cổng camera USB khả dụng nhất."""
        if self.device_param != 'auto' and os.path.exists(self.device_param):
            candidates = [self.device_param]
        else:
            # Ưu tiên các cổng USB gắn ngoài (/dev/video2, /dev/video4) rồi đến webcam tích hợp (/dev/video0)
            devs = sorted(glob.glob('/dev/video*'))
            candidates = [d for d in devs if d not in ('/dev/video0', '/dev/video1')] + ['/dev/video0']

        for dev in candidates:
            try:
                cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
                if cap.isOpened():
                    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.target_width)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.target_height)
                    cap.set(cv2.CAP_PROP_FPS, self.target_fps)
                    ret, test_frame = cap.read()
                    if ret and test_frame is not None:
                        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                        self.get_logger().info(f"📷 Đã kết nối Camera tại {dev}: {w}x{h} @ {self.target_fps} FPS")
                        return cap
                    cap.release()
            except Exception:
                pass
        return None

    def _cmd_callback(self, msg: Twist):
        self.current_linear_x = msg.linear.x
        self.current_angular_z = msg.angular.z

    def _capture_loop(self):
        """Vòng lặp đọc camera 30 FPS: Vừa phát lên RViz, vừa ghi video, vừa tách ảnh."""
        while self.is_running and self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret or frame is None:
                time.sleep(0.005)
                continue

            now = time.time()
            if self.start_time is None:
                self.start_time = now
                self.last_log_time = now

            h, w, c = frame.shape

            # 1. Phát frame lên ROS 2 để RViz hiển thị trực tiếp
            try:
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

            # 2. Khởi tạo VideoWriter (nếu chưa có) và ghi vào file MP4 riêng biệt
            if self.writer is None:
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                self.writer = cv2.VideoWriter(self.video_path, fourcc, self.target_fps, (w, h))

            self.writer.write(frame)
            self.total_video_frames += 1

            # 3. Tách frame sạch theo chu kỳ và lưu vào thư mục dataset/imgs/ riêng biệt
            if (now - self.last_extract_time) >= self.extract_interval:
                self.total_extracted_images += 1
                img_filename = f"frame_{self.total_extracted_images:05d}.jpg"
                img_save_path = os.path.join(self.imgs_dir, img_filename)
                cv2.imwrite(img_save_path, frame, [cv2.IMWRITE_JPEG_QUALITY, 95])

                # Lưu nhãn vào CSV
                self.csv_writer.writerow([
                    img_filename,
                    f"{now:.3f}",
                    f"{self.current_linear_x:.3f}",
                    f"{self.current_angular_z:.3f}"
                ])
                self.csv_file.flush()
                self.last_extract_time = now

            # In nhật ký định kỳ
            if now - self.last_log_time >= 2.5:
                elapsed = now - self.start_time
                fps = self.total_video_frames / elapsed if elapsed > 0 else 0
                mb = os.path.getsize(self.video_path) / (1024 * 1024) if os.path.exists(self.video_path) else 0
                mins = int(elapsed // 60)
                secs = int(elapsed % 60)
                self.get_logger().info(
                    f"🔴 [Quay RViz]: {mins:02d}:{secs:02d} | Video: {self.total_video_frames} frames ({fps:.1f} FPS, {mb:.1f}MB) | "
                    f"Đã tách: {self.total_extracted_images} ảnh -> dataset/imgs/{self.session_name}/"
                )
                self.last_log_time = now

    def destroy_node(self):
        self.is_running = False
        if hasattr(self, 'capture_thread') and self.capture_thread.is_alive():
            self.capture_thread.join(timeout=2.0)

        if self.writer is not None:
            self.writer.release()
            self.writer = None

        if self.cap is not None:
            self.cap.release()
            self.cap = None

        if hasattr(self, 'csv_file') and self.csv_file and not self.csv_file.closed:
            self.csv_file.close()

        elapsed = time.time() - self.start_time if self.start_time else 0.0
        final_fps = self.total_video_frames / elapsed if elapsed > 0 else 0.0
        mb = os.path.getsize(self.video_path) / (1024 * 1024) if os.path.exists(self.video_path) else 0.0

        mins = int(elapsed // 60)
        secs = int(elapsed % 60)

        self.get_logger().info("=" * 65)
        self.get_logger().info("✅ ĐÃ HOÀN TẤT THU THẬP DỮ LIỆU TỪ RVIZ!")
        self.get_logger().info(f"⏱️ Thời lượng:       {mins:02d}:{secs:02d} ({self.total_video_frames} frames @ {final_fps:.1f} FPS)")
        self.get_logger().info(f"📹 File Video riêng: {self.video_path} ({mb:.2f} MB)")
        self.get_logger().info(f"🖼️ Thư mục Ảnh riêng:{self.imgs_dir}/ ({self.total_extracted_images} ảnh sẵn sàng train)")
        self.get_logger().info(f"📊 File nhãn CSV:   {self.csv_path}")
        self.get_logger().info("=" * 65)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraRecorderNode()
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
