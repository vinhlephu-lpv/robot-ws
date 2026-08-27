#!/usr/bin/env python3
"""
Raw Video Recorder Node for ROS 2.
Ghi video THÔ trực tiếp từ sensor camera Orbbec Astra ra file MP4 chuẩn trên Raspberry Pi.

Đặc điểm:
- 100% Raw Data: Không vẽ bất kỳ chữ/thông tin/watermark nào lên ảnh.
- 100% Headless: Không mở bất kỳ cửa sổ GUI nào (không cv2.imshow).
- Tiết kiệm băng thông: Không stream video thô qua Wi-Fi trong lúc quay.
- Tự động lưu vào ~/robot-ws/recordings/.
"""

import os
import sys
import time
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

        # Thư mục lưu mặc định
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
        self.writer = None
        self.frame_count = 0
        self.start_time = None
        self.last_log_time = 0.0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE
        )

        self.sub = self.create_subscription(
            Image,
            self.topic_name,
            self.image_callback,
            qos
        )

        self.get_logger().info(f"[Video Recorder] Đã bật chế độ quay video thô -> {self.output_path}")

    def image_callback(self, msg: Image):
        try:
            # Chuyển đổi dữ liệu ROS sang ảnh BGR thô thuần túy
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
            if cvt is not None:
                frame = cv2.cvtColor(frame, cvt)

            now = time.time()
            if self.writer is None:
                h, w = frame.shape[:2]
                self.start_time = now
                self.last_log_time = now
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                self.writer = cv2.VideoWriter(self.output_path, fourcc, self.fps, (w, h))
                self.get_logger().info(
                    f"🔴 [Video Recorder] BẮT ĐẦU GHI: {w}x{h} @ {self.fps:.0f} FPS -> {self.output_path}"
                )

            # GHI TRỰC TIẾP FRAME THÔ (KHÔNG VẼ BẤT KỲ GÌ LÊN ẢNH)
            self.writer.write(frame)
            self.frame_count += 1

            if now - self.last_log_time >= 5.0:
                elapsed = now - self.start_time
                fps_actual = self.frame_count / elapsed if elapsed > 0 else 0
                mb = os.path.getsize(self.output_path) / (1024 * 1024) if os.path.exists(self.output_path) else 0
                self.get_logger().info(
                    f"[Video Recorder] Đang ghi: {self.frame_count} frames ({fps_actual:.1f} fps) | {mb:.1f} MB"
                )
                self.last_log_time = now

        except Exception as e:
            self.get_logger().error(f"Lỗi ghi frame: {e}", throttle_duration_sec=5.0)

    def close(self):
        if self.writer is not None:
            self.writer.release()
            self.writer = None
            mb = os.path.getsize(self.output_path) / (1024 * 1024) if os.path.exists(self.output_path) else 0
            self.get_logger().info("=" * 60)
            self.get_logger().info(f"✅ ĐÃ HOÀN TẤT LƯU VIDEO THÔ!")
            self.get_logger().info(f"📁 Đường dẫn: {self.output_path}")
            self.get_logger().info(f"📦 Tổng: {self.frame_count} frames | {mb:.2f} MB")
            self.get_logger().info("=" * 60)


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
