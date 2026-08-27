#!/usr/bin/env python3
"""
Record Video Tool - Quay video chất lượng cao trực tiếp trên Raspberry Pi.

- Subscribe topic ROS 2: /camera/color/image_raw (hoặc /camera/image_raw)
- Ghi trực tiếp ra file MP4 chuẩn (640x480 @ 30 FPS) lưu trên thẻ nhớ/SSD của Pi.
- Không tiêu tốn băng thông Wi-Fi trong lúc quay -> Xe và LiDAR chạy mượt 100%.
- Sau khi bấm Ctrl+C dừng quay: Tự động hỗ trợ gửi video về Laptop qua SCP hoặc mở link Web tải về.

Sử dụng:
    python3 record_video.py
    python3 record_video.py [tên_video]
    python3 record_video.py [tên_video] --topic /camera/color/image_raw
"""

import os
import sys
import time
import argparse
from datetime import datetime
import numpy as np

try:
    import cv2
except ImportError:
    print("❌ Lỗi: Chưa cài đặt opencv-python! Chạy: pip3 install opencv-python")
    sys.exit(1)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image


class VideoRecorderNode(Node):
    def __init__(self, output_path, topic_name):
        super().__init__('video_recorder_node')
        self.output_path = output_path
        self.topic_name = topic_name

        self.writer = None
        self.frame_count = 0
        self.start_time = None
        self.last_stat_time = 0.0
        self.width = None
        self.height = None
        self.fps = 30.0

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

        self.get_logger().info(f"Đang chờ khung hình đầu tiên từ topic: {self.topic_name} ...")

    def image_callback(self, msg: Image):
        try:
            # Chuyển đổi dữ liệu ROS Image sang numpy array
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
                self.height, self.width = frame.shape[:2]
                self.start_time = now
                self.last_stat_time = now

                # Khởi tạo VideoWriter (MP4)
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                self.writer = cv2.VideoWriter(
                    self.output_path, fourcc, self.fps, (self.width, self.height)
                )
                self.total_written_frames = 0
                self.raw_received_frames = 0

                print("\n" + "=" * 60)
                print(f"🔴 ĐANG QUAY VIDEO (ĐỒNG BỘ 1:1): {os.path.basename(self.output_path)}")
                print(f"📐 Độ phân giải: {self.width}x{self.height} | Tốc độ: {self.fps:.0f} FPS")
                print(f"📁 Lưu tại: {self.output_path}")
                print("👉 Bấm [Ctrl + C] bất cứ lúc nào để DỪNG VÀ XUẤT FILE")
                print("=" * 60 + "\n")

            # Đồng bộ thời gian thực: Đảm bảo độ dài video khớp 1:1 với đồng hồ thực tế
            elapsed = now - self.start_time
            target_frames = int(elapsed * self.fps)
            frames_to_write = max(1, target_frames - self.total_written_frames)

            for _ in range(frames_to_write):
                self.writer.write(frame)

            self.total_written_frames += frames_to_write
            self.raw_received_frames += 1

            if now - self.last_stat_time >= 1.0:
                real_fps = self.raw_received_frames / elapsed if elapsed > 0 else 0
                file_size_mb = os.path.getsize(self.output_path) / (1024 * 1024) if os.path.exists(self.output_path) else 0

                mins = int(elapsed // 60)
                secs = int(elapsed % 60)
                print(
                    f"\r🔴 Đang ghi: {mins:02d}:{secs:02d} | "
                    f"Camera: {real_fps:.1f} fps | "
                    f"Video: {self.total_written_frames} frames | "
                    f"Dung lượng: {file_size_mb:.1f} MB   ",
                    end="", flush=True
                )
                self.last_stat_time = now

        except Exception as e:
            self.get_logger().error(f"Lỗi ghi frame: {e}", throttle_duration_sec=5.0)

    def close(self):
        if self.writer is not None:
            self.writer.release()
            self.writer = None
            file_size_mb = os.path.getsize(self.output_path) / (1024 * 1024) if os.path.exists(self.output_path) else 0
            elapsed = time.time() - self.start_time if self.start_time else 0
            print("\n\n" + "=" * 60)
            print("✅ ĐÃ HOÀN TẤT VÀ LƯU VIDEO THÀNH CÔNG!")
            print(f"📁 File: {self.output_path}")
            print(f"⏱️ Thời lượng: {int(elapsed//60):02d}:{int(elapsed%60):02d} ({self.frame_count} frames)")
            print(f"📦 Dung lượng: {file_size_mb:.2f} MB")
            print("=" * 60 + "\n")


def print_transfer_guides(video_path):
    """In hướng dẫn chuyển video về Laptop nhanh nhất."""
    filename = os.path.basename(video_path)
    dirname = os.path.dirname(video_path)

    # Tự động phát hiện IP Laptop nếu đang SSH vào Pi
    ssh_client = os.environ.get("SSH_CLIENT", "")
    laptop_ip = ssh_client.split()[0] if ssh_client else "<IP_LAPTOP>"

    print("📤 CÁC CÁCH CHUYỂN VIDEO XUỐNG LAPTOP:")
    print("-" * 60)
    print("👉 CÁCH 1: Kéo file từ Laptop (Chạy lệnh này trên Terminal LAPTOP):")
    print(f"   scp bao@$(hostname -I | awk '{{print $1}}'):{video_path} ~/Downloads/\n")

    print("👉 CÁCH 2: Bắn file từ Pi sang Laptop (Chạy lệnh này ngay trên PI):")
    print(f"   scp {video_path} vinh@{laptop_ip}:~/Downloads/\n")

    print("👉 CÁCH 3: Tải qua trình duyệt Web (Không cần mật khẩu SSH):")
    print(f"   Trên Pi chạy:  python3 -m http.server 8080 --directory \"{dirname}\"")
    print(f"   Trên Laptop:   Mở trình duyệt gõ ->  http://$(hostname -I | awk '{{print $1}}'):8080/{filename}")
    print("-" * 60 + "\n")


def main():
    parser = argparse.ArgumentParser(description="Công cụ quay video Dataset trên Raspberry Pi")
    parser.add_argument('filename', nargs='?', default=None, help="Tên file video (tùy chọn)")
    parser.add_argument('--topic', default='/camera/color/image_raw', help="Topic camera ROS 2")
    parser.add_argument('--dir', default=None, help="Thư mục lưu video")
    args = parser.parse_args()

    # Xác định thư mục lưu recordings/
    script_dir = os.path.dirname(os.path.abspath(__file__))
    ws_dir = os.path.abspath(os.path.join(script_dir, '..'))
    record_dir = args.dir or os.path.join(ws_dir, 'recordings')
    os.makedirs(record_dir, exist_ok=True)

    # Đặt tên file video
    if args.filename:
        base_name = args.filename
        if not base_name.endswith('.mp4'):
            base_name += '.mp4'
    else:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        base_name = f"dataset_video_{timestamp}.mp4"

    output_path = os.path.join(record_dir, base_name)

    rclpy.init()
    node = VideoRecorderNode(output_path, args.topic)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

        if os.path.exists(output_path) and os.path.getsize(output_path) > 1024:
            print_transfer_guides(output_path)


if __name__ == '__main__':
    main()
