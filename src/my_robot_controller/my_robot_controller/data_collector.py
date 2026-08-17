#!/usr/bin/env python3
"""
ROS 2 Node: Data Collector
Tự động đăng ký topic /camera/image_raw và lưu ảnh thu thập vào thư mục `ros2-imgs`.
Tự động đăng ký topic /localization/gps hoặc /gps/fix và lưu thông tin tọa độ GPS chuẩn WGS-84 vào file `metadata.csv`.
Mỗi 0.5s lấy 1 frame với định dạng tên `ros2-farmeXX.png` (hoặc `ros2-farme1.png`, `ros2-farme2.png`,...).
"""

import os
import time
import csv
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix
from geometry_msgs.msg import Twist


class DataCollectorNode(Node):
    def __init__(self):
        super().__init__('data_collector_node')

        default_dir = os.path.join(
            os.path.expanduser('~'),
            'Màn hình nền', 'Luanvan', 'Luan van', 'ros2-imgs'
        )
        self.declare_parameter('output_dir', default_dir)
        self.declare_parameter('save_interval', 0.5)       # Mỗi 0.5s thu 1 frame
        self.declare_parameter('file_prefix', 'ros2-farme')  # Tiền tố tên ảnh: ros2-farmexx.png
        self.declare_parameter('only_when_moving', True)   # Chỉ lưu khi xe đang di chuyển
        self.declare_parameter('gps_topic', '/localization/gps')

        self.output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self.save_interval = self.get_parameter('save_interval').get_parameter_value().double_value
        self.file_prefix = self.get_parameter('file_prefix').get_parameter_value().string_value
        self.only_when_moving = self.get_parameter('only_when_moving').get_parameter_value().bool_value
        self.gps_topic = self.get_parameter('gps_topic').get_parameter_value().string_value

        os.makedirs(self.output_dir, exist_ok=True)
        from datetime import datetime
        timestamp_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.csv_path = os.path.join(self.output_dir, f'metadata_{timestamp_str}.csv')
        self.init_csv()

        self.last_save_time = 0.0
        self.image_count = 0
        self.is_moving = True  # Coi như đang di chuyển nếu chưa nhận cmd_vel

        self.current_lat = float('nan')
        self.current_lon = float('nan')
        self.current_alt = float('nan')

        # Subscriptions
        self.sub_image = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.sub_cmd_vel = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.sub_gps = self.create_subscription(
            NavSatFix,
            self.gps_topic,
            self.gps_callback,
            10
        )

        self.get_logger().info(f"[Data Collector] Node đã sẵn sàng!")
        self.get_logger().info(f"[Data Collector] Thư mục lưu ảnh: {self.output_dir}")
        self.get_logger().info(f"[Data Collector] File metadata GPS: {self.csv_path}")
        self.get_logger().info(f"[Data Collector] Chu kỳ lưu: {self.save_interval}s/frame | Tiền tố: {self.file_prefix}")

    def init_csv(self):
        if not os.path.exists(self.csv_path):
            with open(self.csv_path, mode='w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['filename', 'timestamp', 'latitude', 'longitude', 'altitude'])

    def cmd_vel_callback(self, msg: Twist):
        moving = (abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01)
        self.is_moving = moving

    def gps_callback(self, msg: NavSatFix):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.current_alt = msg.altitude

    def image_callback(self, msg: Image):
        current_time = time.time()
        if (current_time - self.last_save_time) < self.save_interval:
            return

        if self.only_when_moving and not self.is_moving:
            return

        try:
            height = msg.height
            width = msg.width
            encoding = msg.encoding

            if encoding in ['rgb8', 'bgr8']:
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, 3))
                if encoding == 'rgb8':
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            elif encoding in ['mono8']:
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width))
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            else:
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, -1))
                if img.shape[2] == 4:
                    img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)

            self.image_count += 1
            # Tên file dạng: ros2-farme1.png, ros2-farme2.png,...
            filename = f"{self.file_prefix}{self.image_count}.png"
            filepath = os.path.join(self.output_dir, filename)

            cv2.imwrite(filepath, img)
            self.last_save_time = current_time

            # Log metadata GPS
            with open(self.csv_path, mode='a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    filename,
                    f"{current_time:.3f}",
                    f"{self.current_lat:.8f}",
                    f"{self.current_lon:.8f}",
                    f"{self.current_alt:.3f}"
                ])

            self.get_logger().debug(f"[Data Collector] Saved frame {self.image_count}: {filename} | GPS: ({self.current_lat:.6f}, {self.current_lon:.6f})")

        except Exception as e:
            self.get_logger().error(f"[Data Collector] Lỗi khi xử lý ảnh: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = DataCollectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
