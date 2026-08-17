#!/usr/bin/env python3
"""
Custom ROS 2 Map Saver Tool for Agricultural SLAM.
Saves /map topic to .pgm image and .yaml metadata file (Nav2 / ROS standard).
Usage:
  ros2 run my_robot_slam map_saver -f my_corn_farm_map
"""

import sys
import os
import argparse
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import OccupancyGrid
import numpy as np


class MapSaverNode(Node):
    def __init__(self, filename="my_corn_farm_map"):
        super().__init__('map_saver_node')
        self.filename = filename
        self.map_saved = False

        qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos
        )
        self.get_logger().info("Đang chờ nhận dữ liệu từ topic /map...")

    def map_callback(self, msg: OccupancyGrid):
        if self.map_saved:
            return

        w = msg.info.width
        h = msg.info.height
        res = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        origin_z = msg.info.origin.position.z

        if w == 0 or h == 0:
            self.get_logger().warn("Bản đồ trống (width=0 hoặc height=0). Tiếp tục đợi...")
            return

        self.get_logger().info(f"Đã nhận bản đồ: kích thước {w}x{h}, độ phân giải {res:.3f}m/cell")

        # Map conversion to PGM (8-bit grayscale)
        # Standard ROS map format:
        # 0 (free) -> 254 (white)
        # 100 (occupied) -> 0 (black)
        # -1 (unknown) -> 205 (gray)
        raw_data = np.array(msg.data, dtype=np.int8).reshape((h, w))
        
        pgm_data = np.zeros((h, w), dtype=np.uint8)
        pgm_data[raw_data == 0] = 254
        pgm_data[raw_data >= 50] = 0
        pgm_data[raw_data < 0] = 205
        
        # PGM image coordinate system: row 0 is top, in ROS row 0 is bottom (flip vertically)
        pgm_data = np.flipud(pgm_data)

        # File paths
        pgm_filename = f"{self.filename}.pgm"
        yaml_filename = f"{self.filename}.yaml"

        # Write PGM (P5 binary)
        with open(pgm_filename, 'wb') as f:
            header = f"P5\n# CREATED BY my_robot_slam map_saver\n{w} {h}\n255\n".encode('ascii')
            f.write(header)
            f.write(pgm_data.tobytes())

        # Write YAML metadata
        yaml_content = f"""image: {os.path.basename(pgm_filename)}
mode: trinary
resolution: {res}
origin: [{origin_x:.6f}, {origin_y:.6f}, {origin_z:.6f}]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
"""
        with open(yaml_filename, 'w') as f:
            f.write(yaml_content)

        self.get_logger().info(f"✅ ĐÃ LƯU BẢN ĐỒ THÀNH CÔNG!")
        self.get_logger().info(f"  - File ảnh: {os.path.abspath(pgm_filename)}")
        self.get_logger().info(f"  - File cấu hình: {os.path.abspath(yaml_filename)}")
        self.map_saved = True


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description="Save ROS 2 /map to .pgm and .yaml")
    parser.add_argument('-f', '--filename', default='my_corn_farm_map', help='Output map filename prefix')
    parsed_args, unknown = parser.parse_known_args()

    node = MapSaverNode(filename=parsed_args.filename)
    
    timeout_sec = 10.0
    start_time = node.get_clock().now()
    
    while rclpy.ok() and not node.map_saved:
        rclpy.spin_once(node, timeout_sec=0.5)
        elapsed = (node.get_clock().now() - start_time).nanoseconds / 1e9
        if elapsed > timeout_sec and not node.map_saved:
            node.get_logger().error(f"Hết thời gian chờ {timeout_sec}s mà không nhận được dữ liệu từ /map. Hãy chắc chắn SLAM đang chạy!")
            break

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
