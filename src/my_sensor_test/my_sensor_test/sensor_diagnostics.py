#!/usr/bin/env python3
"""
Sensor Diagnostics Node for Camera & LiDAR
Checks hardware ports, permissions, and measures real-time topic frequencies & health.
"""

import os
import glob
import time
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan


class SensorDiagnosticsNode(Node):
    def __init__(self):
        super().__init__('sensor_diagnostics_node')

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self._image_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self._scan_callback, 10
        )

        # Image stats
        self.img_count = 0
        self.last_img_time = 0.0
        self.img_fps = 0.0
        self.img_width = 0
        self.img_height = 0
        self.img_encoding = "N/A"

        # LiDAR stats
        self.scan_count = 0
        self.last_scan_time = 0.0
        self.scan_hz = 0.0
        self.scan_points = 0
        self.scan_min_dist = float('inf')
        self.scan_max_dist = 0.0

        # Periodic timer for terminal display
        self.timer = self.create_timer(1.0, self._render_status)
        self.start_time = time.time()

    def _image_callback(self, msg: Image):
        now = time.time()
        self.img_count += 1
        if self.last_img_time > 0:
            dt = now - self.last_img_time
            if dt > 0:
                instant_fps = 1.0 / dt
                self.img_fps = 0.8 * self.img_fps + 0.2 * instant_fps if self.img_fps > 0 else instant_fps
        self.last_img_time = now
        self.img_width = msg.width
        self.img_height = msg.height
        self.img_encoding = msg.encoding

    def _scan_callback(self, msg: LaserScan):
        now = time.time()
        self.scan_count += 1
        if self.last_scan_time > 0:
            dt = now - self.last_scan_time
            if dt > 0:
                instant_hz = 1.0 / dt
                self.scan_hz = 0.8 * self.scan_hz + 0.2 * instant_hz if self.scan_hz > 0 else instant_hz
        self.last_scan_time = now

        valid_ranges = [r for r in msg.ranges if not math.isnan(r) and not math.isinf(r) and r > 0.05]
        self.scan_points = len(valid_ranges)
        if valid_ranges:
            self.scan_min_dist = min(valid_ranges)
            self.scan_max_dist = max(valid_ranges)

    def _render_status(self):
        now = time.time()
        video_devs = sorted(glob.glob('/dev/video*'))
        serial_devs = sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*'))

        # Check timeouts
        camera_active = (now - self.last_img_time) < 2.0 and self.img_count > 0
        lidar_active = (now - self.last_scan_time) < 2.0 and self.scan_count > 0

        # Terminal output
        os.system('clear' if os.name == 'posix' else 'cls')
        print("=" * 65)
        print("        ROS 2 SENSOR HARDWARE DIAGNOSTICS MONITOR")
        print("=" * 65)

        # 1. Hardware Ports Scan
        print("[1] CỔNG PHẦN CỨNG HỆ THỐNG:")
        print(f"  • Video Devices  : {', '.join(video_devs) if video_devs else 'KHÔNG TÌM THẤY (/dev/video*)'}")
        print(f"  • Serial Devices : {', '.join(serial_devs) if serial_devs else 'KHÔNG TÌM THẤY (/dev/ttyUSB* /dev/ttyACM*)'}")
        print("-" * 65)

        # 2. Camera Status
        print("[2] TRẠNG THÁI CAMERA (/camera/image_raw):")
        if camera_active:
            print(f"  • Trạng thái : \033[92m[OK - HOẠT ĐỘNG]\033[0m")
            print(f"  • Tốc độ khung hình: {self.img_fps:.1f} FPS")
            print(f"  • Kích thước ảnh   : {self.img_width} x {self.img_height} ({self.img_encoding})")
            print(f"  • Tổng frames nhận : {self.img_count}")
        else:
            print(f"  • Trạng thái : \033[91m[CHƯA CÓ DỮ LIỆU / MẤT TÍN HIỆU]\033[0m")
            print(f"  • Gợi ý      : Kiểm tra driver camera hoặc quyền truy cập /dev/video0")
        print("-" * 65)

        # 3. LiDAR Status
        print("[3] TRẠNG THÁI LIDAR (/scan):")
        if lidar_active:
            print(f"  • Trạng thái : \033[92m[OK - HOẠT ĐỘNG]\033[0m")
            print(f"  • Tần số quét      : {self.scan_hz:.1f} Hz")
            print(f"  • Số điểm laser    : {self.scan_points} points/scan")
            print(f"  • Khoảng cách gần nhất : {self.scan_min_dist:.2f} m")
            print(f"  • Khoảng cách xa nhất  : {self.scan_max_dist:.2f} m")
            print(f"  • Tổng scans nhận  : {self.scan_count}")
        else:
            print(f"  • Trạng thái : \033[91m[CHƯA CÓ DỮ LIỆU / MẤT TÍN HIỆU]\033[0m")
            print(f"  • Gợi ý      : Kiểm tra cổng /dev/ttyUSB0 (chmod 666) & baudrate")
        print("=" * 65)
        print("Nhấn Ctrl+C để dừng giám sát.")


def main(args=None):
    rclpy.init(args=args)
    node = SensorDiagnosticsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
