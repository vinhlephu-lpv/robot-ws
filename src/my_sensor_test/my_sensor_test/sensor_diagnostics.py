#!/usr/bin/env python3
"""
Sensor Diagnostics Node for Camera (Orbbec Astra 3D / USB Webcam) & LiDAR
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

        # Subscribers for Color and Depth images
        self.color_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self._color_callback, 10
        )
        self.raw_sub = self.create_subscription(
            Image, '/camera/image_raw', self._raw_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self._depth_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self._scan_callback, 10
        )

        # Color Image stats
        self.color_count = 0
        self.last_color_time = 0.0
        self.color_fps = 0.0
        self.color_width = 0
        self.color_height = 0
        self.color_encoding = "N/A"
        self.color_source_topic = "/camera/color/image_raw"

        # Depth Image stats
        self.depth_count = 0
        self.last_depth_time = 0.0
        self.depth_fps = 0.0
        self.depth_width = 0
        self.depth_height = 0
        self.depth_encoding = "N/A"

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

    def _color_callback(self, msg: Image):
        now = time.time()
        self.color_count += 1
        if self.last_color_time > 0:
            dt = now - self.last_color_time
            if dt > 0:
                instant_fps = 1.0 / dt
                self.color_fps = 0.8 * self.color_fps + 0.2 * instant_fps if self.color_fps > 0 else instant_fps
        self.last_color_time = now
        self.color_width = msg.width
        self.color_height = msg.height
        self.color_encoding = msg.encoding
        self.color_source_topic = "/camera/color/image_raw"

    def _raw_callback(self, msg: Image):
        if (time.time() - self.last_color_time) > 2.0:
            now = time.time()
            self.color_count += 1
            if self.last_color_time > 0:
                dt = now - self.last_color_time
                if dt > 0:
                    instant_fps = 1.0 / dt
                    self.color_fps = 0.8 * self.color_fps + 0.2 * instant_fps if self.color_fps > 0 else instant_fps
            self.last_color_time = now
            self.color_width = msg.width
            self.color_height = msg.height
            self.color_encoding = msg.encoding
            self.color_source_topic = "/camera/image_raw"

    def _depth_callback(self, msg: Image):
        now = time.time()
        self.depth_count += 1
        if self.last_depth_time > 0:
            dt = now - self.last_depth_time
            if dt > 0:
                instant_fps = 1.0 / dt
                self.depth_fps = 0.8 * self.depth_fps + 0.2 * instant_fps if self.depth_fps > 0 else instant_fps
        self.last_depth_time = now
        self.depth_width = msg.width
        self.depth_height = msg.height
        self.depth_encoding = msg.encoding

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
        serial_devs = sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*') + glob.glob('/dev/rplidar') + glob.glob('/dev/esp32'))

        # Check timeouts
        color_active = (now - self.last_color_time) < 2.0 and self.color_count > 0
        depth_active = (now - self.last_depth_time) < 2.0 and self.depth_count > 0
        lidar_active = (now - self.last_scan_time) < 2.0 and self.scan_count > 0

        # Terminal output
        os.system('clear' if os.name == 'posix' else 'cls')
        print("=" * 70)
        print("        ROS 2 SENSOR HARDWARE DIAGNOSTICS MONITOR")
        print("=" * 70)

        # 1. Hardware Ports Scan
        print("[1] CỔNG PHẦN CỨNG HỆ THỐNG:")
        print(f"  • Video Devices  : {', '.join(video_devs) if video_devs else 'KHÔNG TÌM THẤY (/dev/video*)'}")
        print(f"  • Serial Devices : {', '.join(serial_devs) if serial_devs else 'KHÔNG TÌM THẤY (/dev/ttyUSB* /dev/rplidar)'}")
        print("-" * 70)

        # 2. Camera RGB Status
        print(f"[2] TRẠNG THÁI CAMERA MÀU (RGB - {self.color_source_topic}):")
        if color_active:
            print(f"  • Trạng thái : \033[92m[OK - HOẠT ĐỘNG]\033[0m")
            print(f"  • Tốc độ khung hình: {self.color_fps:.1f} FPS")
            print(f"  • Kích thước ảnh   : {self.color_width} x {self.color_height} ({self.color_encoding})")
            print(f"  • Tổng frames nhận : {self.color_count}")
        else:
            print(f"  • Trạng thái : \033[91m[CHƯA CÓ DỮ LIỆU / MẤT TÍN HIỆU]\033[0m")
            print(f"  • Gợi ý      : Kiểm tra driver Astra Camera hoặc USB Webcam (/dev/video*)")
        print("-" * 70)

        # 3. Camera Depth Status (Astra 3D)
        print("[3] TRẠNG THÁI CAMERA ĐỘ SÂU (Depth - /camera/depth/image_raw):")
        if depth_active:
            print(f"  • Trạng thái : \033[92m[OK - HOẠT ĐỘNG (ORBBEC ASTRA 3D)]\033[0m")
            print(f"  • Tốc độ khung hình: {self.depth_fps:.1f} FPS")
            print(f"  • Kích thước ảnh   : {self.depth_width} x {self.depth_height} ({self.depth_encoding})")
            print(f"  • Tổng frames nhận : {self.depth_count}")
        else:
            print(f"  • Trạng thái : \033[93m[TẮT / CHƯA BẬT (Depth chỉ có trên Astra 3D)]\033[0m")
        print("-" * 70)

        # 4. LiDAR Status
        print("[4] TRẠNG THÁI LIDAR (/scan):")
        if lidar_active:
            print(f"  • Trạng thái : \033[92m[OK - HOẠT ĐỘNG]\033[0m")
            print(f"  • Tần số quét      : {self.scan_hz:.1f} Hz")
            print(f"  • Số điểm laser    : {self.scan_points} points/scan")
            print(f"  • Khoảng cách gần nhất : {self.scan_min_dist:.2f} m")
            print(f"  • Khoảng cách xa nhất  : {self.scan_max_dist:.2f} m")
            print(f"  • Tổng scans nhận  : {self.scan_count}")
        else:
            print(f"  • Trạng thái : \033[91m[CHƯA CÓ DỮ LIỆU / MẤT TÍN HIỆU]\033[0m")
            print(f"  • Gợi ý      : Kiểm tra cổng /dev/rplidar hoặc /dev/ttyUSB0 (chmod 666) & baudrate")
        print("=" * 70)
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
