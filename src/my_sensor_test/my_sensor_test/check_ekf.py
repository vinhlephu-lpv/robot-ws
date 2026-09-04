#!/usr/bin/env python3
"""
==============================================================================
  Công cụ CLI Giám sát Kiến trúc Dual EKF + NavSat Transform (REP-105)
  Hiển thị trực quan trạng thái 4 khối theo đúng sơ đồ:
    1. Cảm biến đầu vào: /imu/data & /wheel/odom
    2. EKF 1 (Local): /odometry/local & TF odom -> base_footprint
    3. Cầu nối GPS: /gps/fix -> /odometry/gps (navsat_transform)
    4. EKF 2 (Global): /odometry/global & TF map -> odom
==============================================================================
"""

import math
import os
import sys
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from nav_msgs.msg import Odometry
from std_msgs.msg import String as StringMsg


def quat_to_yaw_deg(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


class DualEkfMonitor(Node):
    def __init__(self):
        super().__init__('check_ekf_monitor')

        # Dữ liệu theo dõi
        self.imu_count = 0
        self.imu_hz = 0.0
        self.imu_last_time = time.time()
        self.imu_yaw = 0.0
        self.imu_wz = 0.0

        self.wheel_count = 0
        self.wheel_hz = 0.0
        self.wheel_last_time = time.time()
        self.wheel_vx = 0.0
        self.wheel_wz = 0.0
        self.wheel_status_str = "N/A"

        self.ekf1_count = 0
        self.ekf1_hz = 0.0
        self.ekf1_last_time = time.time()
        self.ekf1_x = 0.0
        self.ekf1_y = 0.0
        self.ekf1_yaw = 0.0
        self.ekf1_vx = 0.0
        self.ekf1_wz = 0.0

        self.gps_count = 0
        self.gps_hz = 0.0
        self.gps_last_time = time.time()
        self.gps_lat = 0.0
        self.gps_lon = 0.0
        self.gps_status = -1

        self.navsat_count = 0
        self.navsat_hz = 0.0
        self.navsat_last_time = time.time()
        self.navsat_x = 0.0
        self.navsat_y = 0.0

        self.ekf2_count = 0
        self.ekf2_hz = 0.0
        self.ekf2_last_time = time.time()
        self.ekf2_x = 0.0
        self.ekf2_y = 0.0
        self.ekf2_yaw = 0.0

        # Subscriptions
        self.create_subscription(Imu, '/imu/data', self.cb_imu, 10)
        self.create_subscription(Odometry, '/wheel/odom', self.cb_wheel, 10)
        self.create_subscription(Odometry, '/odom/raw', self.cb_wheel_raw, 10)
        self.create_subscription(StringMsg, '/wheel/status', self.cb_wheel_status, 10)
        self.create_subscription(Odometry, '/odometry/local', self.cb_ekf1, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.cb_ekf1_alt, 10)
        self.create_subscription(NavSatFix, '/gps/fix', self.cb_gps, 10)
        self.create_subscription(Odometry, '/odometry/gps', self.cb_navsat, 10)
        self.create_subscription(Odometry, '/odometry/global', self.cb_ekf2, 10)

        # Timer hiển thị 4Hz
        self.start_time = time.time()
        self.timer = self.create_timer(0.25, self.render_screen)

    def cb_imu(self, msg: Imu):
        self.imu_count += 1
        now = time.time()
        dt = now - self.imu_last_time
        if dt >= 0.5:
            self.imu_hz = self.imu_count / dt
            self.imu_count = 0
            self.imu_last_time = now
        self.imu_yaw = quat_to_yaw_deg(msg.orientation)
        self.imu_wz = msg.angular_velocity.z

    def cb_wheel(self, msg: Odometry):
        self.wheel_count += 1
        now = time.time()
        dt = now - self.wheel_last_time
        if dt >= 0.5:
            self.wheel_hz = self.wheel_count / dt
            self.wheel_count = 0
            self.wheel_last_time = now
        self.wheel_vx = msg.twist.twist.linear.x
        self.wheel_wz = msg.twist.twist.angular.z

    def cb_wheel_raw(self, msg: Odometry):
        if self.wheel_hz == 0.0:
            self.cb_wheel(msg)

    def cb_wheel_status(self, msg: StringMsg):
        self.wheel_status_str = msg.data

    def cb_ekf1(self, msg: Odometry):
        self.ekf1_count += 1
        now = time.time()
        dt = now - self.ekf1_last_time
        if dt >= 0.5:
            self.ekf1_hz = self.ekf1_count / dt
            self.ekf1_count = 0
            self.ekf1_last_time = now
        self.ekf1_x = msg.pose.pose.position.x
        self.ekf1_y = msg.pose.pose.position.y
        self.ekf1_yaw = quat_to_yaw_deg(msg.pose.pose.orientation)
        self.ekf1_vx = msg.twist.twist.linear.x
        self.ekf1_wz = msg.twist.twist.angular.z

    def cb_ekf1_alt(self, msg: Odometry):
        if self.ekf1_hz == 0.0:
            self.cb_ekf1(msg)

    def cb_gps(self, msg: NavSatFix):
        self.gps_count += 1
        now = time.time()
        dt = now - self.gps_last_time
        if dt >= 0.5:
            self.gps_hz = self.gps_count / dt
            self.gps_count = 0
            self.gps_last_time = now
        self.gps_lat = msg.latitude
        self.gps_lon = msg.longitude
        self.gps_status = msg.status.status

    def cb_navsat(self, msg: Odometry):
        self.navsat_count += 1
        now = time.time()
        dt = now - self.navsat_last_time
        if dt >= 0.5:
            self.navsat_hz = self.navsat_count / dt
            self.navsat_count = 0
            self.navsat_last_time = now
        self.navsat_x = msg.pose.pose.position.x
        self.navsat_y = msg.pose.pose.position.y

    def cb_ekf2(self, msg: Odometry):
        self.ekf2_count += 1
        now = time.time()
        dt = now - self.ekf2_last_time
        if dt >= 0.5:
            self.ekf2_hz = self.ekf2_count / dt
            self.ekf2_count = 0
            self.ekf2_last_time = now
        self.ekf2_x = msg.pose.pose.position.x
        self.ekf2_y = msg.pose.pose.position.y
        self.ekf2_yaw = quat_to_yaw_deg(msg.pose.pose.orientation)

    def render_screen(self):
        elapsed = time.time() - self.start_time

        # Format status strings
        imu_status = f"✅ ĐANG CHẠY ({self.imu_hz:4.1f} Hz)" if self.imu_hz > 5.0 else "⏳ CHỜ DỮ LIỆU"
        wheel_status = f"✅ ĐANG CHẠY ({self.wheel_hz:4.1f} Hz)" if self.wheel_hz > 2.0 else "⏳ CHỜ DỮ LIỆU"
        ekf1_status = f"✅ ĐANG DUNG HỢP ({self.ekf1_hz:4.1f} Hz)" if self.ekf1_hz > 5.0 else "⏳ CHỜ DỮ LIỆU"

        if self.gps_status >= 0:
            gps_str = f"✅ FIX ({self.gps_hz:4.1f} Hz)"
        elif self.gps_hz > 0.0:
            gps_str = f"⚠️ NO FIX ({self.gps_hz:4.1f} Hz)"
        else:
            gps_str = "⏳ CHƯA BẬT / CHỜ GPS"

        navsat_str = f"✅ ĐANG TÍNH ({self.navsat_hz:4.1f} Hz)" if self.navsat_hz > 2.0 else "⏳ CHỜ GPS FIX"
        ekf2_status = f"✅ ĐANG DUNG HỢP ({self.ekf2_hz:4.1f} Hz)" if self.ekf2_hz > 5.0 else "⏳ CHƯA BẬT (enable_gps:=false)"

        out = [
            "\033[2J\033[H",  # Clear màn hình
            "======================================================================",
            f"  🤖 GIÁM SÁT KIẾN TRÚC ĐỊNH VỊ DUAL EKF (REP-105) | Thời gian: {elapsed:4.1f}s",
            "======================================================================",
            f" [1. CẢM BIẾN ĐẦU VÀO]",
            f"   • IMU Madgwick (/imu/data)   : {imu_status}",
            f"     -> Hướng Yaw: {self.imu_yaw:+6.1f}° | Vận tốc góc Wz: {self.imu_wz:+5.2f} rad/s",
            f"   • 4 Bánh xe (/wheel/odom)    : {wheel_status}",
            f"     -> Vận tốc tiến Vx: {self.wheel_vx:+5.2f} m/s | Quay Wz: {self.wheel_wz:+5.2f} rad/s",
            f"     -> Trạng thái 4 bánh: {self.wheel_status_str}",
            "----------------------------------------------------------------------",
            f" [2. TẦNG CỤC BỘ - EKF 1 LOCAL] (/odometry/local) -> TF: odom -> base_footprint",
            f"   • Trạng thái bộ lọc         : {ekf1_status}",
            f"   • Tọa độ Cục Bộ (Odom)      : X = {self.ekf1_x:+6.2f} m  | Y = {self.ekf1_y:+6.2f} m",
            f"   • Hướng & Tốc độ Dung hợp   : Yaw = {self.ekf1_yaw:+6.1f}° | Vx = {self.ekf1_vx:+5.2f} m/s | Wz = {self.ekf1_wz:+5.2f} rad/s",
            "----------------------------------------------------------------------",
            f" [3. CẦU NỐI GPS - NAVSAT TRANSFORM] (/odometry/gps)",
            f"   • Tín hiệu Vệ tinh (/gps/fix): {gps_str}",
            f"     -> Tọa độ Địa lý: Lat = {self.gps_lat:.7f}° | Lon = {self.gps_lon:.7f}°",
            f"   • Tọa độ Phẳng Descartes     : {navsat_str}",
            f"     -> Easting (X) = {self.navsat_x:+7.2f} m | Northing (Y) = {self.navsat_y:+7.2f} m",
            "----------------------------------------------------------------------",
            f" [4. TẦNG TOÀN CỤC - EKF 2 GLOBAL] (/odometry/global) -> TF: map -> odom",
            f"   • Trạng thái bộ lọc         : {ekf2_status}",
            f"   • Tọa độ Toàn Cục (Map)     : X = {self.ekf2_x:+6.2f} m  | Y = {self.ekf2_y:+6.2f} m",
            f"   • Hướng Xe Tuyệt Đối (Map)  : Yaw = {self.ekf2_yaw:+6.1f}°",
            "======================================================================",
            "  Nhấn Ctrl+C để thoát giám sát.",
        ]
        sys.stdout.write("\n".join(out) + "\n")
        sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    node = DualEkfMonitor()
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
            rclpy.shutdown()


if __name__ == '__main__':
    main()
