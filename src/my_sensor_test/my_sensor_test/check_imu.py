#!/usr/bin/env python3
"""
==============================================================================
  Công cụ kiểm tra & hiệu chuẩn trực tiếp Cảm biến IMU (ICM-20948 + Madgwick)
  
  Mục đích:
  - Lắng nghe topic /imu/data và /imu/data_raw
  - Chuyển đổi Quaternion (x, y, z, w) sang Euler (Roll, Pitch, Yaw)
  - Đo đạc độ lệch tĩnh (Drift / Bias) khi xe đứng yên
  - Kiểm tra chiều quay chuẩn của Robot (REP-103):
      + Quay Trái: Gyro Z > 0 (+), Yaw tăng
      + Quay Phải: Gyro Z < 0 (-), Yaw giảm
      + Xe tiến: Accel X > 0 (+)
      + Đứng yên: Accel Z ≈ +9.81 m/s² (Trọng lực), Roll/Pitch ≈ 0°
==============================================================================
"""

import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


def euler_from_quaternion(x, y, z, w):
    """
    Chuyển đổi Quaternion (x, y, z, w) sang góc Euler (Roll, Pitch, Yaw) tính bằng độ (°).
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class ImuCheckerNode(Node):
    def __init__(self):
        super().__init__('check_imu')

        self.declare_parameter('imu_topic', '/imu/data')
        self.imu_topic = self.get_parameter('imu_topic').value

        self.sub_imu = self.create_subscription(Imu, self.imu_topic, self.imu_callback, 10)

        self.msg_count = 0
        self.last_print_time = 0.0

        # Lịch sử để đo độ trôi góc Yaw (Drift Rate)
        self.initial_yaw = None
        self.start_time = None
        self.latest_yaw = 0.0
        self.latest_pitch = 0.0
        self.latest_roll = 0.0
        self.latest_gz = 0.0
        self.latest_ax = 0.0
        self.latest_ay = 0.0
        self.latest_az = 0.0

        print("\n" + "=" * 70)
        print("  🧭 CÔNG CỤ KIỂM TRA & HIỆU CHUẨN CẢM BIẾN IMU 9-TRỤC (ICM-20948)")
        print(f"  Đang lắng nghe topic: {self.imu_topic} ...")
        print("=" * 70)

    def imu_callback(self, msg: Imu):
        self.msg_count += 1
        now = time.time()

        # Lấy quaternion
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        roll, pitch, yaw = euler_from_quaternion(qx, qy, qz, qw)
        self.latest_roll = roll
        self.latest_pitch = pitch
        self.latest_yaw = yaw

        # Tốc độ góc (rad/s)
        self.latest_gz = msg.angular_velocity.z
        gz_deg = math.degrees(self.latest_gz)

        # Gia tốc (m/s^2)
        self.latest_ax = msg.linear_acceleration.x
        self.latest_ay = msg.linear_acceleration.y
        self.latest_az = msg.linear_acceleration.z

        if self.initial_yaw is None:
            self.initial_yaw = yaw
            self.start_time = now

        # In kết quả với tần số 4 Hz (mỗi 0.25 giây)
        if now - self.last_print_time >= 0.25:
            self.last_print_time = now
            elapsed = now - self.start_time if self.start_time else 1.0
            drift = yaw - self.initial_yaw
            # Chuẩn hóa drift trong [-180, 180]
            if drift > 180.0:
                drift -= 360.0
            elif drift < -180.0:
                drift += 360.0
            drift_rate = abs(drift) / max(1.0, elapsed) * 60.0  # Độ trôi mỗi phút (°/phút)

            # Đánh giá trạng thái chuyển động
            if abs(self.latest_gz) < 0.05 and abs(self.latest_ax) < 0.3:
                status = "🟢 ĐỨNG YÊN (Tĩnh)"
            elif self.latest_gz > 0.08:
                status = "🔄 ĐANG QUAY TRÁI (CCW > 0)"
            elif self.latest_gz < -0.08:
                status = "🔄 ĐANG QUAY PHẢI (CW < 0)"
            elif self.latest_ax > 0.3:
                status = "⏩ ĐANG TIẾN THẲNG"
            elif self.latest_ax < -0.3:
                status = "⏪ ĐANG LÙI THẲNG"
            else:
                status = "🟡 CHUYỂN ĐỘNG NHẸ"

            # In bảng điều khiển trực tiếp
            print("\033[H\033[J", end="")  # Xóa màn hình terminal
            print("=" * 70)
            print("       🧭 KIỂM TRA ĐỊNH HƯỚNG IMU (ICM-20948 + MADGWICK FILTER)")
            print("=" * 70)
            print(f"  Topic: {self.imu_topic} | Số mẫu nhận: {self.msg_count} | Thời gian chạy: {elapsed:.1f}s")
            print("-" * 70)
            print(f"  [GÓC HƯỚNG QUAY]  Yaw (Z):    {yaw:+7.2f}°  (Chuẩn: Quay Trái tăng, Quay Phải giảm)")
            print(f"  [GÓC CHÚC ĐẦU]    Pitch (Y):  {pitch:+7.2f}°  (Nghiêng dốc trước/sau)")
            print(f"  [GÓC NGHIÊNG XE]  Roll (X):   {roll:+7.2f}°  (Nghiêng vè trái/phải)")
            print("-" * 70)
            print(f"  [TỐC ĐỘ GÓC Z]    Gyro Z:     {self.latest_gz:+7.4f} rad/s ({gz_deg:+6.1f}°/s)")
            print(f"  [GIA TỐC THẲNG]   Accel X:    {self.latest_ax:+7.2f} m/s² (Tiến/Lùi)")
            print(f"                    Accel Y:    {self.latest_ay:+7.2f} m/s² (Ngang)")
            print(f"                    Accel Z:    {self.latest_az:+7.2f} m/s² (Chuẩn trọng lực ~9.81)")
            print("-" * 70)
            print(f"  TRẠNG THÁI: {status}")
            print(f"  Độ trôi tĩnh Yaw: {drift:+5.1f}° | Tốc độ trôi: {drift_rate:.2f}°/phút")
            print("=" * 70)
            print("  👉 HƯỚNG DẪN KIỂM TRA 4 BƯỚC:")
            print("     1. Để xe đứng yên: Kiểm tra Gyro Z ≈ 0.000 rad/s, Accel Z ≈ 9.81 m/s²")
            print("     2. Đẩy xe tiến:    Kiểm tra Accel X dương (+), Yaw không đổi")
            print("     3. Xoay xe sang TRÁI: Kiểm tra Yaw TĂNG (+) và Gyro Z > 0")
            print("     4. Xoay xe sang PHẢI: Kiểm tra Yaw GIẢM (-) và Gyro Z < 0")
            print("=" * 70)


def main(args=None):
    rclpy.init(args=args)
    node = ImuCheckerNode()
    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
