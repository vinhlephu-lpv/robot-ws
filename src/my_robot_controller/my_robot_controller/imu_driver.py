#!/usr/bin/env python3
"""
ROS 2 Hardware IMU Driver Node for ICM-20948 (9-DoF I2C Motion Sensor).
Đọc dữ liệu Gia tốc (Accel), Vận tốc góc (Gyro) và Từ kế (Mag) từ module ICM-20948
gắn trên chân I2C của Raspberry Pi (SDA Pin 3, SCL Pin 5).

Tính năng:
1. Giao tiếp I2C Native (hoạt động độc lập không cần thư viện ngoài) hoặc qua smbus2.
2. Tự động chuyển đổi Bank 0 / Bank 2 và cấu hình bộ lọc chống nhiễu DLPF.
3. Tự động cân bằng Zero-Bias Gyroscope khi vừa khởi động.
4. Bộ lọc dung hợp Complementary Orientation Filter tính toán góc nghiêng Roll, Pitch, Yaw
   và chuyển đổi sang Quaternion chuẩn sensor_msgs/msg/Imu.
5. Phát dữ liệu lên topic /imu @ 50 Hz.
"""

import os
import math
import time
import struct
import fcntl
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion


# ── ICM-20948 I2C Registers & Constants ─────────────────────────────────
I2C_SLAVE = 0x0703

# Common Register
REG_BANK_SEL = 0x7F

# Bank 0 Registers
WHO_AM_I     = 0x00
USER_CTRL    = 0x03
PWR_MGMT_1   = 0x06
PWR_MGMT_2   = 0x07
INT_PIN_CFG  = 0x0F
ACCEL_XOUT_H = 0x2D
GYRO_XOUT_H  = 0x33
TEMP_OUT_H   = 0x39

# Bank 2 Registers
GYRO_CONFIG_1 = 0x01
ACCEL_CONFIG  = 0x14

# Conversion Constants
GRAVITY_MSS   = 9.80665
DEG_TO_RAD    = math.pi / 180.0


class I2CInterface:
    """Giao tiếp I2C cấp thấp chuẩn Linux /dev/i2c-X."""
    def __init__(self, bus_num=1, address=0x68):
        self.bus_num = bus_num
        self.address = address
        self.dev_path = f"/dev/i2c-{bus_num}"
        self.fd = None
        self.open()

    def open(self):
        try:
            if os.path.exists(self.dev_path):
                self.fd = os.open(self.dev_path, os.O_RDWR)
                fcntl.ioctl(self.fd, I2C_SLAVE, self.address)
                return True
        except Exception:
            self.fd = None
        return False

    def is_open(self):
        return self.fd is not None

    def write_byte(self, reg, val):
        if self.fd is None:
            return False
        try:
            os.write(self.fd, bytes([reg, val & 0xFF]))
            return True
        except Exception:
            return False

    def read_bytes(self, reg, length):
        if self.fd is None:
            return None
        try:
            os.write(self.fd, bytes([reg]))
            data = os.read(self.fd, length)
            return data if len(data) == length else None
        except Exception:
            return None

    def close(self):
        if self.fd is not None:
            try:
                os.close(self.fd)
            except Exception:
                pass
            self.fd = None


class ICM20948Driver:
    """Driver điều khiển cảm biến 9 trục ICM-20948."""
    def __init__(self, i2c: I2CInterface):
        self.i2c = i2c
        self.accel_scale = 8192.0   # ±4g -> 8192 LSB/g
        self.gyro_scale = 65.5      # ±500 dps -> 65.5 LSB/dps
        self.is_initialized = False

    def select_bank(self, bank):
        """Chuyển đổi User Bank (0, 1, 2, 3)."""
        return self.i2c.write_byte(REG_BANK_SEL, (bank & 0x03) << 4)

    def initialize(self):
        if not self.i2c.is_open() and not self.i2c.open():
            return False

        # 1. Reset chip
        self.select_bank(0)
        self.i2c.write_byte(PWR_MGMT_1, 0x80)
        time.sleep(0.05)

        # 2. Wake up & Auto Clock Select
        self.select_bank(0)
        self.i2c.write_byte(PWR_MGMT_1, 0x01)
        time.sleep(0.02)

        # 3. Enable Accelerometer & Gyroscope
        self.i2c.write_byte(PWR_MGMT_2, 0x00)

        # 4. Kiểm tra WHO_AM_I (0xEA hoặc 0x98)
        data = self.i2c.read_bytes(WHO_AM_I, 1)
        if not data or data[0] not in (0xEA, 0x98):
            return False

        # 5. Cấu hình Bank 2: Gyro ±500 dps, DLPF 3
        self.select_bank(2)
        self.i2c.write_byte(GYRO_CONFIG_1, (0x01 << 1) | 0x01)  # ±500 dps, DLPF on

        # 6. Cấu hình Bank 2: Accel ±4g, DLPF 3
        self.i2c.write_byte(ACCEL_CONFIG, (0x01 << 1) | 0x01)   # ±4g, DLPF on

        # 7. Quay lại Bank 0 để đọc dữ liệu
        self.select_bank(0)
        self.is_initialized = True
        return True

    def read_raw_sensors(self):
        """Đọc đồng thời 12 byte Gia tốc và Vận tốc góc từ Bank 0."""
        if not self.is_initialized:
            return None

        # Đọc 12 bytes liên tục từ ACCEL_XOUT_H (0x2D) đến GYRO_ZOUT_L (0x38)
        data = self.i2c.read_bytes(ACCEL_XOUT_H, 12)
        if not data or len(data) < 12:
            return None

        # Unpack 6 số nguyên 16-bit có dấu (Big Endian)
        ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw = struct.unpack('>6h', data)

        # Chuyển đổi sang đơn vị chuẩn SI
        # Accel: m/s^2
        ax = (ax_raw / self.accel_scale) * GRAVITY_MSS
        ay = (ay_raw / self.accel_scale) * GRAVITY_MSS
        az = (az_raw / self.accel_scale) * GRAVITY_MSS

        # Gyro: rad/s
        gx = (gx_raw / self.gyro_scale) * DEG_TO_RAD
        gy = (gy_raw / self.gyro_scale) * DEG_TO_RAD
        gz = (gz_raw / self.gyro_scale) * DEG_TO_RAD

        return ax, ay, az, gx, gy, gz


class ImuDriverNode(Node):
    def __init__(self):
        super().__init__('imu_driver_node')

        # Parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x68)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_topic', '/imu/data')
        self.declare_parameter('raw_topic', '/imu/data_raw')
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('calibrate_samples', 60)

        self.bus_num = int(self.get_parameter('i2c_bus').value)
        self.address = int(self.get_parameter('i2c_address').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_topic = self.get_parameter('publish_topic').value
        self.raw_topic = self.get_parameter('raw_topic').value
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.calib_target = int(self.get_parameter('calibrate_samples').value)

        # Publishers (Cả dữ liệu thô cho Madgwick và dữ liệu nội suy)
        self.imu_pub = self.create_publisher(Imu, self.publish_topic, 10)
        self.imu_raw_pub = self.create_publisher(Imu, self.raw_topic, 10)

        # I2C & Driver
        self.i2c = I2CInterface(bus_num=self.bus_num, address=self.address)
        self.sensor = ICM20948Driver(self.i2c)

        # Calibration State
        self.calibrating = True
        self.calib_count = 0
        self.gyro_bias_x = 0.0
        self.gyro_bias_y = 0.0
        self.gyro_bias_z = 0.0

        # Orientation Filter State (Euler -> Quaternion)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.alpha = 0.96  # Complementary filter weight (96% gyro, 4% gravity)
        self.last_time = time.time()
        self.last_log_time = 0.0

        # Connect to Hardware
        if self.sensor.initialize():
            self.get_logger().info("=" * 60)
            self.get_logger().info(f"🚀 [ICM-20948 IMU] ĐÃ KẾT NỐI THÀNH CÔNG!")
            self.get_logger().info(f"📍 I2C: /dev/i2c-{self.bus_num} @ 0x{self.address:02X} | Topic: {self.publish_topic}")
            self.get_logger().info(f"🔄 Đang tự động hiệu chuẩn Zero-Bias Gyroscope ({self.calib_target} mẫu)...")
            self.get_logger().info("=" * 60)
        else:
            self.get_logger().warn(
                f"⚠️ Không tìm thấy cảm biến ICM-20948 tại /dev/i2c-{self.bus_num} (0x{self.address:02X}).\n"
                f"   Kiểm tra chân cắm: 3.3V, GND, SDA (Pin 3), SCL (Pin 5) hoặc lệnh 'sudo i2cdetect -y 1'."
            )

        # Timer Loop (50 Hz)
        period = 1.0 / max(1.0, self.rate_hz)
        self.timer = self.create_timer(period, self.timer_callback)

    def timer_callback(self):
        raw = self.sensor.read_raw_sensors()
        if raw is None:
            # Thử khởi tạo lại định kỳ nếu mất kết nối
            if not self.sensor.is_initialized:
                self.sensor.initialize()
            return

        ax, ay, az, gx, gy, gz = raw
        now = time.time()
        dt = max(0.001, min(0.1, now - self.last_time))
        self.last_time = now

        # Giai đoạn 1: Hiệu chuẩn Gyro Zero-Bias khi đứng yên ban đầu
        if self.calibrating:
            self.gyro_bias_x += gx
            self.gyro_bias_y += gy
            self.gyro_bias_z += gz
            self.calib_count += 1

            if self.calib_count >= self.calib_target:
                self.gyro_bias_x /= self.calib_count
                self.gyro_bias_y /= self.calib_count
                self.gyro_bias_z /= self.calib_count
                self.calibrating = False

                # Khởi tạo góc nghiêng ban đầu từ gia tốc kế
                self.roll = math.atan2(ay, az)
                self.pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
                self.get_logger().info(
                    f"✅ [ICM-20948] HIỆU CHUẨN XONG! Bias Gyro (deg/s): "
                    f"X={self.gyro_bias_x/DEG_TO_RAD:.2f}, Y={self.gyro_bias_y/DEG_TO_RAD:.2f}, Z={self.gyro_bias_z/DEG_TO_RAD:.2f}"
                )
            return

        # Giai đoạn 2: Trừ Zero Bias
        gx_corr = gx - self.gyro_bias_x
        gy_corr = gy - self.gyro_bias_y
        gz_corr = gz - self.gyro_bias_z

        # Tính góc nghiêng tĩnh từ gia tốc trọng trường
        roll_acc = math.atan2(ay, az)
        pitch_acc = math.atan2(-ax, math.sqrt(ay * ay + az * az))

        # Bộ lọc dung hợp Complementary Filter
        self.roll = self.alpha * (self.roll + gx_corr * dt) + (1.0 - self.alpha) * roll_acc
        self.pitch = self.alpha * (self.pitch + gy_corr * dt) + (1.0 - self.alpha) * pitch_acc
        self.yaw += gz_corr * dt

        # Chuyển đổi Euler (Roll, Pitch, Yaw) sang Quaternion chuẩn ROS
        cy = math.cos(self.yaw * 0.5)
        sy = math.sin(self.yaw * 0.5)
        cp = math.cos(self.pitch * 0.5)
        sp = math.sin(self.pitch * 0.5)
        cr = math.cos(self.roll * 0.5)
        sr = math.sin(self.roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        # 1. Publish Raw IMU message cho bộ lọc ngoài (Madgwick AHRS Filter)
        now_msg = self.get_clock().now().to_msg()
        raw_msg = Imu()
        raw_msg.header.stamp = now_msg
        raw_msg.header.frame_id = self.frame_id
        # Orientation unknown -> orientation_covariance[0] = -1
        raw_msg.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        raw_msg.angular_velocity.x = gx_corr
        raw_msg.angular_velocity.y = gy_corr
        raw_msg.angular_velocity.z = gz_corr
        raw_msg.angular_velocity_covariance = [
            0.0001, 0.0, 0.0,
            0.0, 0.0001, 0.0,
            0.0, 0.0, 0.0001
        ]
        raw_msg.linear_acceleration.x = ax
        raw_msg.linear_acceleration.y = ay
        raw_msg.linear_acceleration.z = az
        raw_msg.linear_acceleration_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]
        self.imu_raw_pub.publish(raw_msg)

        # 2. Publish Internal Filtered IMU message (Dự phòng Complementary Filter)
        msg = Imu()
        msg.header.stamp = now_msg
        msg.header.frame_id = self.frame_id

        msg.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        msg.orientation_covariance = [
            0.001, 0.0, 0.0,
            0.0, 0.001, 0.0,
            0.0, 0.0, 0.005
        ]

        msg.angular_velocity.x = gx_corr
        msg.angular_velocity.y = gy_corr
        msg.angular_velocity.z = gz_corr
        msg.angular_velocity_covariance = [
            0.0001, 0.0, 0.0,
            0.0, 0.0001, 0.0,
            0.0, 0.0, 0.0001
        ]

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.linear_acceleration_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]

        self.imu_pub.publish(msg)

        # Định kỳ in trạng thái trực quan ra màn hình terminal (4 Hz) để người dùng theo dõi
        if (now - self.last_log_time) >= 0.25:
            self.last_log_time = now
            roll_deg = self.roll / DEG_TO_RAD
            pitch_deg = self.pitch / DEG_TO_RAD
            yaw_deg = self.yaw / DEG_TO_RAD
            self.get_logger().info(
                f"📐 [IMU Live] Nghiêng (Roll): {roll_deg:+6.1f}° | Dốc (Pitch): {pitch_deg:+6.1f}° | Hướng (Yaw): {yaw_deg:+6.1f}° | "
                f"Gia tốc: ({ax:+5.2f}, {ay:+5.2f}, {az:+5.2f}) m/s²"
            )

    def destroy_node(self):
        if self.i2c:
            self.i2c.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImuDriverNode()
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
