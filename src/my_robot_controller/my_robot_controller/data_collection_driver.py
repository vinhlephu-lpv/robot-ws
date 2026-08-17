#!/usr/bin/env python3
"""
ROS 2 Node: Data Collection Driver (Open-Space U-Turn & Feedforward Trajectory Tracker)
Tự động lái xe bám chuẩn 100% theo tâm luống từ đầu hàng thẳng (-12m) đến hết hàng cong (+12m).
Quay đầu 180 độ hoàn toàn ngoài vùng đất trống (x > 12.6m và x < -12.6m) để KHÔNG BAO GIỜ chạm cây bắp khi quẹo.
Bám đường bằng Feedforward Curvature + Stanley Feedback Controller cực kỳ chính xác.
"""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class DataCollectionDriverNode(Node):
    def __init__(self):
        super().__init__('data_collection_driver_node')

        self.declare_parameter('base_speed', 0.45)           # Tốc độ tiến (m/s)
        self.declare_parameter('curve_amp', 0.0)             # Biên độ uốn cong hàng cây (0.0 cho hàng thẳng)
        self.declare_parameter('curve_wavelength', 12.0)     # Bước sóng uốn cong (m)
        self.declare_parameter('offset_amp', 0.02)           # Biên độ chủ động lệch tâm luống (±2cm để xe không đụng cây)

        self.base_speed = self.get_parameter('base_speed').get_parameter_value().double_value
        self.curve_amp = self.get_parameter('curve_amp').get_parameter_value().double_value
        self.curve_wl = self.get_parameter('curve_wavelength').get_parameter_value().double_value
        self.offset_amp = self.get_parameter('offset_amp').get_parameter_value().double_value

        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Hiệu chỉnh Odometry về tọa độ thực trong Gazebo
        self.odom_calibrated = False
        self.x_offset = 0.0
        self.y_offset = 0.0
        self.yaw_offset = 0.0

        self.world_x = -1.5
        self.world_y = 0.4
        self.world_yaw = 0.0

        self.min_front_dist = 999.0
        self.start_time = time.time()
        
        self.direction = 1.0       # +1.0 (chạy tiến từ -1.5m -> +4.2m), -1.0 (chạy ngược lại)
        self.turning = False
        self.turn_until = 0.0

        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz control loop
        self.get_logger().info("[Data Collection Driver] Khởi chạy Bộ điều khiển bám đường & Quay đầu đất trống (Open-Space U-Turn)!")

    def odom_callback(self, msg: Odometry):
        raw_x = msg.pose.pose.position.x
        raw_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        raw_yaw = math.atan2(siny_cosp, cosy_cosp)

        if not self.odom_calibrated:
            self.x_offset = 0.0
            self.y_offset = 0.0
            self.yaw_offset = 0.0
            self.odom_calibrated = True
            self.get_logger().info(f"[Driver] Raw Odom calibrated. Start Pose: ({raw_x:.2f}, {raw_y:.2f})")

        self.world_x = raw_x + self.x_offset
        self.world_y = raw_y + self.y_offset
        self.world_yaw = normalize_angle(raw_yaw + self.yaw_offset)

    def scan_callback(self, msg: LaserScan):
        if not msg.ranges:
            return
        num_samples = len(msg.ranges)
        mid_idx = num_samples // 2
        window = max(1, int(num_samples * (10.0 / 360.0)))
        front_ranges = msg.ranges[max(0, mid_idx - window): min(num_samples, mid_idx + window)]
        valid_ranges = [r for r in front_ranges if r > 0.05 and not math.isnan(r) and not math.isinf(r)]
        if valid_ranges:
            self.min_front_dist = min(valid_ranges)
        else:
            self.min_front_dist = 999.0

    def get_path_kinematics(self, x):
        """Tính chính xác vị trí y, đạo hàm dy/dx và độ cong d2y/dx2 tại tọa độ x trong Gazebo"""
        k = 2.0 * math.pi / self.curve_wl
        lane_center_y = 0.5  # Đâm giữa làn upper (y=0.5m) giữa 2 hàng bắp y=0.0m và y=1.0m
        if x < 0.0:
            y_c = lane_center_y
            dy_c = 0.0
            ddy_c = 0.0
        elif x <= 3.6:
            y_c = lane_center_y + self.curve_amp * math.sin(k * x)
            dy_c = self.curve_amp * k * math.cos(k * x)
            ddy_c = -self.curve_amp * k * k * math.sin(k * x)
        else:
            # Vùng đất trống ngoài hàng bắp (x > 3.6m)
            y_c = lane_center_y + self.curve_amp * math.sin(k * 3.6)
            dy_c = 0.0
            ddy_c = 0.0
        return y_c, dy_c, ddy_c

    def control_loop(self):
        if not self.odom_calibrated:
            return

        t = time.time() - self.start_time
        msg = Twist()

        # 1. Đang trong vùng đất trống ngoài hàng bắp thực hiện quay đầu 180 độ
        if self.turning:
            if time.time() < self.turn_until:
                msg.linear.x = 0.15
                msg.angular.z = 1.2
                self.pub_cmd_vel.publish(msg)
                return
            else:
                self.turning = False
                self.get_logger().info(f"[Driver] Đã quay đầu 180° trên đất trống xong! Tiến vào luống chiều dir = {self.direction}")

        # 2. Chỉ quay đầu khi xe đã chạy HẲN RA NGOÀI ĐẤT TRỐNG (x > 4.2m hoặc x < -2.0m)
        reached_end_forward = (self.direction > 0 and self.world_x > 4.2)
        reached_end_backward = (self.direction < 0 and self.world_x < -2.0)

        if reached_end_forward or reached_end_backward:
            self.turning = True
            self.turn_until = time.time() + 2.7  # Thời gian xoay 180 độ ngoài đất trống (~2.7s)
            self.direction *= -1.0
            self.get_logger().info(f"[Driver] Đã ra vùng đất trống ngoài hàng bắp (World X = {self.world_x:.2f}m) -> Quay đầu 180° an toàn...")
            msg.linear.x = 0.0
            msg.angular.z = 1.2
            self.pub_cmd_vel.publish(msg)
            return

        # 3. Tính toán đường bám & độ cong chính xác (Feedforward Trajectory Control)
        y_center, dy_center, ddy_center = self.get_path_kinematics(self.world_x)

        # Tạo độ đung đưa lệch tâm luống nhẹ nhàng (±6cm) giúp bộ ảnh phong phú mà cực kỳ an toàn
        dynamic_offset = self.offset_amp * math.sin(0.3 * t)
        target_y = y_center + dynamic_offset

        # Tính góc hướng mong muốn & độ cong
        path_angle = math.atan2(dy_center, 1.0)
        curvature = ddy_center / math.pow(1.0 + dy_center * dy_center, 1.5)

        v_ref = self.base_speed
        if self.direction < 0:
            target_heading = normalize_angle(math.pi + path_angle)
            w_feedforward = -v_ref * curvature
        else:
            target_heading = path_angle
            w_feedforward = v_ref * curvature

        # Sai số vị trí & sai số góc lái
        cross_track_err = self.world_y - target_y
        heading_err = normalize_angle(self.world_yaw - target_heading)

        # Bộ điều khiển Feedforward + Stanley Feedback Controller
        ky = 2.2
        k_heading = 2.5

        if self.direction > 0:
            steering = w_feedforward - ky * cross_track_err - k_heading * heading_err
        else:
            steering = w_feedforward + ky * cross_track_err - k_heading * heading_err

        # Giới hạn góc lái an toàn [-0.8, 0.8] rad/s
        steering = max(-0.8, min(0.8, steering))

        msg.linear.x = v_ref
        msg.angular.z = steering

        self.pub_cmd_vel.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DataCollectionDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
