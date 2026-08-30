#!/usr/bin/env python3
"""
ESP32 Hardware Bridge Node for ROS 2.
Connects ROS 2 high-level navigation/AI to ESP32 motor PID & Encoder controller.

Roles:
1. Downlink: Converts /cmd_vel (v, w) -> wheel Target RPMs -> Sends to ESP32 via Serial/WiFi.
2. Uplink: Receives wheel encoder speeds/distances from ESP32 -> Computes Odometry (x, y, yaw) -> Publishes /odom & TF.
3. Fallback: If ESP32 is not connected, operates in graceful Mock/Open-Loop mode.
"""

import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


class ESP32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('connection_mode', 'serial')  # 'serial', 'wifi', 'mock'
        self.declare_parameter('serial_port', '/dev/ttyUSB1')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('esp32_ip', '192.168.1.100')
        self.declare_parameter('wheel_diameter', 0.20)  # meters (from codepid2608.ino)
        self.declare_parameter('wheel_base', 0.58)      # meters (distance between left and right wheels)
        self.declare_parameter('odom_topic', '/odom/raw')
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        self.mode = self.get_parameter('connection_mode').value
        self.port = self.get_parameter('serial_port').value
        self.baud = self.get_parameter('baudrate').value
        self.wheel_d = float(self.get_parameter('wheel_diameter').value)
        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.odom_topic = self.get_parameter('odom_topic').value
        
        raw_pub_tf = self.get_parameter('publish_tf').value
        if isinstance(raw_pub_tf, str):
            self.publish_tf = raw_pub_tf.lower() in ('true', '1', 'yes')
        else:
            self.publish_tf = bool(raw_pub_tf)

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.wheel_circ = math.pi * self.wheel_d

        # ── Robot Odometry State (Dead Reckoning) ─────────────────────
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vth = 0.0
        self.last_time = self.get_clock().now()

        # ── Publishers & Subscribers ──────────────────────────────────
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(
            Odometry, self.odom_topic, 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ── Serial Connection ─────────────────────────────────────────
        self.ser = None
        if self.mode == 'serial':
            self.init_serial()

        # ── Loop Timer (20 Hz for Odom processing) ────────────────────
        self.timer = self.create_timer(0.05, self.update_loop)

        self.get_logger().info(
            f'ESP32 Bridge started [Mode: {self.mode}] [Wheel D: {self.wheel_d}m, Base: {self.wheel_base}m]')

    def init_serial(self):
        if not SERIAL_AVAILABLE:
            self.get_logger().warn('pyserial not found! Falling back to MOCK mode.')
            self.mode = 'mock'
            return

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
            self.get_logger().info(f'Connected to ESP32 on {self.port} at {self.baud} baud')
        except Exception as e:
            self.get_logger().warn(f'Could not open serial port {self.port}: {e}. Running in MOCK mode.')
            self.mode = 'mock'

    def cmd_vel_callback(self, msg: Twist):
        """
        Receives ROS 2 /cmd_vel (linear.x [m/s], angular.z [rad/s]).
        Calculates Left & Right wheel target RPMs and sends to ESP32.
        """
        v = msg.linear.x
        w = msg.angular.z

        # Differential drive inverse kinematics
        v_left = v - (w * self.wheel_base / 2.0)
        v_right = v + (w * self.wheel_base / 2.0)

        # Convert m/s -> RPM: RPM = (v * 60) / (pi * D)
        rpm_left = (v_left * 60.0) / self.wheel_circ
        rpm_right = (v_right * 60.0) / self.wheel_circ

        # Store for mock odometry
        self.vx = v
        self.vth = w

        # Send to ESP32
        if self.mode == 'serial' and self.ser and self.ser.is_open:
            cmd_str = f'V {rpm_left:.1f} {rpm_right:.1f}\n'
            try:
                self.ser.write(cmd_str.encode('utf-8'))
            except Exception as e:
                self.get_logger().warn(f'Serial write error: {e}')

    def update_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if dt <= 0:
            return

        v_l = 0.0
        v_r = 0.0

        # Read serial feedback if available
        if self.mode == 'serial' and self.ser and self.ser.is_open:
            try:
                while self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    # 1. Giao thức ODOM: "ODOM <v_left> <v_right>" (m/s)
                    if line.startswith('ODOM') or line.startswith('O '):
                        parts = line.split()
                        if len(parts) >= 3:
                            v_l = float(parts[1])
                            v_r = float(parts[2])
                            self.vx = (v_r + v_l) / 2.0
                            self.vth = (v_r - v_l) / self.wheel_base
                    # 2. Giao thức ENC: "ENC <tick_FL> <tick_RL> <tick_FR> <tick_RR> <dt_ms>" hoặc "ENC <tick_L> <tick_R> <dt_ms>"
                    elif line.startswith('ENC'):
                        parts = line.split()
                        if len(parts) >= 6:
                            # Chuẩn 4 bánh độc lập từ code3008.ino
                            tick_l = (float(parts[1]) + float(parts[2])) / 2.0
                            tick_r = (float(parts[3]) + float(parts[4])) / 2.0
                            dt_ms = float(parts[5])
                        elif len(parts) >= 4:
                            # Chuẩn 2 cụm bánh
                            tick_l = float(parts[1])
                            tick_r = float(parts[2])
                            dt_ms = float(parts[3])
                        else:
                            dt_ms = 0.0

                        if dt_ms > 0:
                            if hasattr(self, '_last_tick_l') and hasattr(self, '_last_tick_r'):
                                dt_s = dt_ms / 1000.0
                                d_l = tick_l - self._last_tick_l
                                d_r = tick_r - self._last_tick_r
                                v_l = (d_l * self.wheel_circ / 200.0) / dt_s
                                v_r = (d_r * self.wheel_circ / 200.0) / dt_s
                                self.vx = (v_r + v_l) / 2.0
                                self.vth = (v_r - v_l) / self.wheel_base
                            self._last_tick_l = tick_l
                            self._last_tick_r = tick_r
            except Exception:
                pass

        # Dead reckoning integration
        delta_x = (self.vx * math.cos(self.yaw)) * dt
        delta_y = (self.vx * math.sin(self.yaw)) * dt
        delta_th = self.vth * dt

        self.x += delta_x
        self.y += delta_y
        self.yaw += delta_th

        # Normalize yaw [-pi, pi]
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        # Publish Odometry & TF
        self.publish_odometry(now)

    def publish_odometry(self, now):
        # Quaternion from yaw
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)

        # 1. Publish Odometry message kèm Covariance cho EKF
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.pose.covariance = [
            0.01, 0.0,  0.0, 0.0, 0.0, 0.0,
            0.0,  0.01, 0.0, 0.0, 0.0, 0.0,
            0.0,  0.0,  1e6, 0.0, 0.0, 0.0,
            0.0,  0.0,  0.0, 1e6, 0.0, 0.0,
            0.0,  0.0,  0.0, 0.0, 1e6, 0.0,
            0.0,  0.0,  0.0, 0.0, 0.0, 0.05
        ]

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.vth

        odom.twist.covariance = [
            0.02, 0.0,  0.0, 0.0, 0.0, 0.0,
            0.0,  1e6,  0.0, 0.0, 0.0, 0.0,
            0.0,  0.0,  1e6, 0.0, 0.0, 0.0,
            0.0,  0.0,  0.0, 1e6, 0.0, 0.0,
            0.0,  0.0,  0.0, 0.0, 1e6, 0.0,
            0.0,  0.0,  0.0, 0.0, 0.0, 0.05
        ]

        self.odom_pub.publish(odom)

        # 2. Broadcast TF (odom -> base_footprint)
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = ESP32Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
