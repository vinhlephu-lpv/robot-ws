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

        self.target_rpm_left = 0.0
        self.target_rpm_right = 0.0
        self._stop_sent_count = 0
        self._serial_rx_buffer = ''

        # ── Loop Timer (20 Hz for Odom processing & continuous ESP32 streaming) ──
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
        Calculates Left & Right wheel target RPMs and updates target.
        """
        v = msg.linear.x
        w = msg.angular.z

        # Differential drive inverse kinematics
        v_left = v - (w * self.wheel_base / 2.0)
        v_right = v + (w * self.wheel_base / 2.0)

        # Convert m/s -> RPM: RPM = (v * 60) / (pi * D)
        self.target_rpm_left = (v_left * 60.0) / self.wheel_circ
        self.target_rpm_right = (v_right * 60.0) / self.wheel_circ

        # Store for mock odometry only when NOT in real hardware serial mode
        if self.mode != 'serial':
            self.vx = v
            self.vth = w

        # Direct write to ESP32 immediately on cmd_vel arrival
        if self.mode == 'serial' and self.ser and self.ser.is_open:
            is_zero = (abs(self.target_rpm_left) < 0.1 and abs(self.target_rpm_right) < 0.1)
            if not is_zero:
                self._stop_sent_count = 0
                cmd_str = f'V {self.target_rpm_left:.1f} {self.target_rpm_right:.1f}\n'
                try:
                    self.ser.write(cmd_str.encode('utf-8'))
                except Exception as e:
                    self.get_logger().warn(f'Serial write error: {e}')
            elif self._stop_sent_count < 3:
                self._stop_sent_count += 1
                try:
                    self.ser.write(b'V 0.0 0.0\n')
                except Exception:
                    pass

    def update_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if dt <= 0:
            return

        # Phát nuôi Watchdog ESP32 chỉ khi xe đang di chuyển hoặc chuyển trạng thái dừng
        if self.mode == 'serial' and self.ser and self.ser.is_open:
            is_zero = (abs(self.target_rpm_left) < 0.1 and abs(self.target_rpm_right) < 0.1)
            if not is_zero:
                self._stop_sent_count = 0
                cmd_str = f'V {self.target_rpm_left:.1f} {self.target_rpm_right:.1f}\n'
                try:
                    self.ser.write(cmd_str.encode('utf-8'))
                except Exception:
                    pass
            elif self._stop_sent_count < 3:
                # Chỉ gửi lệnh dừng 3 lần rồi im lặng, không spam liên tục 20Hz khi đứng yên
                self._stop_sent_count += 1
                try:
                    self.ser.write(b'V 0.0 0.0\n')
                except Exception:
                    pass

        v_l = 0.0
        v_r = 0.0

        # Read serial feedback non-blocking
        if self.mode == 'serial' and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    raw_data = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                    self._serial_rx_buffer += raw_data
                    odom_received = False

                    while '\n' in self._serial_rx_buffer:
                        line, self._serial_rx_buffer = self._serial_rx_buffer.split('\n', 1)
                        line = line.strip()
                        if not line:
                            continue

                        # 1. Giao thức ODOM chuẩn Kalman từ ESP32: "ODOM <v_left> <v_right>" (m/s)
                        if line.startswith('ODOM') or line.startswith('O '):
                            parts = line.split()
                            if len(parts) >= 3:
                                try:
                                    v_l = float(parts[1])
                                    v_r = float(parts[2])
                                    self.vx = (v_r + v_l) / 2.0
                                    self.vth = (v_r - v_l) / self.wheel_base
                                    odom_received = True
                                except ValueError:
                                    pass

                        # 2. Giao thức ENC: Chỉ dùng dự phòng nếu CHƯA nhận được bản tin ODOM
                        elif line.startswith('ENC') and not odom_received:
                            parts = line.split()
                            try:
                                if len(parts) >= 6:
                                    tick_l = (float(parts[1]) + float(parts[2])) / 2.0
                                    tick_r = (float(parts[3]) + float(parts[4])) / 2.0
                                    dt_ms = float(parts[5])
                                elif len(parts) >= 4:
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
                            except ValueError:
                                pass
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

    def destroy_node(self):
        """Clean shutdown: brake motors to 0 and safely close serial port on Ctrl+C."""
        try:
            if self.mode == 'serial' and self.ser and self.ser.is_open:
                for _ in range(2):
                    self.ser.write(b'V 0.0 0.0\n')
                    time.sleep(0.01)
                self.ser.write(b'STOP\n')
                self.ser.flush()
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


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
