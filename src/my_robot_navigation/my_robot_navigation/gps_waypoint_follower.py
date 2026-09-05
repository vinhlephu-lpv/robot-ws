#!/usr/bin/env python3
"""
==============================================================================
  ROS 2 Node: gps_waypoint_follower (Package: my_robot_navigation)
  
  Tính năng:
  - Tự động dẫn đường xe tự hành theo tọa độ GPS ngoài trời kết hợp Nav2.
  - Chuyển đổi Kinh độ/Vĩ độ (WGS84) sang Tọa độ Bản đồ (map frame)
    thông qua Service /fromLL của navsat_transform_node.
  - Gửi mục tiêu NavigateToPose Action tới Nav2 Controller.
  - Hiển thị cọc tiêu Marker trực quan trên RViz (/gps_waypoints_markers).
  - Hỗ trợ 2 chế độ:
      1. Nhận điểm GPS thời gian thực qua topic /goal_gps (sensor_msgs/NavSatFix)
      2. Nạp danh sách Waypoint từ file YAML (tọa độ các luống cây ngoài đồng)
==============================================================================
"""

import math
import os
import sys
import time
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import NavSatFix
from visualization_msgs.msg import Marker, MarkerArray
from geographic_msgs.msg import GeoPoint
from robot_localization.srv import FromLL
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


class GpsWaypointFollower(Node):
    def __init__(self):
        super().__init__('gps_waypoint_follower')

        # ── Tham số cấu hình ──────────────────────────────────────────────
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('default_yaw', 0.0)
        self.declare_parameter('goal_tolerance_m', 0.8)
        self.declare_parameter('map_frame', 'map')

        self.waypoints_file = self.get_parameter('waypoints_file').value
        self.default_yaw = float(self.get_parameter('default_yaw').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance_m').value)
        self.map_frame = self.get_parameter('map_frame').value

        # ── Service Client /fromLL (navsat_transform_node) ────────────────
        self.from_ll_client = self.create_client(FromLL, '/fromLL')

        # ── Action Client NavigateToPose (Nav2) ───────────────────────────
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ── Publishers & Subscribers ──────────────────────────────────────
        self.marker_pub = self.create_publisher(MarkerArray, '/gps_waypoints_markers', 10)
        self.goal_sub = self.create_subscription(NavSatFix, '/goal_gps', self.goal_gps_callback, 10)

        # ── Trạng thái hàng đợi Waypoints ─────────────────────────────────
        self.waypoint_queue = []
        self.current_waypoint_idx = 0
        self.is_navigating = False
        self.current_goal_handle = None

        self.get_logger().info('🚀 [gps_waypoint_follower] Khởi động thành công!')
        self.get_logger().info('   -> Sẵn sàng nhận điểm GPS qua topic /goal_gps hoặc file YAML.')

        # Nạp file nếu được chỉ định
        if self.waypoints_file and os.path.isfile(self.waypoints_file):
            self.load_waypoints_from_file(self.waypoints_file)

        # Timer chu kỳ kiểm tra và dẫn đường (1 Hz)
        self.timer = self.create_timer(1.0, self.process_queue)

    def load_waypoints_from_file(self, file_path):
        """Đọc danh sách tọa độ GPS từ file YAML."""
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
            pts = data.get('waypoints', [])
            self.waypoint_queue = []
            for p in pts:
                lat = float(p.get('latitude', p.get('lat', 0.0)))
                lon = float(p.get('longitude', p.get('lon', 0.0)))
                yaw = float(p.get('yaw', self.default_yaw))
                if lat != 0.0 and lon != 0.0:
                    self.waypoint_queue.append({'lat': lat, 'lon': lon, 'yaw': yaw})
            self.get_logger().info(f'📍 Đã nạp thành công {len(self.waypoint_queue)} điểm GPS từ: {file_path}')
            self.publish_markers()
        except Exception as e:
            self.get_logger().error(f'Lỗi đọc file waypoints: {e}')

    def goal_gps_callback(self, msg: NavSatFix):
        """Nhận điểm GPS thời gian thực từ người dùng hoặc bản đồ vệ tinh."""
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            self.get_logger().warn('Tọa độ GPS nhận được không hợp lệ (NaN)!')
            return
        self.get_logger().info(f'🎯 Nhận điểm GPS mới: Lat={msg.latitude:.7f}, Lon={msg.longitude:.7f}')
        self.waypoint_queue.append({
            'lat': msg.latitude,
            'lon': msg.longitude,
            'yaw': self.default_yaw
        })
        self.publish_markers()

    def process_queue(self):
        """Xử lý hàng đợi các điểm Waypoint."""
        if self.is_navigating or not self.waypoint_queue:
            return

        if self.current_waypoint_idx >= len(self.waypoint_queue):
            return

        # Lấy waypoint tiếp theo
        wp = self.waypoint_queue[self.current_waypoint_idx]
        self.get_logger().info(
            f'🚜 Bắt đầu dẫn đường tới Waypoint [{self.current_waypoint_idx + 1}/{len(self.waypoint_queue)}]: '
            f'Lat={wp["lat"]:.7f}, Lon={wp["lon"]:.7f}'
        )
        self.navigate_to_gps(wp['lat'], wp['lon'], wp['yaw'])

    def navigate_to_gps(self, lat, lon, yaw_deg):
        """Chuyển đổi GPS sang Map qua /fromLL và gửi lệnh Nav2."""
        if not self.from_ll_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('⏳ Đang chờ Service /fromLL của navsat_transform_node...')
            return

        req = FromLL.Request()
        req.ll_point = GeoPoint(latitude=lat, longitude=lon, altitude=0.0)
        future = self.from_ll_client.call_async(req)
        future.add_done_callback(lambda f: self.on_from_ll_done(f, yaw_deg))
        self.is_navigating = True

    def on_from_ll_done(self, future, yaw_deg):
        """Nhận tọa độ Map X, Y từ Service /fromLL."""
        try:
            resp = future.result()
            x = resp.map_point.x
            y = resp.map_point.y
            self.get_logger().info(f'🗺️ Chuyển đổi thành công: Map X={x:+.2f}m, Y={y:+.2f}m')
            self.send_nav2_goal(x, y, yaw_deg)
        except Exception as e:
            self.get_logger().error(f'Gọi Service /fromLL thất bại: {e}')
            self.is_navigating = False

    def send_nav2_goal(self, x, y, yaw_deg):
        """Gửi Action Goal tới Nav2 navigate_to_pose."""
        if not self.nav_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn('⏳ Đang chờ Nav2 Action Server (navigate_to_pose)...')
            self.is_navigating = False
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = self.map_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0

        # Chuyển góc yaw sang Quaternion
        yaw_rad = math.radians(yaw_deg)
        goal_msg.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)

        send_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_cb
        )
        send_future.add_done_callback(self.nav_goal_response_cb)

    def nav_goal_response_cb(self, future):
        """Phản hồi khi Nav2 chấp nhận hoặc từ chối mục tiêu."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('❌ Nav2 từ chối nhận mục tiêu!')
            self.is_navigating = False
            return

        self.current_goal_handle = goal_handle
        self.get_logger().info('✅ Nav2 đã chấp nhận mục tiêu. Đang di chuyển...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_cb)

    def nav_feedback_cb(self, feedback_msg):
        """Theo dõi khoảng cách còn lại tới mục tiêu."""
        fb = feedback_msg.feedback
        dist = getattr(fb, 'distance_remaining', None)
        if dist is not None:
            self.get_logger().info(f'🚗 Khoảng cách tới điểm GPS: {dist:4.2f}m', throttle_duration_sec=2.0)

    def nav_result_cb(self, future):
        """Kết quả sau khi Nav2 hoàn thành mục tiêu."""
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'🎉 ĐÃ ĐẾN NƠI Waypoint [{self.current_waypoint_idx + 1}] thành công!')
            self.current_waypoint_idx += 1
            if self.current_waypoint_idx >= len(self.waypoint_queue):
                self.get_logger().info('🏁 ĐÃ HOÀN THÀNH TOÀN BỘ DANH SÁCH ĐIỂM GPS!')
        else:
            self.get_logger().warn(f'⚠️ Không thể hoàn thành waypoint (Status: {status}). Chuyển sang điểm kế tiếp.')
            self.current_waypoint_idx += 1

        self.is_navigating = False
        self.publish_markers()

    def publish_markers(self):
        """Hiển thị các cọc cờ Waypoint GPS trên RViz."""
        marker_arr = MarkerArray()
        for idx, wp in enumerate(self.waypoint_queue):
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'gps_waypoints'
            m.id = idx
            m.type = Marker.CYLINDER
            m.action = Marker.ADD

            # Kích thước
            m.scale.x = 0.4
            m.scale.y = 0.4
            m.scale.z = 0.8

            # Màu sắc: Đã hoàn thành (Xám), Đang chạy (Xanh lá), Chờ (Vàng)
            if idx < self.current_waypoint_idx:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.5, 0.5, 0.5, 0.6
            elif idx == self.current_waypoint_idx:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 0.9
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.8, 0.0, 0.8

            marker_arr.markers.append(m)

        if marker_arr.markers:
            self.marker_pub.publish(marker_arr)


def main(args=None):
    rclpy.init(args=args)
    node = GpsWaypointFollower()
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
