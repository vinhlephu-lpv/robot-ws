#!/usr/bin/env python3
"""
Ultra-Compact Micro-HUD Sensor Visualizer Node.
Displays ultra-minimalist symbols:
- F, B, L, R compass symbols (Front, Back, Left, Right)
- Micro distance labels (1m, 2m, 3m, 5m)
- Dynamic micro-distance tag at nearest obstacle (e.g. '0.85m')
- Subtle safety circle (0.5m)
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class SensorVisualizer(Node):
    def __init__(self):
        super().__init__('sensor_visualizer')

        self.marker_pub = self.create_publisher(
            MarkerArray, '/sensor_test/markers', 10)

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)

        self.latest_scan = None
        self.timer = self.create_timer(0.1, self.publish_markers)

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def publish_markers(self):
        markers = MarkerArray()
        now = self.get_clock().now().to_msg()

        # -------------------------------------------------------------
        # 1. Thin Concentric Rings (1m, 2m, 3m, 5m) + Micro labels
        # -------------------------------------------------------------
        radii = [1.0, 2.0, 3.0, 5.0]
        for idx, r in enumerate(radii):
            ring = Marker()
            ring.header.frame_id = 'base_footprint'
            ring.header.stamp = now
            ring.ns = 'rings'
            ring.id = idx
            ring.type = Marker.LINE_STRIP
            ring.action = Marker.ADD
            ring.scale.x = 0.008  # Ultra-thin line

            ring.color.r = 0.0
            ring.color.g = 0.75
            ring.color.b = 0.90
            ring.color.a = 0.25

            for i in range(49):
                theta = i * 2 * math.pi / 48
                ring.points.append(Point(x=r * math.cos(theta), y=r * math.sin(theta), z=0.005))

            markers.markers.append(ring)

            # Micro Distance Tag (1m, 2m, 3m, 5m)
            label = Marker()
            label.header.frame_id = 'base_footprint'
            label.header.stamp = now
            label.ns = 'labels'
            label.id = idx
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = r * 0.707
            label.pose.position.y = -r * 0.707
            label.pose.position.z = 0.02
            label.scale.z = 0.11  # Micro font
            label.color.r = 0.0
            label.color.g = 0.85
            label.color.b = 1.0
            label.color.a = 0.60
            label.text = f'{int(r)}m'
            markers.markers.append(label)

        # -------------------------------------------------------------
        # 2. Micro Compass Symbols (F, B, L, R)
        # -------------------------------------------------------------
        compass = [
            (1.5, 0.0, 'F', 0.2, 1.0, 0.4),   # Front (Green)
            (-1.5, 0.0, 'B', 1.0, 0.4, 0.2),  # Back (Orange)
            (0.0, 1.5, 'L', 0.2, 0.8, 1.0),   # Left (Cyan)
            (0.0, -1.5, 'R', 0.2, 0.8, 1.0),  # Right (Cyan)
        ]
        for idx, (x, y, symbol, r, g, b) in enumerate(compass):
            m = Marker()
            m.header.frame_id = 'base_footprint'
            m.header.stamp = now
            m.ns = 'compass'
            m.id = idx
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.03
            m.scale.z = 0.14  # Micro clean symbol
            m.color.r = r
            m.color.g = g
            m.color.b = b
            m.color.a = 0.75
            m.text = symbol
            markers.markers.append(m)

        # -------------------------------------------------------------
        # 3. Micro Distance Tag at Nearest Obstacle (e.g. '0.85m')
        # -------------------------------------------------------------
        min_dist = 999.0
        min_x = 0.0
        min_y = 0.0

        if self.latest_scan:
            angle = self.latest_scan.angle_min
            for dist in self.latest_scan.ranges:
                if self.latest_scan.range_min < dist < self.latest_scan.range_max:
                    if dist < min_dist:
                        min_dist = dist
                        min_x = -dist * math.cos(angle)
                        min_y = -dist * math.sin(angle)
                angle += self.latest_scan.angle_increment

        # Safety Zone (0.5m)
        safety = Marker()
        safety.header.frame_id = 'base_footprint'
        safety.header.stamp = now
        safety.ns = 'safety'
        safety.id = 0
        safety.type = Marker.CYLINDER
        safety.action = Marker.ADD
        safety.pose.position.z = 0.005
        safety.scale.x = 1.0
        safety.scale.y = 1.0
        safety.scale.z = 0.002

        if min_dist < 0.5:
            safety.color.r = 1.0
            safety.color.g = 0.1
            safety.color.b = 0.1
            safety.color.a = 0.25
        else:
            safety.color.r = 0.1
            safety.color.g = 0.8
            safety.color.b = 0.2
            safety.color.a = 0.08

        markers.markers.append(safety)

        # Micro distance floating tag on closest point
        if min_dist < 12.0:
            tag = Marker()
            tag.header.frame_id = 'base_footprint'
            tag.header.stamp = now
            tag.ns = 'closest_tag'
            tag.id = 0
            tag.type = Marker.TEXT_VIEW_FACING
            tag.action = Marker.ADD
            tag.pose.position.x = min_x
            tag.pose.position.y = min_y
            tag.pose.position.z = 0.12
            tag.scale.z = 0.12  # Micro font
            tag.color.r = 1.0
            tag.color.g = 0.9
            tag.color.b = 0.1
            tag.color.a = 0.85
            tag.text = f'{min_dist:.2f}m'
            markers.markers.append(tag)

        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = SensorVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
