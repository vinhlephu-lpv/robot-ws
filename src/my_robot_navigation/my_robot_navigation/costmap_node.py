#!/usr/bin/env python3
"""
Costmap Inflation Node for Agricultural Robot
Generates clean 2D Layered Costmap with Noise Filtering and Inflation Gradients
from SLAM Occupancy Grid (/map).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2


class CostmapNode(Node):
    def __init__(self):
        super().__init__('costmap_node')

        # Declare parameters
        self.declare_parameter('inscribed_radius', 0.35)      # m - robot physical radius (35cm)
        self.declare_parameter('inflation_radius', 0.50)      # m - robot (0.35m) + safety buffer (0.15m) = 0.50m
        self.declare_parameter('cost_scaling_factor', 4.0)    # exponential decay steepness
        self.declare_parameter('obstacle_threshold', 60)      # minimum SLAM confidence (0-100) to treat as obstacle
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)

        self.inscribed_radius = self.get_parameter('inscribed_radius').value
        self.inflation_radius = self.get_parameter('inflation_radius').value
        self.cost_scaling_factor = self.get_parameter('cost_scaling_factor').value
        self.obs_thresh = self.get_parameter('obstacle_threshold').value

        # QoS Profiles
        map_sub_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        costmap_pub_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers & Publishers
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_sub_qos
        )

        self.costmap_pub = self.create_publisher(
            OccupancyGrid,
            '/costmap',
            costmap_pub_qos
        )

        self.get_logger().info(
            f"Costmap Node Initialized: inscribed={self.inscribed_radius}m, "
            f"inflation={self.inflation_radius}m, threshold={self.obs_thresh}%"
        )

    def map_callback(self, msg: OccupancyGrid):
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution

        if width == 0 or height == 0 or resolution <= 0.0:
            return

        # Convert OccupancyGrid 1D data to 2D numpy array
        raw_data = np.array(msg.data, dtype=np.int8).reshape((height, width))

        # ── 1. Lọc nhiễu (Denoising) ──────────────────────────────────
        # Chỉ lấy các điểm có độ tin cậy cao từ SLAM (>= 60%)
        obs_raw = (raw_data >= self.obs_thresh).astype(np.uint8)

        # Khử các chấm nhiễu đơn lẻ (speckle noise) bằng phép mở hình thái học (Morphological Open)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        obs_clean = cv2.morphologyEx(obs_raw, cv2.MORPH_OPEN, kernel)

        # ── 2. Tính toán khoảng cách & Inflation Gradient ─────────────
        costmap = np.full((height, width), -1, dtype=np.int8)

        # Vùng không gian đã biết (Known Free Space: 0 <= raw < obs_thresh)
        known_mask = (raw_data >= 0)
        costmap[known_mask] = 0

        if np.any(obs_clean):
            # Tính khoảng cách Euclidean từ mỗi ô đến chướng ngại vật gần nhất
            inv_obs = 1 - obs_clean
            dist_cells = cv2.distanceTransform(inv_obs, cv2.DIST_L2, 5)
            dist_m = dist_cells * resolution

            # 1. Chướng ngại vật thực tế (Tường, cột: cost = 100)
            costmap[obs_clean == 1] = 100

            # 2. Vùng nguy hiểm va chạm robot (Inscribed radius: cost = 99)
            inscribed_mask = (obs_clean == 0) & (dist_m <= self.inscribed_radius) & known_mask
            costmap[inscribed_mask] = 99

            # 3. Vùng đệm an toàn giảm dần (Inflation gradient: 98 -> 1)
            decay_mask = (dist_m > self.inscribed_radius) & (dist_m <= self.inflation_radius) & known_mask
            decay_d = dist_m[decay_mask] - self.inscribed_radius
            decay_costs = 98.0 * np.exp(-self.cost_scaling_factor * decay_d)
            costmap[decay_mask] = np.clip(np.round(decay_costs).astype(np.int8), 1, 98)

        # ── 3. Xuất bản Costmap OccupancyGrid ──────────────────────────
        costmap_msg = OccupancyGrid()
        costmap_msg.header = msg.header
        costmap_msg.info = msg.info
        costmap_msg.data = costmap.flatten().tolist()

        self.costmap_pub.publish(costmap_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CostmapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
