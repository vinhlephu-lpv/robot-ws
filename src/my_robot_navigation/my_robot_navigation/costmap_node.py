#!/usr/bin/env python3
"""
Costmap Inflation Node for Agricultural Robot
Generates academic-standard 2D Layered Costmap with Inflation Gradients
from SLAM Occupancy Grid (/map) and LiDAR Scan (/scan).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import numpy as np
import cv2


class CostmapNode(Node):
    def __init__(self):
        super().__init__('costmap_node')

        # Declare parameters
        self.declare_parameter('inscribed_radius', 0.18)      # m - robot inner collision boundary
        self.declare_parameter('inflation_radius', 0.50)      # m - outer safety buffer halo (covers row midpoint)
        self.declare_parameter('cost_scaling_factor', 3.5)    # exponential decay steepness
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        self.inscribed_radius = self.get_parameter('inscribed_radius').value
        self.inflation_radius = self.get_parameter('inflation_radius').value
        self.cost_scaling_factor = self.get_parameter('cost_scaling_factor').value

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

        self.last_map_msg = None
        self.get_logger().info(
            f"Costmap Inflation Node initialized (inscribed={self.inscribed_radius}m, "
            f"inflation={self.inflation_radius}m, factor={self.cost_scaling_factor})"
        )

    def map_callback(self, msg: OccupancyGrid):
        self.last_map_msg = msg
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution

        if width == 0 or height == 0 or resolution <= 0.0:
            return

        # Convert OccupancyGrid 1D data to 2D numpy array
        raw_data = np.array(msg.data, dtype=np.int8).reshape((height, width))

        # Identify obstacles and free space
        obs_mask = (raw_data >= 50).astype(np.uint8)

        # Check if there are obstacles
        if np.any(obs_mask):
            # Euclidean distance transform (computes distance to nearest 0)
            inv_obs = 1 - obs_mask
            dist_cells = cv2.distanceTransform(inv_obs, cv2.DIST_L2, 5)
            dist_m = dist_cells * resolution

            costmap = np.full((height, width), -1, dtype=np.int8)

            # Known free space
            known_mask = (raw_data >= 0)
            costmap[known_mask] = 0

            # Lethal obstacles (corn stalk centers: cost = 100)
            costmap[obs_mask == 1] = 100

            # Inscribed radius (cost = 99)
            inscribed_mask = (obs_mask == 0) & (dist_m <= self.inscribed_radius) & known_mask
            costmap[inscribed_mask] = 99

            # Inflation gradient (cost = 98 -> 1)
            decay_mask = (dist_m > self.inscribed_radius) & (dist_m <= self.inflation_radius) & known_mask
            decay_d = dist_m[decay_mask] - self.inscribed_radius
            decay_costs = 98.0 * np.exp(-self.cost_scaling_factor * decay_d)
            costmap[decay_mask] = np.clip(np.round(decay_costs).astype(np.int8), 1, 98)
        else:
            costmap = raw_data.copy()

        # Build output OccupancyGrid
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
