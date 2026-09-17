#!/usr/bin/env python3
"""
Costmap Inflation Node for Agricultural Robot
Generates clean 2D Layered Costmap with Noise Filtering and Inflation Gradients.
Supports Dual Operation Modes:
1. SLAM Mode: Generates costmap from SLAM Occupancy Grid (/map).
2. Real-Time Scan Mapping Mode (AI CNN / Row Following):
   Builds and paints real-time costmap directly from RPLIDAR C1 (/scan) + EKF Odometry (odom frame)
   as the robot navigates down the crop rows, strictly for RViz visualization.
"""

import math
import time
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy, qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
import tf2_ros


class CostmapNode(Node):
    def __init__(self):
        super().__init__('costmap_node')

        # ── Declare Parameters ─────────────────────────────────────────
        self.declare_parameter('inscribed_radius', 0.35)       # m - robot physical boundary (radius 35cm)
        self.declare_parameter('inflation_radius', 0.45)       # m - safety buffer (45cm total)
        self.declare_parameter('cost_scaling_factor', 10.0)    # smooth exponential decay across buffer
        self.declare_parameter('obstacle_threshold', 50)       # hit threshold (0-100) to treat as obstacle
        self.declare_parameter('resolution', 0.05)             # m/cell (5cm resolution)
        self.declare_parameter('map_length_m', 60.0)           # m along row (X axis in odom)
        self.declare_parameter('map_width_m', 20.0)            # m lateral width (Y axis in odom)
        self.declare_parameter('publish_rate', 4.0)            # Hz - publishing frequency
        self.declare_parameter('global_frame', 'odom')         # Fixed frame for mapping
        self.declare_parameter('rolling_window', False)        # True: rolling window, False: cumulative map
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)

        self.inscribed_radius = float(self.get_parameter('inscribed_radius').value)
        self.inflation_radius = float(self.get_parameter('inflation_radius').value)
        self.cost_scaling_factor = float(self.get_parameter('cost_scaling_factor').value)
        self.obs_thresh = int(self.get_parameter('obstacle_threshold').value)
        self.resolution = float(self.get_parameter('resolution').value)
        self.map_length_m = float(self.get_parameter('map_length_m').value)
        self.map_width_m = float(self.get_parameter('map_width_m').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.global_frame = str(self.get_parameter('global_frame').value)
        self.rolling_window = bool(self.get_parameter('rolling_window').value)

        # ── Coordinate Grid Setup (odom frame) ─────────────────────────
        # In ROS OccupancyGrid:
        # width = cells along X, height = cells along Y
        self.grid_w = max(50, int(self.map_length_m / self.resolution))
        self.grid_h = max(50, int(self.map_width_m / self.resolution))
        self.origin_x = -5.0  # allow 5m behind start position
        self.origin_y = -self.map_width_m / 2.0  # centered laterally

        # Internal grid states (height rows x width cols)
        self.obs_hits = np.zeros((self.grid_h, self.grid_w), dtype=np.int16)
        self.seen_mask = np.zeros((self.grid_h, self.grid_w), dtype=bool)
        self.costmap_data = np.full((self.grid_h, self.grid_w), -1, dtype=np.int8)

        # Robot Pose Tracking
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.last_pose_time = 0.0

        # SLAM Map vs Scan Tracking
        self.last_slam_map = None
        self.last_slam_map_time = 0.0
        self.last_scan_time = 0.0

        # Active modified region bounds (bounding box for fast updates)
        self.min_c = self.grid_w // 2
        self.max_c = self.grid_w // 2
        self.min_r = self.grid_h // 2
        self.max_r = self.grid_h // 2

        # ── TF2 Setup ──────────────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── QoS Profiles ───────────────────────────────────────────────
        latch_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ── Subscribers ────────────────────────────────────────────────
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            latch_qos
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10
        )

        # ── Publishers ─────────────────────────────────────────────────
        self.costmap_pub = self.create_publisher(
            OccupancyGrid,
            '/costmap',
            latch_qos
        )

        self.local_costmap_pub = self.create_publisher(
            OccupancyGrid,
            '/local_costmap/costmap',
            latch_qos
        )

        # ── Timer for Costmap Computation & Publishing ─────────────────
        timer_period = 1.0 / max(0.5, self.publish_rate)
        self.timer = self.create_timer(timer_period, self.timer_publish_callback)

        self.get_logger().info(
            f"✅ Real-Time Costmap Node started! Grid: {self.grid_w}x{self.grid_h} ({self.map_length_m:.1f}mx{self.map_width_m:.1f}m), "
            f"res={self.resolution}m, inscribed={self.inscribed_radius}m, inflation={self.inflation_radius}m"
        )

    # ── Odometry Fallback Callback ─────────────────────────────────────
    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.last_pose_time = time.time()

    # ── SLAM Map Callback (Mode 1: SLAM Map Inflation) ────────────────
    def map_callback(self, msg: OccupancyGrid):
        self.last_slam_map = msg
        self.last_slam_map_time = time.time()

    # ── Real-time Scan Callback (Mode 2: Raycasting & Grid Update) ────
    def scan_callback(self, msg: LaserScan):
        self.last_scan_time = time.time()

        # Determine sensor pose in global_frame
        tx, ty, yaw = self.robot_x, self.robot_y, self.robot_yaw
        try:
            if self.tf_buffer.can_transform(self.global_frame, msg.header.frame_id, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.04)):
                t = self.tf_buffer.lookup_transform(self.global_frame, msg.header.frame_id, rclpy.time.Time())
                tx = t.transform.translation.x
                ty = t.transform.translation.y
                q = t.transform.rotation
                siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                yaw = math.atan2(siny_cosp, cosy_cosp)
                self.robot_x, self.robot_y, self.robot_yaw = tx, ty, yaw
        except Exception:
            pass  # Fallback to EKF odom values

        # Robot cell index
        rc = int(round((tx - self.origin_x) / self.resolution))
        rr = int(round((ty - self.origin_y) / self.resolution))

        # Expand or shift map if robot approaches border
        if not self.rolling_window:
            if rc >= self.grid_w - 40 or rc < 20 or rr >= self.grid_h - 20 or rr < 20:
                self._shift_or_expand_grid(tx, ty)
                rc = int(round((tx - self.origin_x) / self.resolution))
                rr = int(round((ty - self.origin_y) / self.resolution))
        else:
            # Rolling window: keep robot centered
            target_orig_x = tx - self.map_length_m / 2.0
            target_orig_y = ty - self.map_width_m / 2.0
            self.origin_x = target_orig_x
            self.origin_y = target_orig_y
            rc = int(round((tx - self.origin_x) / self.resolution))
            rr = int(round((ty - self.origin_y) / self.resolution))

        if not (0 <= rc < self.grid_w and 0 <= rr < self.grid_h):
            return

        # Vectorized Laser Ray Projection
        ranges = np.array(msg.ranges, dtype=np.float32)
        n_points = len(ranges)
        if n_points == 0:
            return

        angles = msg.angle_min + np.arange(n_points, dtype=np.float32) * msg.angle_increment
        valid_mask = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= min(msg.range_max, 8.0))

        v_ranges = ranges[valid_mask]
        v_angles = angles[valid_mask]

        if len(v_ranges) == 0:
            return

        # Global hit coordinates in odom frame
        g_angles = yaw + v_angles
        ox = tx + v_ranges * np.cos(g_angles)
        oy = ty + v_ranges * np.sin(g_angles)

        ec = np.clip(np.round((ox - self.origin_x) / self.resolution).astype(np.int32), 0, self.grid_w - 1)
        er = np.clip(np.round((oy - self.origin_y) / self.resolution).astype(np.int32), 0, self.grid_h - 1)

        # Free space ray clearing using OpenCV line drawing (C++ optimized, < 1ms)
        free_mask = np.zeros((self.grid_h, self.grid_w), dtype=np.uint8)
        step = max(1, len(ec) // 360)
        for i in range(0, len(ec), step):
            cv2.line(free_mask, (rc, rr), (int(ec[i]), int(er[i])), 1, 1)

        # Do not mark endpoint obstacle cells as free space
        free_mask[er, ec] = 0

        # Clear free cells along rays
        free_idx = (free_mask > 0)
        self.seen_mask[free_idx] = True
        self.obs_hits[free_idx] = np.maximum(self.obs_hits[free_idx] - 2, 0)

        # Accumulate obstacle hits at endpoints (persistent over consecutive scans)
        np.add.at(self.obs_hits, (er, ec), 20)
        self.obs_hits[er, ec] = np.minimum(self.obs_hits[er, ec], 100)
        self.seen_mask[er, ec] = True

        # Update active region bounding box
        c_min_val = min(rc, int(np.min(ec)))
        c_max_val = max(rc, int(np.max(ec)))
        r_min_val = min(rr, int(np.min(er)))
        r_max_val = max(rr, int(np.max(er)))

        pad = int(math.ceil(self.inflation_radius / self.resolution)) + 2
        self.min_c = max(0, min(self.min_c, c_min_val - pad))
        self.max_c = min(self.grid_w, max(self.max_c, c_max_val + pad))
        self.min_r = max(0, min(self.min_r, r_min_val - pad))
        self.max_r = min(self.grid_h, max(self.max_r, r_max_val + pad))

    def _shift_or_expand_grid(self, cur_x, cur_y):
        """Auto-shifts grid origin when robot moves beyond boundary to prevent overflow."""
        shift_x = cur_x - self.origin_x - 10.0
        if abs(shift_x) > 10.0:
            cells_x = int(round(shift_x / self.resolution))
            new_obs = np.zeros_like(self.obs_hits)
            new_seen = np.zeros_like(self.seen_mask)
            if cells_x > 0 and cells_x < self.grid_w:
                new_obs[:, :-cells_x] = self.obs_hits[:, cells_x:]
                new_seen[:, :-cells_x] = self.seen_mask[:, cells_x:]
            self.obs_hits = new_obs
            self.seen_mask = new_seen
            self.origin_x += cells_x * self.resolution
            self.min_c = 0
            self.max_c = self.grid_w

    # ── Periodic Costmap Inflation & Publication ──────────────────────
    def timer_publish_callback(self):
        now_sec = time.time()

        # ── Mode 1: SLAM Map is actively publishing ────────────────────
        if self.last_slam_map is not None and (now_sec - self.last_slam_map_time) < 2.5:
            self._process_and_pub_slam_costmap(self.last_slam_map)
            return

        # ── Mode 2: Real-time Scan Mapping ─────────────────────────────
        if (now_sec - self.last_scan_time) > 3.0:
            return  # No recent lidar data

        if self.max_c <= self.min_c or self.max_r <= self.min_r:
            return

        # Extract active ROI for fast distance transform (< 1ms)
        min_r, max_r = self.min_r, self.max_r
        min_c, max_c = self.min_c, self.max_c

        roi_obs = self.obs_hits[min_r:max_r, min_c:max_c]
        roi_seen = self.seen_mask[min_r:max_r, min_c:max_c]

        # Temporal log-odds thresholding (filters out single-scan dust/speckles naturally)
        clean_obs = (roi_obs >= self.obs_thresh).astype(np.uint8)

        roi_cost = np.full(roi_obs.shape, -1, dtype=np.int8)
        roi_cost[roi_seen] = 0  # Free space

        if np.any(clean_obs):
            inv_obs = 1 - clean_obs
            dist_cells = cv2.distanceTransform(inv_obs, cv2.DIST_L2, 5)
            dist_m = dist_cells * self.resolution

            # 1. Lethal Obstacles (Cost = 100, Red in RViz)
            roi_cost[clean_obs == 1] = 100

            # 2. Inscribed Robot Radius (Cost = 99, Danger Zone)
            inscribed_mask = (clean_obs == 0) & (dist_m <= self.inscribed_radius) & roi_seen
            roi_cost[inscribed_mask] = 99

            # 3. Inflation Buffer Gradient (Cost = 98 -> 1, Cyan/Yellow Gradient)
            decay_mask = (dist_m > self.inscribed_radius) & (dist_m <= self.inflation_radius) & roi_seen
            decay_d = dist_m[decay_mask] - self.inscribed_radius
            decay_costs = 98.0 * np.exp(-self.cost_scaling_factor * decay_d)
            roi_cost[decay_mask] = np.clip(np.round(decay_costs).astype(np.int8), 1, 98)

        # Update full costmap
        self.costmap_data[min_r:max_r, min_c:max_c] = roi_cost

        # ── Publish Global Cumulative Costmap (/costmap) ───────────────
        costmap_msg = OccupancyGrid()
        costmap_msg.header.stamp = self.get_clock().now().to_msg()
        costmap_msg.header.frame_id = self.global_frame
        costmap_msg.info.resolution = self.resolution
        costmap_msg.info.width = self.grid_w
        costmap_msg.info.height = self.grid_h
        costmap_msg.info.origin.position.x = self.origin_x
        costmap_msg.info.origin.position.y = self.origin_y
        costmap_msg.info.origin.position.z = 0.0
        costmap_msg.info.origin.orientation.w = 1.0
        costmap_msg.data = self.costmap_data.flatten().tolist()

        self.costmap_pub.publish(costmap_msg)

        # ── Also Publish Rolling Local Costmap (/local_costmap/costmap) ─
        # 6m x 6m local window centered on current robot position
        local_w_m = 6.0
        local_cells = int(local_w_m / self.resolution)
        lr_c = int(round((self.robot_x - self.origin_x) / self.resolution))
        lr_r = int(round((self.robot_y - self.origin_y) / self.resolution))

        l_c0 = max(0, lr_c - local_cells // 2)
        l_c1 = min(self.grid_w, l_c0 + local_cells)
        l_r0 = max(0, lr_r - local_cells // 2)
        l_r1 = min(self.grid_h, l_r0 + local_cells)

        local_patch = self.costmap_data[l_r0:l_r1, l_c0:l_c1]
        local_msg = OccupancyGrid()
        local_msg.header.stamp = costmap_msg.header.stamp
        local_msg.header.frame_id = self.global_frame
        local_msg.info.resolution = self.resolution
        local_msg.info.width = local_patch.shape[1]
        local_msg.info.height = local_patch.shape[0]
        local_msg.info.origin.position.x = self.origin_x + l_c0 * self.resolution
        local_msg.info.origin.position.y = self.origin_y + l_r0 * self.resolution
        local_msg.info.origin.position.z = 0.0
        local_msg.info.origin.orientation.w = 1.0
        local_msg.data = local_patch.flatten().tolist()

        self.local_costmap_pub.publish(local_msg)

    # ── SLAM OccupancyGrid Inflation Handler ──────────────────────────
    def _process_and_pub_slam_costmap(self, msg: OccupancyGrid):
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution

        if width == 0 or height == 0 or resolution <= 0.0:
            return

        raw_data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        obs_raw = (raw_data >= self.obs_thresh).astype(np.uint8)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        obs_clean = cv2.morphologyEx(obs_raw, cv2.MORPH_OPEN, kernel)

        costmap = np.full((height, width), -1, dtype=np.int8)
        known_mask = (raw_data >= 0)
        costmap[known_mask] = 0

        if np.any(obs_clean):
            inv_obs = 1 - clean_obs
            dist_cells = cv2.distanceTransform(inv_obs, cv2.DIST_L2, 5)
            dist_m = dist_cells * resolution

            costmap[obs_clean == 1] = 100
            inscribed_mask = (obs_clean == 0) & (dist_m <= self.inscribed_radius) & known_mask
            costmap[inscribed_mask] = 99

            decay_mask = (dist_m > self.inscribed_radius) & (dist_m <= self.inflation_radius) & known_mask
            decay_d = dist_m[decay_mask] - self.inscribed_radius
            decay_costs = 98.0 * np.exp(-self.cost_scaling_factor * decay_d)
            costmap[decay_mask] = np.clip(np.round(decay_costs).astype(np.int8), 1, 98)

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
