#!/usr/bin/env python3
"""
CNN Driver Node — ROS 2 controller cho robot tự hành giữa hàng bắp.

Tách biệt các khối theo mô hình: Perception ↓ Decision ↓ Planning ↓ Control ↓ Hardware.
- Decision: FSMCoordinator (fsm.py)
- Planning: RRTStarPlanner (planners.py)
- Control: TrackingControllerSMC, PurePursuitController (controllers.py)
"""

import os
import sys
import math
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, NavSatFix, NavSatStatus, Imu
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None

from my_robot_controller.inference_handler import InferenceHandler
from my_robot_controller.interfaces import PlannerInterface, ControllerInterface
from my_robot_controller.planners import RRTStarPlanner
from my_robot_controller.controllers import TrackingControllerSMC, PurePursuitController, ControllerManager, SafetyController
from my_robot_controller.fsm import FSMCoordinator, FSMState, FSMEvent
from my_robot_controller.path_utils import Path
from my_robot_controller.lidar_processor import LidarProcessor
from my_robot_controller.perception_manager import PerceptionManager, EndOfRowDetector
from my_robot_controller.localization_manager import LocalizationManager
from my_robot_controller.telemetry_logger import TelemetryLogger



class CnnDriverNode(Node):
    def __init__(self):
        super().__init__('cnn_driver_node')
        self.get_logger().info("Initializing standardized FSM cnn_driver_node...")

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('model_path', '')
        self.declare_parameter('input_height', 512)
        self.declare_parameter('input_width', 512)
        self.declare_parameter('num_threads', 0)
        self.declare_parameter('mask_threshold', 0.04)
        self.declare_parameter('linear_speed', 0.30)
        self.declare_parameter('turn_linear_speed', 0.20)
        self.declare_parameter('turn_angular_speed', 0.60)
        self.declare_parameter('low_confidence_threshold', 0.35)
        self.declare_parameter('high_confidence_threshold', 0.50)
        self.declare_parameter('lambda_smc', 2.0)
        self.declare_parameter('k_smc', 3.5)
        self.declare_parameter('eta_smc', 0.6)
        self.declare_parameter('phi_smc', 0.5)
        self.declare_parameter('max_steering_angle_deg', 10.0)
        self.declare_parameter('ema_alpha', 0.45)
        self.declare_parameter('warmup_time', 1.0)
        self.declare_parameter('navigation_mode', 'auto_three_lanes')
        self.declare_parameter('min_row_length', 5.0)
        self.declare_parameter('max_row_length', 30.0)
        self.declare_parameter('low_conf_frames_threshold', 15)
        self.declare_parameter('drive_out_distance', 0.70)
        self.declare_parameter('min_turn_angle_deg', 140.0)
        self.declare_parameter('max_turn_angle_deg', 200.0)
        self.declare_parameter('reactive_avoid_wait_time', 3.0)  # seconds to wait before planning bypass
        self.declare_parameter('recovery_backup_distance', 1.0)  # meters to back up during recovery
        self.declare_parameter('use_hsv_mask', False)
        self.declare_parameter('datum_latitude', 10.775667)
        self.declare_parameter('datum_longitude', 106.670889)
        self.declare_parameter('datum_altitude', 10.0)
        self.declare_parameter('gps_topic', '/gps/fix')
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('imu_topic', '/imu/data')
        
        default_log_dir = os.path.join(
            os.path.expanduser('~'), 'ros2_telemetry_logs'
        )
        self.declare_parameter('enable_file_logging', True)
        self.declare_parameter('log_output_dir', default_log_dir)
        self.declare_parameter('terminal_log_interval', 1.0)
        self.declare_parameter('save_debug_imgs', False)

        p = self.get_parameter
        self.model_path               = p('model_path').value
        self.input_height             = p('input_height').value
        self.input_width              = p('input_width').value
        self.num_threads              = p('num_threads').value
        self.mask_threshold           = p('mask_threshold').value
        self.linear_speed             = p('linear_speed').value
        self.turn_linear_speed        = p('turn_linear_speed').value
        self.turn_angular_speed       = p('turn_angular_speed').value
        self.low_confidence_threshold = p('low_confidence_threshold').value
        self.high_confidence_threshold= p('high_confidence_threshold').value
        self.lambda_smc               = p('lambda_smc').value
        self.k_smc                    = p('k_smc').value
        self.eta_smc                  = p('eta_smc').value
        self.phi_smc                  = p('phi_smc').value
        self.max_steering_angle_deg   = p('max_steering_angle_deg').value
        self.ema_alpha                = p('ema_alpha').value
        self.warmup_time              = p('warmup_time').value
        self.navigation_mode          = p('navigation_mode').value
        self.min_row_length           = p('min_row_length').value
        self.max_row_length           = p('max_row_length').value
        self.low_conf_frames_threshold= p('low_conf_frames_threshold').value
        self.drive_out_distance       = p('drive_out_distance').value
        self.min_turn_angle_deg       = p('min_turn_angle_deg').value
        self.max_turn_angle_deg       = p('max_turn_angle_deg').value
        self.reactive_avoid_wait_time = p('reactive_avoid_wait_time').value
        self.recovery_backup_distance = p('recovery_backup_distance').value
        self.use_hsv_mask             = p('use_hsv_mask').value
        self.datum_latitude          = p('datum_latitude').value
        self.datum_longitude         = p('datum_longitude').value
        self.datum_altitude          = p('datum_altitude').value
        self.gps_topic               = p('gps_topic').value
        self.image_topic             = p('image_topic').value
        self.odom_topic              = p('odom_topic').value
        self.imu_topic               = p('imu_topic').value
        self.enable_file_logging     = p('enable_file_logging').value
        self.log_output_dir          = p('log_output_dir').value
        self.terminal_log_interval   = p('terminal_log_interval').value
        self.save_debug_imgs         = p('save_debug_imgs').value
        self._last_terminal_log_time = 0.0

        if self.enable_file_logging:
            self.telemetry_logger = TelemetryLogger(log_dir=self.log_output_dir)
        else:
            self.telemetry_logger = None

        # ── CV Bridge ─────────────────────────────────────────────────
        self.bridge = CvBridge() if CvBridge is not None else None
        if self.bridge is None:
            self.get_logger().warn("CvBridge unavailable — using numpy fallback.")

        if not self.use_hsv_mask:
            if not self.model_path or not os.path.exists(self.model_path):
                self.get_logger().info(f"Resolving model path for '{self.model_path}'...")
                
                # 1. Try my_robot_controller share directory
                try:
                    from ament_index_python.packages import get_package_share_directory
                    share_dir = get_package_share_directory('my_robot_controller')
                    fallback_share = os.path.join(share_dir, 'models', 'crop_row_cnn_best_final.onnx')
                    if os.path.exists(fallback_share):
                        self.model_path = fallback_share
                        self.get_logger().info(f"Using model from my_robot_controller share: {self.model_path}")
                except Exception:
                    pass

                # 2. Try luanvan_control share directory
                if not self.model_path or not os.path.exists(self.model_path):
                    try:
                        from ament_index_python.packages import get_package_share_directory
                        share_dir = get_package_share_directory('luanvan_control')
                        fallback_share = os.path.join(share_dir, 'models', 'crop_row_cnn_best_final.onnx')
                        if os.path.exists(fallback_share):
                            self.model_path = fallback_share
                            self.get_logger().info(f"Using model from luanvan_control share: {self.model_path}")
                    except Exception:
                        pass

            if not self.model_path or not os.path.exists(self.model_path):
                # 3. Fallback to relative path from source files
                current_dir = os.path.dirname(os.path.abspath(__file__))
                fallback_source = os.path.abspath(os.path.join(current_dir, '..', 'models', 'crop_row_cnn_best_final.onnx'))
                if os.path.exists(fallback_source):
                    self.model_path = fallback_source
                    self.get_logger().info(f"Using model from source: {self.model_path}")

            if not self.model_path or not os.path.exists(self.model_path):
                self.get_logger().error(f"ONNX model not found anywhere: {self.model_path}")
                sys.exit(1)

        self.inference = InferenceHandler(
            model_path=self.model_path,
            mask_threshold=self.mask_threshold,
            input_size=(self.input_height, self.input_width),
            use_hsv_mask=self.use_hsv_mask,
            num_threads=self.num_threads
        )
        self.lidar_processor = LidarProcessor()
        self.eor_detector = EndOfRowDetector(
            min_row_distance=self.min_row_length,
            low_confidence_threshold=self.low_confidence_threshold
        )
        self.perception_manager = PerceptionManager(
            inference_handler=self.inference,
            lidar_processor=self.lidar_processor,
            eor_detector=self.eor_detector
        )
        self.localization_manager = LocalizationManager(
            datum_lat=self.datum_latitude,
            datum_lon=self.datum_longitude,
            datum_alt=self.datum_altitude
        )

        # ── Decoupled Architecture Components ─────────────────────────
        self.fsm = FSMCoordinator(FSMState.TRACKING)
        
        self.planner = RRTStarPlanner()
        self.planner.initialize(
            step_size=0.5,
            max_iter=300,
            search_radius=1.2,
            robot_radius=0.38
        )

        self.tracking_controller = TrackingControllerSMC()
        self.tracking_controller.initialize(
            lambda_smc=self.lambda_smc,
            k_smc=self.k_smc,
            eta_smc=self.eta_smc,
            phi_smc=self.phi_smc,
            linear_speed=self.linear_speed,
            turn_angular_speed=self.turn_angular_speed
        )

        self.pure_pursuit_controller = PurePursuitController()
        self.pure_pursuit_controller.initialize(
            turn_linear_speed=self.turn_linear_speed,
            turn_angular_speed=self.turn_angular_speed
        )

        self.controller_manager = ControllerManager()
        self.controller_manager.register_controller('smc', self.tracking_controller)
        self.controller_manager.register_controller('pure_pursuit', self.pure_pursuit_controller)

        # ── FSM state variables ───────────────────────────────────────
        self.state_start_time       = None
        self.node_start_time        = None
        self.smoothed_angle_deg     = 0.0
        self.low_confidence_counter = 0
        self.high_confidence_counter = 0
        self.inside_row             = False

        # ── Odometry tracking ─────────────────────────────────────────
        self.distance_traveled  = 0.0    # m — cộng dồn từ đầu hàng
        self._prev_odom_x       = None
        self._prev_odom_y       = None
        self._odom_received     = False

        # New pose & orientation variables
        self.current_x          = 0.0
        self.current_y          = 0.0
        self.current_yaw        = 0.0

        # Drive out & turn tracking variables
        self.drive_out_start_x  = 0.0
        self.drive_out_start_y  = 0.0
        self.rotate_start_yaw   = 0.0
        self.accumulated_turn_angle = 0.0
        self._prev_yaw_for_turn = None
        self.turn_direction     = 1.0  # 1.0 = left (CCW), -1.0 = right (CW)

        # Recovery tracking variables
        self.recovery_start_x   = 0.0
        self.recovery_start_y   = 0.0

        # Navigation tracking
        self.last_visited_lane  = None  # can be 'lane1', 'lane2', 'lane3'
        self.latest_scan        = None
        self.last_path_completion_time = -100.0
        self.row_start_x        = None
        self.row_completed      = False
        self.uturn_goal_x       = None
        self.uturn_goal_y       = None
        self.uturn_target_lane  = None

        # ── Image logging ─────────────────────────────────────────────
        self.last_img_save_time = 0.0
        self.output_dir = os.path.join(os.path.expanduser('~'), 'ros2_debug_imgs')

        # ── Publishers & Subscribers ──────────────────────────────────
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.gps_pub     = self.create_publisher(NavSatFix, '/localization/gps', 10)

        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, 10)
        # Fallback subscription for simulation / raw image
        if self.image_topic not in ('/camera/image_raw', 'camera/image_raw'):
            self.image_fallback_sub = self.create_subscription(
                Image, '/camera/image_raw', self.image_callback, 10)

        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10)
        # Fallback subscription for raw odom
        if self.odom_topic not in ('/odom', 'odom'):
            self.odom_fallback_sub = self.create_subscription(
                Odometry, '/odom', self.odom_callback, 10)

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.gps_sub = self.create_subscription(
            NavSatFix, self.gps_topic, self.gps_callback, 10)

        self.imu_sub = self.create_subscription(
            Imu, self.imu_topic, self.imu_callback, 10)
        # Fallback subscription for alternative imu topic
        if self.imu_topic not in ('/imu', 'imu'):
            self.imu_fallback_sub = self.create_subscription(
                Imu, '/imu', self.imu_callback, 10)

        self.get_logger().info(
            f"cnn_driver_node ready | navigation_mode={self.navigation_mode} | "
            f"Image topic={self.image_topic} | Odom topic={self.odom_topic} | "
            f"min_row_length={self.min_row_length}m | drive_out_distance={self.drive_out_distance}m | "
            f"GPS topic={self.gps_topic} | Datum Lat/Lon=({self.datum_latitude:.6f}, {self.datum_longitude:.6f})"
        )

    # ── Compatibility Properties ─────────────────────────────────────
    @property
    def state(self):
        return self.fsm.get_state()

    @state.setter
    def state(self, value):
        self.fsm.set_state(value)

    @property
    def state_before_planning(self):
        return self.fsm.state_before_planning

    @state_before_planning.setter
    def state_before_planning(self, value):
        self.fsm.state_before_planning = value

    @property
    def planned_path(self):
        return self.pure_pursuit_controller.path

    @planned_path.setter
    def planned_path(self, value):
        self.pure_pursuit_controller.set_path(value)

    @property
    def path_index(self):
        return self.pure_pursuit_controller.path_index

    @path_index.setter
    def path_index(self, value):
        self.pure_pursuit_controller.path_index = value

    # ── FSM Coordination Interface Methods (Task 7 & 8) ──────────────
    def PlanPath(self, goal_x, goal_y):
        """Standard planning request wrapper."""
        start = [self.current_x, self.current_y]
        goal = [goal_x, goal_y]
        obstacles = self.lidar_processor.get_obstacles_global(
            self.current_x, self.current_y, self.current_yaw
        )
        return self.planner.plan(start, goal, obstacles)

    def StartTracking(self, dt):
        """Enable tracking controller and calculate output."""
        self.controller_manager.select_controller('smc')
        res = self.controller_manager.compute_command(self.smoothed_angle_deg, dt_actual=dt)
        return res["linear_velocity"], res["angular_velocity"]

    def FollowPath(self):
        """Enable path following controller and calculate output."""
        self.controller_manager.select_controller('pure_pursuit')
        res = self.controller_manager.compute_command(self.current_x, self.current_y, self.current_yaw)
        finished = (res["status"] == "COMPLETED")
        return res["linear_velocity"], res["angular_velocity"], finished, self.pure_pursuit_controller.path_index

    def StopRobot(self):
        """Issue zero velocities to the actuators."""
        self.controller_manager.reset()
        if self.controller_manager.active_name in self.controller_manager.controllers:
            self.controller_manager.controllers[self.controller_manager.active_name].stop()
        self.controller_manager.active_name = None
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        return twist

    def Recovery(self):
        """Perform recovery backup maneuver."""
        # Calculate backup distance
        dx = self.current_x - self.recovery_start_x
        dy = self.current_y - self.recovery_start_y
        dist = math.sqrt(dx*dx + dy*dy)

        twist = Twist()
        if dist < self.recovery_backup_distance:
            twist.linear.x = -0.2  # Back up slowly
            twist.angular.z = 0.0
            finished = False
        else:
            finished = True
        return twist, finished

    # ── GPS & IMU callbacks ───────────────────────────────────────────
    def imu_callback(self, msg: Imu):
        self.localization_manager.update_imu(msg)

    def gps_callback(self, msg: NavSatFix):
        self.localization_manager.update_gps(msg)
        self.publish_gps()

    def publish_gps(self):
        gps_info = self.localization_manager.get_gps_coordinates()
        nav_msg = NavSatFix()
        nav_msg.header.stamp = self.get_clock().now().to_msg()
        nav_msg.header.frame_id = 'gps_link'
        nav_msg.latitude = gps_info['latitude']
        nav_msg.longitude = gps_info['longitude']
        nav_msg.altitude = gps_info['altitude']
        nav_msg.status.status = NavSatStatus.STATUS_FIX if gps_info['status'] == 'FIX' else NavSatStatus.STATUS_NO_FIX
        self.gps_pub.publish(nav_msg)

    # ── Odometry callback ──────────────────────────────────────────────
    def odom_callback(self, msg: Odometry):
        self.localization_manager.update_odometry(msg)
        pose = self.localization_manager.get_pose()
        self.publish_gps()

        # Check diagnostics status
        if pose["status"] == "SENSOR_FAILED":
            self.get_logger().warn(f"Localization warning: {pose['status']}.")

        x = pose["x"]
        y = pose["y"]
        self.current_yaw = pose["yaw"]
        self.current_x = x
        self.current_y = y
        self._odom_received = True

        if self._prev_odom_x is not None:
            dx = x - self._prev_odom_x
            dy = y - self._prev_odom_y
            if self.fsm.get_state() == FSMState.TRACKING:
                self.distance_traveled += math.sqrt(dx*dx + dy*dy)

        self._prev_odom_x   = x
        self._prev_odom_y   = y

        # Accumulate turn angle if rotating
        is_uturn_active = (
            self.fsm.get_state() == FSMState.UTURN_EXECUTION or
            (self.fsm.get_state() == FSMState.PATH_FOLLOWING and self.fsm.state_before_planning == FSMState.UTURN_PLANNING) or
            self.fsm.get_state() == FSMState.UTURN_PLANNING
        )
        if is_uturn_active:
            if self._prev_yaw_for_turn is not None:
                dyaw = self.current_yaw - self._prev_yaw_for_turn
                # Normalize dyaw to [-pi, pi]
                dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))
                self.accumulated_turn_angle += abs(dyaw)
            self._prev_yaw_for_turn = self.current_yaw
        else:
            self._prev_yaw_for_turn = None

    # ── Image conversion ───────────────────────────────────────────────
    def convert_image(self, msg: Image) -> np.ndarray:
        if self.bridge is not None:
            return self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if msg.encoding in ('rgb8', 'bgr8'):
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                (msg.height, msg.width, 3))
            return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else img
        raise RuntimeError(f"Unsupported encoding: {msg.encoding}")

    # ── Transition helper ──────────────────────────────────────────────
    def transition_to_state(self, new_state, now):
        old_state = self.fsm.get_state()
        if self.fsm.set_state(new_state):
            self.state_start_time = now
            if new_state == FSMState.TRACKING:
                self.tracking_controller.reset()
            elif new_state == FSMState.RECOVERY:
                self.recovery_start_x = self.current_x
                self.recovery_start_y = self.current_y
            
            # Reset inside_row flag ONLY when transitioning out of row (UTURN / IDLE)
            if old_state == FSMState.TRACKING and new_state in [FSMState.UTURN_PLANNING, FSMState.UTURN_EXECUTION, FSMState.IDLE]:
                self.inside_row = False
                self.high_confidence_counter = 0
                
            self.get_logger().info(f"[FSM TRANSITION] {old_state} ──► {new_state}")
            if self.telemetry_logger:
                self.telemetry_logger.log_event("FSM_TRANSITION", f"{old_state} -> {new_state}")

    # ── Main image callback / FSM ──────────────────────────────────────
    def image_callback(self, msg: Image):
        # Prevent processing duplicate frames within 10ms
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        if hasattr(self, '_last_img_cb_ns') and (now_ns - self._last_img_cb_ns) < 10000000:
            return
        self._last_img_cb_ns = now_ns

        try:
            bgr_image = self.convert_image(msg)
            # Debug: Save the first image to verify camera is working (if enabled)
            if self.save_debug_imgs and not hasattr(self, '_debug_image_saved'):
                os.makedirs(self.output_dir, exist_ok=True)
                cv2.imwrite(os.path.join(self.output_dir, 'camera_test.png'), bgr_image)
                self.get_logger().debug("--- Saved camera_test.png ---")
                self._debug_image_saved = True
        except Exception as e:
            self.get_logger().error(f"Image convert failed: {e}")
            return

        # ── Perception Processing via PerceptionManager ───────────────
        perception = self.perception_manager.process_sensors(
            cv_image=bgr_image,
            distance_traveled=self.distance_traveled,
            rx=self.current_x,
            ry=self.current_y,
            ryaw=self.current_yaw,
            max_angle_deg=self.max_steering_angle_deg,
            inside_row=self.inside_row
        )
        
        confidence = perception["confidence"]
        obstacle_detected = perception["obstacle_detected"]
        end_of_row = perception["end_of_row_detected"]
        raw_angle = perception["heading_error"]
        lane_center = perception["lane_center"]
        lane_offset = perception["lane_offset"]

        # ── FSM ───────────────────────────────────────────────────────
        twist        = Twist()
        now          = self.get_clock().now()
        if self.node_start_time is None:
            self.node_start_time = now
            self.state_start_time = now
        elapsed_state= (now - self.state_start_time).nanoseconds / 1e9
        elapsed_total= (now - self.node_start_time).nanoseconds / 1e9
        warmup_done  = elapsed_total > self.warmup_time

        current_state = self.fsm.get_state()

        # Save diagnostic frame every 3 seconds (sim time, if enabled)
        if self.save_debug_imgs:
            now_sec = now.nanoseconds / 1e9
            if now_sec - self.last_img_save_time >= 3.0:
                self.last_img_save_time = now_sec
                try:
                    os.makedirs(self.output_dir, exist_ok=True)
                    if hasattr(self.inference, 'latest_mask') and self.inference.latest_mask is not None:
                        # Resize mask to match BGR image height and width
                        mask_resized = cv2.resize(self.inference.latest_mask, (bgr_image.shape[1], bgr_image.shape[0]))
                        
                        # Convert single-channel mask (0.0 to 1.0) to binary mask (0 or 255)
                        binary_mask = (mask_resized >= self.inference.mask_threshold).astype(np.uint8) * 255
                        binary_mask_colored = cv2.cvtColor(binary_mask, cv2.COLOR_GRAY2BGR)
                        
                        # Create overlay image: cyan highlight for segmented areas
                        overlay_image = bgr_image.copy()
                        overlay_image[binary_mask > 128] = [255, 255, 0] # Cyan BGR
                        blend_image = cv2.addWeighted(bgr_image, 0.6, overlay_image, 0.4, 0)
                        
                        # Draw guidance lines on blend_image
                        h, w = bgr_image.shape[:2]
                        image_center = (w - 1) / 2.0
                        
                        # 1. Lime dashed line for image center
                        for y_start in range(0, h, 20):
                            cv2.line(blend_image, (int(image_center), y_start), (int(image_center), min(y_start + 10, h)), (0, 255, 0), 2)
                        
                        # 2. Deepskyblue target line for lane center
                        lane_center_raw = lane_center * (w / self.inference.input_size[1])
                        lane_center_raw = np.clip(lane_center_raw, 0.0, w - 1.0)
                        line_top = int(h * 0.6)
                        cv2.line(blend_image, (int(lane_center_raw), h - 1), (int(lane_center_raw), line_top), (255, 191, 0), 3)
                        
                        # Concatenate horizontally: BGR, Mask, Overlay
                        canvas = np.hstack((bgr_image, binary_mask_colored, blend_image))
                        
                        # Add text details for easier debugging
                        cv2.putText(canvas, f"State: {current_state} | Conf: {confidence:.2f} | Dist: {self.distance_traveled:.2f}m | Steer: {self.smoothed_angle_deg:.2f} deg", 
                                    (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
                        
                        # Save the image
                        filename = os.path.join(self.output_dir, f"frame_{int(now_sec)}.png")
                        cv2.imwrite(filename, canvas)
                        self.get_logger().debug(f"--- Saved diagnostic frame: {filename} ---")
                except Exception as save_err:
                    self.get_logger().error(f"Failed to save diagnostic frame: {save_err}")

        # Check if we have entered the row dynamically (arm when settled inside the lane)
        if current_state == FSMState.TRACKING and not self.inside_row:
            dir_fwd = (math.cos(self.current_yaw) >= 0)
            in_row_zone = (self.current_x >= 0.20 or confidence >= self.low_confidence_threshold) if dir_fwd else (self.current_x <= 4.30 or confidence >= self.low_confidence_threshold)
            
            if in_row_zone and confidence >= 0.15:
                self.inside_row = True
                if self.row_start_x is None:
                    self.row_start_x = self.current_x
                self.row_completed = False
                self.get_logger().info(f"--- Robot dynamically entered crop row at x={self.row_start_x:.2f}m (confidence={confidence:.2f}) ---")

        # ── IDLE ──────────────────────────────────────────────────────
        if current_state == FSMState.IDLE:
            self.StopRobot()
            if confidence >= self.high_confidence_threshold:
                self.transition_to_state(FSMState.TRACKING, now)
            return

        # ── TRACKING ──────────────────────────────────────────────────
        elif current_state == FSMState.TRACKING:
            # 1. Obstacle avoidance check (ONLY active when inside the crop row)
            post_path_cooldown = (now_sec - self.last_path_completion_time) < 0.6
            if self.inside_row and obstacle_detected and not post_path_cooldown:
                self.get_logger().warn("Confirmed obstacle in front corridor! Starting smooth avoidance maneuver...")
                self.transition_to_state(FSMState.AVOID_PLANNING, now)
                self.StopRobot()
                return

            # 2. End-of-Row exit distance handling
            if getattr(self, 'eor_detected', False):
                dx = self.current_x - self.eor_trigger_x
                if abs(dx) >= self.drive_out_distance:
                    self.get_logger().info(
                        f"--- Cleared row completely (traveled {abs(dx):.2f}m past EOR). Starting U-turn... ---"
                    )
                    self.eor_detected = False
                    self.transition_to_state(FSMState.UTURN_PLANNING, now)
                    self.low_confidence_counter = 0
                    self.smoothed_angle_deg  = 0.0
                    self.distance_traveled   = 0.0
                    self.inside_row          = False
                    self.row_start_x         = None
                    self.row_completed       = False
                    self.accumulated_turn_angle = 0.0
                    self.StopRobot()
                    return
                else:
                    lane_center = round(self.current_y - 0.5) + 0.5
                    dir_x = 1.0 if math.cos(self.current_yaw) >= 0 else -1.0
                    target_yaw = 0.0 if dir_x > 0 else (math.pi if self.current_yaw >= 0 else -math.pi)
                    yaw_err = math.atan2(math.sin(target_yaw - self.current_yaw), math.cos(target_yaw - self.current_yaw))
                    lat_correction = -1.5 * (self.current_y - lane_center) * dir_x
                    twist.linear.x = self.linear_speed
                    twist.angular.z = float(np.clip(lat_correction + 0.8 * yaw_err, -0.30, 0.30))
                    self.cmd_vel_pub.publish(twist)
                    return

            # Skip-zero: ignore steering commands if confidence is low, and decay to straight driving
            failed = (confidence < self.low_confidence_threshold)
            if not failed:
                self.smoothed_angle_deg = (
                    self.ema_alpha * raw_angle
                    + (1.0 - self.ema_alpha) * self.smoothed_angle_deg
                )
            else:
                self.smoothed_angle_deg *= 0.90

            # Sliding Mode Control (SMC) via modular TrackingControllerSMC
            now_sec = now.nanoseconds / 1e9
            if hasattr(self, '_prev_image_time'):
                dt_actual = now_sec - self._prev_image_time
            else:
                dt_actual = 0.067
            self._prev_image_time = now_sec
            dt_actual = np.clip(dt_actual, 0.001, 1.0)
            
            lin_speed, ang_vel = self.StartTracking(dt_actual)
            twist.linear.x = lin_speed
            twist.angular.z = ang_vel

            # ── Pure Perception U-turn triggers ───────────────────────
            if warmup_done:
                # 1. CNN Confidence trigger
                if confidence < self.low_confidence_threshold:
                    self.low_confidence_counter += 1
                else:
                    self.low_confidence_counter = 0
                
                trigger_confidence = (self.low_confidence_counter >= self.low_conf_frames_threshold)
                
                # 2. Safety distance backup (safety net if vision/LiDAR is degraded)
                trigger_safety_distance = (self.distance_traveled >= self.max_row_length)

                # ── Pure Perception U-turn triggers ───────────────────────
                # Tự động nhận biết hết hàng hoàn toàn bằng đa cảm biến (Perception-driven):
                # 1. LiDAR C1 phát hiện khoảng trống đầu bờ (phía trước > 2.0m, 2 bên sườn > 0.90m)
                # 2. Camera CNN mất dấu hàng thùng khi ra khỏi luống (confidence tụt giảm)
                # 3. Watchdog khoảng cách an toàn khẩn cấp (phòng ngừa cả 2 cảm biến bị lỗi phần cứng)
                if self.inside_row and (end_of_row or trigger_confidence or trigger_safety_distance):
                    self.row_completed = True
                    reason = "LiDAR Headland Clearance" if end_of_row else ("Vision Confidence Drop" if trigger_confidence else "Safety Distance Watchdog")
                    self.get_logger().info(
                        f"--- Đã nhận biết HẾT HÀNG tự động bằng cảm biến ({reason}) tại x={self.current_x:.2f}m! Bắt đầu tự lập kế hoạch quay đầu... ---"
                    )
                    self.eor_detected = True
                    self.eor_trigger_x = self.current_x
                    return

        # ── REACTIVE_AVOID ────────────────────────────────────────────
        elif current_state == FSMState.REACTIVE_AVOID:
            self.StopRobot()
            
            # Check if obstacle has cleared
            if not obstacle_detected:
                self.get_logger().info("Obstacle cleared! Resuming TRACKING...")
                self.transition_to_state(FSMState.TRACKING, now)
                return
                
            # If wait time exceeded, transition to AVOID_PLANNING (Task 9)
            if elapsed_state >= self.reactive_avoid_wait_time:
                self.get_logger().warn(f"Obstacle still present after {self.reactive_avoid_wait_time}s. Transitioning to AVOID_PLANNING...")
                self.transition_to_state(FSMState.AVOID_PLANNING, now)
                return

        # ── AVOID_PLANNING ────────────────────────────────────────────
        elif current_state == FSMState.AVOID_PLANNING:
            dir_x = 1.0 if math.cos(self.current_yaw) >= 0 else -1.0
            goal_x = self.current_x + dir_x * 4.0
            goal_y = self.current_y

            # Generate smooth lane-constrained polynomial avoidance trajectory
            waypoints = self.generate_backup_avoidance_path(goal_x, goal_y)
            if waypoints:
                self.pure_pursuit_controller.set_path(waypoints)
                self.transition_to_state(FSMState.PATH_FOLLOWING, now)
            else:
                self.get_logger().error("Avoidance path generation failed! Transitioning to RECOVERY...")
                self.transition_to_state(FSMState.RECOVERY, now)

        # ── UTURN_PLANNING ────────────────────────────────────────────
        elif current_state == FSMState.UTURN_PLANNING:
            if self.last_visited_lane is None:
                if self.current_y < 0.0:
                    self.last_visited_lane = 'lane_lower'
                else:
                    self.last_visited_lane = 'lane_upper'

            # Shift lane by -1.0m when exiting Upper Lane (+y), +1.0m when exiting Lower Lane (-y)
            if self.last_visited_lane == 'lane_lower':
                target_y = self.current_y + 1.00
                current_lane = 'lane_upper'
                self.turn_direction = 1.0  # Left / CCW turn
            else:
                target_y = self.current_y - 1.00
                current_lane = 'lane_lower'
                self.turn_direction = -1.0 # Right / CW turn

            dir_x = 1.0 if math.cos(self.current_yaw) >= 0 else -1.0
            goal_x = self.current_x - dir_x * 0.50
            goal_y = target_y

            self.get_logger().info(f"Targeting lane transition: -> target_y={target_y:.2f} (shift={self.turn_direction * 1.00:.2f}m, dir={self.turn_direction})")
            
            # Reset U-turn turn tracking variables
            self.uturn_start_yaw = self.current_yaw
            self.accumulated_turn_angle = 0.0
            self._prev_yaw_for_turn = self.current_yaw

            # 1. Generate smooth, mathematical semicircular U-turn trajectory (180 deg arc)
            waypoints = self.generate_backup_uturn_path(goal_x, goal_y)
            if not waypoints:
                self.get_logger().warn("Semicircular U-turn failed! Attempting RRT* path planner...")
                plan_res = self.PlanPath(goal_x, goal_y)
                path = plan_res["path"] if plan_res else None
                waypoints = path.waypoints if hasattr(path, 'waypoints') else path

            if waypoints:
                self.pure_pursuit_controller.set_path(waypoints)
                self.last_visited_lane = current_lane
                self.transition_to_state(FSMState.PATH_FOLLOWING, now)
            else:
                self.get_logger().error("UTurn path generation failed! Transitioning to RECOVERY...")
                self.transition_to_state(FSMState.RECOVERY, now)

        # ── PATH_FOLLOWING ────────────────────────────────────────────
        elif current_state == FSMState.PATH_FOLLOWING:
            # Use decoupled PurePursuitController wrapper (Task 8)
            lin_vel, ang_vel, finished, path_idx = self.FollowPath()
            
            is_uturn = (self.fsm.state_before_planning == FSMState.UTURN_PLANNING)
            
            if is_uturn:
                turn_angle_deg = np.rad2deg(self.accumulated_turn_angle)
                is_turned_around = (turn_angle_deg >= 135.0)
                caught_row = (confidence >= self.high_confidence_threshold and is_turned_around) or (finished and is_turned_around)
                
                if caught_row:
                    self.get_logger().info(
                        f"U-turn alignment completed and new row verified (conf={confidence:.2f}, turn={turn_angle_deg:.1f}°) -> TRACKING..."
                    )
                    self.StopRobot()
                    self.transition_after_path(now)
                elif finished:
                    self.get_logger().warn(
                        "U-turn path finished but new row not caught yet. Transitioning to UTURN_EXECUTION to sweep..."
                    )
                    self.transition_to_state(FSMState.UTURN_EXECUTION, now)
                else:
                    twist.linear.x = lin_vel
                    twist.angular.z = ang_vel
            else:
                lane_center = round(self.current_y - 0.5) + 0.5
                returned_to_center = (abs(self.current_y - lane_center) < 0.04) and (path_idx >= 12)
                
                # Proactive chained avoidance: if already back in center corridor and sees next obstacle ahead:
                if returned_to_center and obstacle_detected:
                    self.get_logger().warn("Returned to center and detected next obstacle! Planning next avoidance...")
                    self.transition_to_state(FSMState.AVOID_PLANNING, now)
                    self.StopRobot()
                    return
                elif finished or (returned_to_center and confidence >= self.high_confidence_threshold):
                    self.get_logger().info("Avoidance maneuver completely finished! Returning to TRACKING...")
                    self.transition_after_path(now)
                else:
                    twist.linear.x = lin_vel
                    twist.angular.z = ang_vel

        # ── UTURN_EXECUTION ───────────────────────────────────────────
        elif current_state == FSMState.UTURN_EXECUTION:
            # Pivot/rotate continuously in place until CNN detects the new row
            # Use forward velocity so it traces a smooth wider arc into the row
            twist.linear.x  = 0.18
            twist.angular.z = self.turn_direction * self.turn_angular_speed
            
            turn_angle_deg = np.rad2deg(self.accumulated_turn_angle)
            
            # ONLY transition to TRACKING if CNN catches the new row AND we have turned sufficiently (>= 140 degrees)!
            if turn_angle_deg >= self.min_turn_angle_deg:
                if confidence >= self.high_confidence_threshold:
                    self.get_logger().info(
                        f"--- New crop row caught in UTURN_EXECUTION (conf={confidence:.2f}, turn={turn_angle_deg:.1f}°) → TRACKING ---"
                    )
                    self.StopRobot()
                    self.transition_after_path(now)

        # ── RECOVERY ──────────────────────────────────────────────────
        elif current_state == FSMState.RECOVERY:
            rec_twist, finished = self.Recovery()
            
            if finished:
                self.get_logger().info("Recovery maneuver successfully completed. Returning to TRACKING...")
                self.transition_to_state(FSMState.TRACKING, now)
                self.distance_traveled = 0.0
            else:
                twist = rec_twist

        # ── EMERGENCY_STOP ────────────────────────────────────────────
        elif current_state == FSMState.EMERGENCY_STOP:
            self.StopRobot()
        self.cmd_vel_pub.publish(twist)

        # ── Telemetry File Logging & Standardized Terminal Status Output ──
        pose_info = self.localization_manager.get_pose()
        gps_info = pose_info.get('gps', {})
        imu_info = pose_info.get('imu', {})
        if self.enable_file_logging and self.telemetry_logger:
            self.telemetry_logger.log_telemetry({
                'fsm_state': current_state,
                'x': self.current_x,
                'y': self.current_y,
                'yaw': self.current_yaw,
                'steering_angle_deg': self.smoothed_angle_deg,
                'linear_velocity': twist.linear.x,
                'angular_velocity': twist.angular.z,
                'imu_yaw': imu_info.get('yaw', 0.0),
                'imu_angular_vel_z': imu_info.get('angular_vel_z', 0.0),
                'imu_accel_x': imu_info.get('linear_accel_x', 0.0),
                'confidence': confidence,
                'distance_traveled': self.distance_traveled,
                'gps_latitude': gps_info.get('latitude', 0.0),
                'gps_longitude': gps_info.get('longitude', 0.0),
                'gps_altitude': gps_info.get('altitude', 0.0),
                'gps_dms': gps_info.get('dms', ''),
                'gps_status': gps_info.get('status', 'NO_FIX')
            })

        # Throttled Clean Terminal & Text Log Summary (Synchronized)
        now_sec = now.nanoseconds / 1e9
        if now_sec - self._last_terminal_log_time >= self.terminal_log_interval:
            self._last_terminal_log_time = now_sec
            display_steer_deg = self.smoothed_angle_deg
            if current_state != FSMState.TRACKING and abs(twist.linear.x) > 0.01:
                display_steer_deg = math.degrees(math.atan2(twist.angular.z * 0.58, twist.linear.x))
            status_msg = (
                f"[STATUS] [{current_state:^15s}] | Steer: {display_steer_deg:+5.2f}° | "
                f"Vel: ({twist.linear.x:4.2f}m/s, {twist.angular.z:+4.2f}r/s) | "
                f"Pose: ({self.current_x:5.2f}m, {self.current_y:5.2f}m) | "
                f"GPS: ({gps_info.get('latitude', 0.0):.6f}°, {gps_info.get('longitude', 0.0):.6f}° [{gps_info.get('status', 'NO_FIX')}])"
            )
            self.get_logger().info(status_msg)
            if self.enable_file_logging and self.telemetry_logger:
                self.telemetry_logger.log_event("SYSTEM_STATUS", status_msg)

    # ── Laser Scan Callback ──────────────────────────────────────────
    def scan_callback(self, msg: LaserScan):
        self.lidar_processor.update_scan(msg)

    # ── Fallback U-turn path generator ──────────────────────────────
    def generate_backup_uturn_path(self, goal_x, goal_y):
        self.get_logger().info(f"Generating smooth semicircular U-turn path starting near exit x={self.current_x:.2f}m...")
        rx = self.current_x
        ry = self.current_y
        ryaw = self.current_yaw
        dir_x = 1.0 if math.cos(ryaw) >= 0 else -1.0
        
        shift = goal_y - ry
        clearance = 0.15  # Tight clearance since robot already drove out past EOR
        
        waypoints = []
        
        # 1. Drive out slightly for smooth arc entry (0.15m)
        for d in np.linspace(0.05, clearance, 3):
            waypoints.append([rx + dir_x * d, ry])
            
        # 2. Perfect mathematical circular arc U-turn (180 degree turn to target_y)
        x_exit = rx + dir_x * clearance
        y_mid = (ry + goal_y) / 2.0
        R = abs(shift) / 2.0
        y_sign = 1.0 if shift < 0 else -1.0
        
        num_arc_points = 16
        for theta in np.linspace(0.0, math.pi, num_arc_points):
            wp_x = x_exit + dir_x * R * math.sin(theta)
            wp_y = y_mid + R * math.cos(theta) * y_sign
            waypoints.append([wp_x, wp_y])
            
        # 3. Short entry alignment section into the new row (0.40m length)
        for d in np.linspace(0.10, 0.40, 4):
            waypoints.append([x_exit - dir_x * d, goal_y])
            
        return Path(waypoints, planner_type="DynamicUTurn")

    # ── Safe Row Avoidance path generator ───────────────────────────
    def generate_backup_avoidance_path(self, goal_x, goal_y):
        self.get_logger().warn("Generating smooth Quintic Polynomial avoidance trajectory with wide safety buffer...")
        rx = self.current_x
        ry = self.current_y
        ryaw = self.current_yaw
        dir_x = 1.0 if math.cos(ryaw) >= 0 else -1.0
        
        # Extract full geometric obstacle info from LiDAR in global frame
        obs_info = self.lidar_processor.get_front_obstacle_info(rx, ry, ryaw, max_dist=1.80)
        lane_center = obs_info["lane_center"]
        x_obs = obs_info["x_obs"]
        side = obs_info["side"]
        
        # Target lateral shift: 0.09m from center (balanced ~16cm clearance to stalks, ~12cm to obstacle)
        if side == "LEFT":
            nudge_y = lane_center - (0.09 if dir_x > 0 else -0.09)
            self.get_logger().info(f"Obstacle on LEFT at x={x_obs:.2f}m, y={obs_info['y_obs']:.2f}m -> Weaving to y={nudge_y:.2f}m")
        else:
            nudge_y = lane_center + (0.09 if dir_x > 0 else -0.09)
            self.get_logger().info(f"Obstacle on RIGHT at x={x_obs:.2f}m, y={obs_info['y_obs']:.2f}m -> Weaving to y={nudge_y:.2f}m")

        # Hard clamp nudge_y to remain strictly within the safe 1.0m crop lane
        nudge_y = float(np.clip(nudge_y, lane_center - 0.10, lane_center + 0.10))

        waypoints = []
        
        # ── Stage 1: Weave Out (Reaches nudge_y at 0.25m before x_obs)
        x_weave_end = x_obs - dir_x * 0.25
        weave_length = max(0.35, dir_x * (x_weave_end - rx))
        for t in np.linspace(0.05, 1.0, 8):
            s = 10.0 * (t**3) - 15.0 * (t**4) + 6.0 * (t**5)
            wp_x = rx + dir_x * (t * weave_length)
            wp_y = ry + s * (nudge_y - ry)
            waypoints.append([wp_x, wp_y])
            
        # ── Stage 2: Parallel Clearance Corridor (Past x_obs by 0.30m)
        x_clear_end = x_obs + dir_x * 0.30
        clear_length = max(0.30, dir_x * (x_clear_end - (rx + dir_x * weave_length)))
        for d in np.linspace(0.06, clear_length, 6):
            wp_x = rx + dir_x * (weave_length + d)
            wp_y = nudge_y
            waypoints.append([wp_x, wp_y])
            
        # ── Stage 3: Smooth Quintic Polynomial Return (over 0.35m to lane_center)
        x_return_start = rx + dir_x * (weave_length + clear_length)
        return_length = 0.35
        for t in np.linspace(0.05, 1.0, 6):
            s = 10.0 * (t**3) - 15.0 * (t**4) + 6.0 * (t**5)
            wp_x = x_return_start + dir_x * (t * return_length)
            wp_y = nudge_y + s * (lane_center - nudge_y)
            waypoints.append([wp_x, wp_y])
            
        return Path(waypoints, planner_type="SafeCropAvoidance")

    # ── Transition after path completed ─────────────────────────────
    def transition_after_path(self, now=None):
        if now is None:
            now = self.get_clock().now()
            
        is_uturn = (self.fsm.state_before_planning == FSMState.UTURN_PLANNING)
        
        self.get_logger().info("Path following completed. Returning to TRACKING...")
        self.last_path_completion_time = now.nanoseconds / 1e9
        self.transition_to_state(FSMState.TRACKING, now)
        self.low_confidence_counter = 0
        self.smoothed_angle_deg = 0.0
        self.distance_traveled = 0.0
        
        if is_uturn:
            self.inside_row = True  # Arm inside_row immediately so obstacle avoidance is active in Row 2!
            self.row_start_x = self.current_x
            self.row_completed = False
            self.eor_detected = False
            self.accumulated_turn_angle = 0.0
        else:
            self.inside_row = True


def main(args=None):
    rclpy.init(args=args)
    node = CnnDriverNode()
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
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
    main()
