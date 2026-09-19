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
import time
import math
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan, NavSatFix, NavSatStatus, Imu
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path as RosPath
from std_msgs.msg import Float32MultiArray
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
from my_robot_controller.omega_planner import OmegaTurnPlanner
from my_robot_controller.lidar_processor import LidarProcessor
from my_robot_controller.perception_manager import PerceptionManager, EndOfRowDetector
from my_robot_controller.localization_manager import LocalizationManager
from my_robot_controller.telemetry_logger import TelemetryLogger



class CnnDriverNode(Node):
    def __init__(self):
        super().__init__('cnn_driver_node')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('model_path', '')
        self.declare_parameter('input_height', 512)
        self.declare_parameter('input_width', 512)
        self.declare_parameter('num_threads', 0)
        self.declare_parameter('mask_threshold', 0.04)
        self.declare_parameter('linear_speed', 0.20)
        self.declare_parameter('turn_linear_speed', 0.20)
        self.declare_parameter('turn_angular_speed', 0.40)
        self.declare_parameter('low_confidence_threshold', 0.35)
        self.declare_parameter('high_confidence_threshold', 0.50)
        self.declare_parameter('lambda_smc', 2.0)
        self.declare_parameter('k_smc', 3.5)
        self.declare_parameter('eta_smc', 0.6)
        self.declare_parameter('phi_smc', 0.5)
        self.declare_parameter('max_steering_angle_deg', 14.0)
        self.declare_parameter('turn_in_place_threshold_deg', 0.30)
        self.declare_parameter('turn_in_place_resume_deg', 0.30)
        self.declare_parameter('row_spacing', 0.90)
        self.declare_parameter('ema_alpha', 0.45)
        self.declare_parameter('enable_uturn', False)
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

        # ── Tham số Điều Phối Nhiệm Vụ 2 Luống & Quỹ Đạo Omega Turn ───
        self.declare_parameter('enable_mission_goals', True)
        self.declare_parameter('enable_global_plan_assist', True)
        self.declare_parameter('field_length', 3.0)
        self.declare_parameter('headland_area_length', 4.0)
        self.declare_parameter('goal_tolerance', 0.40)
        self.declare_parameter('goal_1_x', 3.0)
        self.declare_parameter('goal_1_y', 0.0)
        self.declare_parameter('goal_2_x', 3.0)
        self.declare_parameter('goal_2_y', -1.20)
        self.declare_parameter('goal_3_x', 0.0)
        self.declare_parameter('goal_3_y', -1.20)
        self.declare_parameter('turn_side', 'RIGHT')
        self.declare_parameter('omega_open_angle_deg', 35.0)
        self.declare_parameter('omega_r1', 0.85)
        self.declare_parameter('omega_clearance', 0.25)
        self.declare_parameter('omega_lead_in', 0.40)

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
        self.turn_in_place_threshold_deg = float(p('turn_in_place_threshold_deg').value)
        self.turn_in_place_resume_deg    = float(p('turn_in_place_resume_deg').value)
        self.row_spacing              = p('row_spacing').value
        self.ema_alpha                = p('ema_alpha').value
        self.enable_uturn             = p('enable_uturn').value
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

        # Mission goals & Omega Turn parameters
        self.enable_mission_goals     = p('enable_mission_goals').value
        self.enable_global_plan_assist= p('enable_global_plan_assist').value
        self.field_length             = p('field_length').value
        self.headland_area_length     = p('headland_area_length').value
        self.goal_tolerance           = p('goal_tolerance').value
        self.goal_1_x                 = p('goal_1_x').value
        self.goal_1_y                 = p('goal_1_y').value
        self.goal_2_x                 = p('goal_2_x').value
        self.goal_2_y                 = p('goal_2_y').value
        self.goal_3_x                 = p('goal_3_x').value
        self.goal_3_y                 = p('goal_3_y').value
        self.turn_side                = p('turn_side').value
        self.omega_open_angle_deg     = p('omega_open_angle_deg').value
        self.omega_r1                 = p('omega_r1').value
        self.omega_clearance          = p('omega_clearance').value
        self.omega_lead_in            = p('omega_lead_in').value
        self.current_lane_idx         = 1

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
                target_name = os.path.basename(self.model_path) if self.model_path else 'crop_row_cnn_best_final_int8.onnx'
                candidate_names = [target_name, 'crop_row_cnn_best_final_int8.onnx', 'crop_row_cnn_best_final.onnx']
                
                # 1. Try my_robot_controller share directory
                try:
                    from ament_index_python.packages import get_package_share_directory
                    share_dir = get_package_share_directory('my_robot_controller')
                    for name in candidate_names:
                        candidate = os.path.join(share_dir, 'models', name)
                        if os.path.exists(candidate):
                            self.model_path = candidate
                            break
                except Exception:
                    pass

                # 2. Try luanvan_control share directory
                if not self.model_path or not os.path.exists(self.model_path):
                    try:
                        from ament_index_python.packages import get_package_share_directory
                        share_dir = get_package_share_directory('luanvan_control')
                        for name in candidate_names:
                            candidate = os.path.join(share_dir, 'models', name)
                            if os.path.exists(candidate):
                                self.model_path = candidate
                                break
                    except Exception:
                        pass

                # 3. Fallback to relative path from source files
                if not self.model_path or not os.path.exists(self.model_path):
                    current_dir = os.path.dirname(os.path.abspath(__file__))
                    for name in candidate_names:
                        candidate = os.path.abspath(os.path.join(current_dir, '..', 'models', name))
                        if os.path.exists(candidate):
                            self.model_path = candidate
                            break

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
            low_confidence_threshold=self.low_confidence_threshold,
            consecutive_frames=self.low_conf_frames_threshold
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
        self.eor_low_conf_frames    = 0
        self.inside_row             = False
        self.has_seen_row           = False
        self.is_adjusting_heading   = False
        self.heading_adjust_target_yaw = 0.0
        self.heading_adjust_dir     = 0.0
        self.heading_adjust_start_time = 0.0
        self.heading_adjust_cooldown_until = 0.0
        self._aligned_frame_count   = 0
        self.current_lane_y         = 0.0

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

        # ── IMU U-Turn & Slip Monitoring Variables ────────────────────
        self.uturn_start_imu_yaw      = 0.0
        self.uturn_target_yaw         = 0.0
        self.uturn_accum_imu_yaw      = 0.0
        self.uturn_accum_wheel_yaw    = 0.0
        self._prev_imu_yaw_for_turn   = None
        self._prev_wheel_yaw_for_turn = None
        self._slip_warning_logged     = False
        self.is_trimming_uturn_heading= False
        self.uturn_trim_start_time    = 0.0

        # ── Image logging ─────────────────────────────────────────────
        self.last_img_save_time = 0.0
        self.output_dir = os.path.join(os.path.expanduser('~'), 'ros2_debug_imgs')

        # ── Publishers & Subscribers ──────────────────────────────────
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.gps_pub     = self.create_publisher(NavSatFix, '/localization/gps', 10)
        self.plan_pub    = self.create_publisher(RosPath, '/plan', 10)
        self.plan_timer  = self.create_timer(1.0, self.publish_mission_path)
        self.heading_adjust_timer = self.create_timer(0.05, self._heading_adjust_timer_callback)

        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, 10)

        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10)
        # Fallback subscription for raw odom
        if self.odom_topic not in ('/odom', 'odom'):
            self.odom_fallback_sub = self.create_subscription(
                Odometry, '/odom', self.odom_callback, 10)

        # Wheel odom subscriber (dùng so sánh góc quay encoder và IMU để phát hiện trượt bánh)
        self.wheel_odom_sub = self.create_subscription(
            Odometry, '/wheel/odom', self.wheel_odom_callback, 10
        )

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)

        self.gps_sub = self.create_subscription(
            NavSatFix, self.gps_topic, self.gps_callback, 10)

        self.imu_sub = self.create_subscription(
            Imu, self.imu_topic, self.imu_callback, 10)
        # Fallback subscription for alternative imu topic
        if self.imu_topic not in ('/imu', 'imu'):
            self.imu_fallback_sub = self.create_subscription(
                Imu, '/imu', self.imu_callback, 10)

        # ── Remote CNN Offloading (Laptop gánh phần suy luận ONNX) ───────
        self._remote_cnn_data = None
        self._remote_cnn_time = 0.0
        self.crop_row_sub = self.create_subscription(
            Float32MultiArray, '/crop_row/detection', self.remote_cnn_callback, 10
        )

        self.get_logger().info(
            f"🚀 [AI CNN Driver] Sẵn sàng tự hành bám luống (Vận tốc: {self.linear_speed:.2f} m/s | Model INT8)"
        )

        # ── State for Periodic 1Hz Status Logging ─────────────────────
        self._last_image_time = 0.0
        self._waiting_camera_notified = False
        self._camera_lost_notified = False
        self._latest_confidence = 0.0
        self._latest_steer_deg = 0.0
        self._latest_twist = Twist()
        self._latest_gps_info = {}
        self._latest_inf_ms = 0.0
        self.status_timer = self.create_timer(1.0, self.status_timer_callback)

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

    def publish_mission_path(self):
        """Xuất bản toàn bộ lộ trình nhiệm vụ 2 luống + Omega Turn lên /plan cho RViz hiển thị trực quan."""
        try:
            full_mission = OmegaTurnPlanner.generate_full_mission_path(
                field_length=self.field_length,
                row_spacing=self.row_spacing,
                turn_side=self.turn_side,
                open_angle_deg=self.omega_open_angle_deg,
                r1=self.omega_r1,
                clearance_dist=self.omega_clearance,
                lead_in_dist=self.omega_lead_in,
            )
            now_msg = self.get_clock().now().to_msg()
            ros_path = OmegaTurnPlanner.to_ros_path(full_mission.waypoints, frame_id='odom', stamp=now_msg)
            self.plan_pub.publish(ros_path)
        except Exception as e:
            self.get_logger().warn(f"Failed to publish mission path: {e}", throttle_duration_sec=5.0)

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
            twist.linear.x = -self.linear_speed  # Back up slowly
            twist.angular.z = 0.0
            finished = False
        else:
            finished = True
        return twist, finished

    # ── GPS, IMU & Wheel Odom callbacks ───────────────────────────────
    def wheel_odom_callback(self, msg: Odometry):
        """Theo dõi góc quay encoder bánh xe để so sánh với IMU phát hiện trượt bánh."""
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        wheel_yaw = math.atan2(siny, cosy)

        is_uturn_active = (
            self.fsm.get_state() == FSMState.UTURN_PLANNING or
            (self.fsm.get_state() == FSMState.PATH_FOLLOWING and self.fsm.state_before_planning == FSMState.UTURN_PLANNING) or
            self.is_trimming_uturn_heading
        )
        if is_uturn_active:
            if self._prev_wheel_yaw_for_turn is not None:
                dyaw_w = math.atan2(
                    math.sin(wheel_yaw - self._prev_wheel_yaw_for_turn),
                    math.cos(wheel_yaw - self._prev_wheel_yaw_for_turn)
                )
                self.uturn_accum_wheel_yaw += abs(dyaw_w)
            self._prev_wheel_yaw_for_turn = wheel_yaw
        else:
            self._prev_wheel_yaw_for_turn = None

    def imu_callback(self, msg: Imu):
        self.localization_manager.update_imu(msg)
        current_imu_yaw = self.localization_manager.imu_yaw

        # 1. Tích lũy góc quay thực tế từ IMU khi xe đang quay đầu
        is_uturn_active = (
            self.fsm.get_state() == FSMState.UTURN_PLANNING or
            (self.fsm.get_state() == FSMState.PATH_FOLLOWING and self.fsm.state_before_planning == FSMState.UTURN_PLANNING) or
            self.is_trimming_uturn_heading
        )
        if is_uturn_active:
            if self._prev_imu_yaw_for_turn is not None:
                dyaw_imu = math.atan2(
                    math.sin(current_imu_yaw - self._prev_imu_yaw_for_turn),
                    math.cos(current_imu_yaw - self._prev_imu_yaw_for_turn)
                )
                self.uturn_accum_imu_yaw += abs(dyaw_imu)
            self._prev_imu_yaw_for_turn = current_imu_yaw

            # 2. Báo cáo phát hiện trượt bánh (Slip Detection)
            if self.uturn_accum_wheel_yaw > 0.10:
                slip_deg = math.degrees(self.uturn_accum_wheel_yaw - self.uturn_accum_imu_yaw)
                if slip_deg > 8.0 and not self._slip_warning_logged:
                    self.get_logger().warn(
                        f"⚠️ [CẢNH BÁO TRƯỢT BÁNH] Bánh xe quay {math.degrees(self.uturn_accum_wheel_yaw):.1f}° "
                        f"nhưng IMU thực tế chỉ quay {math.degrees(self.uturn_accum_imu_yaw):.1f}° (Trượt {slip_deg:.1f}° trên cỏ/đất)! "
                        f"Hệ thống đang điều khiển bù góc theo IMU để đảm bảo quay đúng 180°!"
                    )
                    if self.enable_file_logging and self.telemetry_logger:
                        self.telemetry_logger.log_event(
                            "WHEEL_SLIP_DETECTED",
                            f"Bánh quay {math.degrees(self.uturn_accum_wheel_yaw):.1f}°, IMU quay {math.degrees(self.uturn_accum_imu_yaw):.1f}°, Trượt {slip_deg:.1f}°"
                        )
                    self._slip_warning_logged = True

            # 3. Nắn chỉnh hướng đầu xe khép vòng IMU 50Hz (Heading Auto-Trim)
            if self.is_trimming_uturn_heading:
                now_sec = self.get_clock().now().nanoseconds / 1e9
                yaw_err = math.atan2(
                    math.sin(self.uturn_target_yaw - current_imu_yaw),
                    math.cos(self.uturn_target_yaw - current_imu_yaw)
                )
                trim_threshold = math.radians(1.5)  # Chuẩn sai số < 1.5 độ
                timed_out = (now_sec - self.uturn_trim_start_time) > 4.5

                if abs(yaw_err) <= trim_threshold or timed_out:
                    self.is_trimming_uturn_heading = False
                    self.StopRobot()
                    start_deg = math.degrees(self.uturn_start_imu_yaw)
                    now_deg = math.degrees(current_imu_yaw)
                    accum_deg = math.degrees(self.uturn_accum_imu_yaw)
                    status_note = "THÀNH CÔNG HOÀN HẢO" if abs(yaw_err) <= trim_threshold else "TIMEOUT AN TOÀN"
                    self.get_logger().info(
                        f"✅ [IMU XÁC NHẬN QUAY ĐẦU - {status_note}] Đã quay đúng 180° và căn thẳng vào Luống 2! "
                        f"(Start: {start_deg:+.1f}°, Hiện tại: {now_deg:+.1f}°, IMU tổng xoay: {accum_deg:.1f}°, Sai lệch trục: {math.degrees(yaw_err):+.2f}°). "
                        f"Bàn giao quyền điều khiển cho AI CNN!"
                    )
                    if self.enable_file_logging and self.telemetry_logger:
                        self.telemetry_logger.log_event(
                            "UTURN_SUCCESS_IMU",
                            f"Quay đầu 180° thành công: Start={start_deg:.1f}°, Current={now_deg:.1f}°, Total={accum_deg:.1f}°, Err={math.degrees(yaw_err):.2f}°"
                        )
                    self.transition_after_path(self.get_clock().now())
                else:
                    # Xoay nhẹ đầu xe về hướng target Luống 2
                    twist = Twist()
                    twist.linear.x = 0.0
                    turn_dir = 1.0 if yaw_err > 0 else -1.0
                    turn_speed = max(0.18, min(float(self.turn_angular_speed), 0.35))
                    twist.angular.z = turn_dir * turn_speed
                    self.cmd_vel_pub.publish(twist)
        else:
            self._prev_imu_yaw_for_turn = None

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

        # ── IMU/Odom Watchdog an toàn cho căn chỉnh CNN (6.0s fallback safety) ──
        if self.is_adjusting_heading and self.fsm.get_state() == FSMState.TRACKING:
            now_sec = self.get_clock().now().nanoseconds / 1e9
            timed_out = (now_sec - self.heading_adjust_start_time) > 6.0
            
            if timed_out:
                self.is_adjusting_heading = False
                self.heading_adjust_cooldown_until = now_sec + 1.0
                self._aligned_frame_count = 0
                twist = Twist()
                twist.linear.x = self.linear_speed
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                log_msg = f"⚠️ [ĐIỀU HƯỚNG CNN - WATCHDOG] Căn chỉnh quá 6.0s -> Khôi phục chạy thẳng {self.linear_speed:.2f} m/s an toàn!"
                self.get_logger().warn(log_msg)
                if self.enable_file_logging and self.telemetry_logger:
                    self.telemetry_logger.log_event("CNN_HEADING_TIMEOUT", log_msg)

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

    def _heading_adjust_timer_callback(self):
        """
        Timer 20Hz: Điều khiển vận tốc góc xoay căn chỉnh hướng êm ái, từ từ nhưng đủ lực.
        - Tăng tốc mềm (ramp-up trong 0.25s từ 0 lên turn_angular_speed ~0.35 rad/s).
        - Khi góc đã tiệm cận chuẩn (< 2.0°), giảm nhẹ tốc độ xuống 0.22 rad/s để chống văng lố.
        - An toàn: Nếu quá 6.0s chưa thẳng hàng, tự động khôi phục chạy tiếp để tránh kẹt robot.
        """
        if not self.is_adjusting_heading or self.fsm.get_state() != FSMState.TRACKING:
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9
        elapsed = now_sec - self.heading_adjust_start_time

        # Watchdog an toàn: quá 6s tự động thoát
        if elapsed > 6.0:
            self.is_adjusting_heading = False
            self.heading_adjust_cooldown_until = now_sec + 1.0
            self._aligned_frame_count = 0
            twist = Twist()
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            self.get_logger().warn("⚠️ [ĐIỀU HƯỚNG CNN] Quá 6s căn chỉnh -> Tự động khôi phục chạy thẳng!")
            return

        # Tăng tốc mềm với mô-men khởi động tức thì (tối thiểu 60% để thắng ma sát tĩnh trên cỏ)
        ramp = min(1.0, 0.60 + 0.40 * (elapsed / 0.20))
        base_w = float(getattr(self, 'turn_angular_speed', 0.40))
        
        # Đảm bảo sàn tốc độ >= 0.26 rad/s để 4 bánh không bị khựng/stall trên cỏ
        angle_err = abs(self.smoothed_angle_deg)
        if angle_err < 0.6:
            target_w = 0.26
        elif angle_err < 1.0:
            target_w = 0.32
        else:
            target_w = base_w

        current_w = self.heading_adjust_dir * target_w * ramp

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = current_w
        self.cmd_vel_pub.publish(twist)

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
            self.is_adjusting_heading = False
            self._aligned_frame_count = 0
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

    # ── Remote CNN Callback ───────────────────────────────────────────
    def remote_cnn_callback(self, msg: Float32MultiArray):
        """Nhận kết quả AI tính sẵn từ Laptop (30-60 FPS, độ trễ thấp)."""
        if len(msg.data) >= 4:
            self._remote_cnn_data = {
                'heading_error': float(msg.data[0]),
                'lane_offset': float(msg.data[1]),
                'lane_center': float(msg.data[2]),
                'confidence': float(msg.data[3]),
                'latency_ms': float(msg.data[4]) if len(msg.data) > 4 else 0.0
            }
            self._remote_cnn_time = self.get_clock().now().nanoseconds / 1e9

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
            if getattr(self, '_waiting_camera_notified', False):
                self.get_logger().info("✅ Đã nhận tín hiệu Camera! Bắt đầu chuỗi tự hành AI.")
                self._waiting_camera_notified = False
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
        now_sec = now_ns / 1e9
        is_remote_cnn = (self._remote_cnn_data is not None) and ((now_sec - self._remote_cnn_time) < 0.6)
        external_cnn = self._remote_cnn_data if is_remote_cnn else None

        t_infer_start = time.time()
        perception = self.perception_manager.process_sensors(
            cv_image=bgr_image,
            distance_traveled=self.distance_traveled,
            rx=self.current_x,
            ry=self.current_y,
            ryaw=self.current_yaw,
            max_angle_deg=self.max_steering_angle_deg,
            inside_row=self.inside_row,
            external_cnn=external_cnn
        )
        self._last_inference_ms = (time.time() - t_infer_start) * 1000.0
        
        confidence = perception["confidence"]
        obstacle_detected = perception["obstacle_detected"]
        end_of_row = perception["end_of_row_detected"]
        raw_angle = perception["heading_error"]
        lane_center = perception["lane_center"]
        lane_offset = perception["lane_offset"]
        left_side_dist = perception.get("left_side_dist", float('inf'))
        right_side_dist = perception.get("right_side_dist", float('inf'))

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

        now_sec = now.nanoseconds / 1e9

        # Save diagnostic frame every 3 seconds (sim time, if enabled)
        if self.save_debug_imgs:
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

        # Khi Camera/CNN nhìn thấy hàng (confidence >= low_confidence_threshold):
        # Bám luống ngay lập tức dù đặt xe ở gần hay xa, thẳng hàng hay lệch xéo.
        # Không ràng buộc cứng mét chạy hay điều kiện thành tường.
        if confidence >= self.low_confidence_threshold:
            self.has_seen_row = True
            if not self.inside_row:
                self.inside_row = True
                if self.row_start_x is None:
                    self.row_start_x = self.current_x
                    self.current_lane_y = self.current_y
                self.row_completed = False
                self.get_logger().info(f"🌾 [BÁM HÀNG] Nhận diện luống thành công tại x={self.row_start_x:.2f}m, y={self.current_lane_y:.2f}m (conf={confidence:.2f}). Bắt đầu dẫn hướng vào tim hàng!")
                if self.enable_file_logging and self.telemetry_logger:
                    self.telemetry_logger.log_event("ENTER_ROW", f"Bám luống tại x={self.row_start_x:.2f}m, y={self.current_lane_y:.2f}m (conf={confidence:.2f})")

        # ── IDLE ──────────────────────────────────────────────────────
        if current_state == FSMState.IDLE:
            self.StopRobot()
            if confidence >= self.low_confidence_threshold:
                self.transition_to_state(FSMState.TRACKING, now)
            return

        # ── TRACKING ──────────────────────────────────────────────────
        elif current_state == FSMState.TRACKING:
            # 1. An toàn mũi xe (Bumper collision guard):
            # Nếu có vật cản cứng nằm ngay sát cản trước (< 0.16m), tạm dừng tiến để bảo vệ khung xe.
            # Không tự ý chuyển state hay rẽ lung tung vào cây, chờ đường thoáng xe tự bám tiếp.
            front_min_dist = perception.get("front_min_dist", float('inf'))
            if front_min_dist < 0.16:
                self.StopRobot()
                self.get_logger().warn_throttle(
                    1.0, f"⚠️ Mũi xe sát vật cản ({front_min_dist:.2f}m < 0.16m)! Tạm dừng chờ thoáng..."
                )
                return

            # 2. End-of-Row exit distance handling (Chỉ chạy khi enable_uturn=True)
            if getattr(self, 'eor_detected', False):
                if not self.enable_uturn:
                    self.eor_detected = False
                    self.StopRobot()
                    self.transition_to_state(FSMState.IDLE, now)
                    return
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
                    lane_center = self.current_lane_y
                    dir_x = 1.0 if math.cos(self.current_yaw) >= 0 else -1.0
                    target_yaw = 0.0 if dir_x > 0 else (math.pi if self.current_yaw >= 0 else -math.pi)
                    yaw_err = math.atan2(math.sin(target_yaw - self.current_yaw), math.cos(target_yaw - self.current_yaw))
                    lat_correction = -1.5 * (self.current_y - lane_center) * dir_x
                    twist.linear.x = self.linear_speed
                    twist.angular.z = float(np.clip(lat_correction + 0.8 * yaw_err, -0.30, 0.30))
                    self.cmd_vel_pub.publish(twist)
                    return

            # Xử lý góc lái: Khi camera đủ tin cậy HOẶC khi LiDAR phát hiện xe áp sát thành hàng (< 0.32m)
            # Tuyệt đối KHÔNG triệt tiêu góc lái về 0 khi xe đang ở vị trí sát sườn nguy hiểm!
            failed = (confidence < self.low_confidence_threshold)
            near_side_wall = (left_side_dist < 0.32 or right_side_dist < 0.32)
            
            if not failed or near_side_wall:
                # Nếu đang áp sát thành hàng hoặc đang xoay căn chỉnh, tăng độ nhạy phản xạ (alpha >= 0.65) để phản xạ tức thì
                effective_alpha = max(self.ema_alpha, 0.65) if (near_side_wall or self.is_adjusting_heading) else self.ema_alpha
                self.smoothed_angle_deg = (
                    effective_alpha * raw_angle
                    + (1.0 - effective_alpha) * self.smoothed_angle_deg
                )
            elif self.enable_global_plan_assist and self.inside_row:
                # ── HỖ TRỢ SONG SONG TỪ GLOBAL PLAN (FALLBACK KHI CÂY THƯA / LÁ CHE / THÙNG HỞ) ──
                # Khi camera bị che khuất hoặc thiếu thùng carton (confidence < 0.30),
                # sử dụng tọa độ EKF/Odometry chiếu lên tim luống hiện tại để giữ xe chạy thẳng:
                target_y = 0.0 if self.current_lane_idx == 1 else self.current_lane_y
                target_yaw = 0.0 if self.current_lane_idx == 1 else (math.pi if self.current_yaw >= 0 else -math.pi)

                # Sai số vị trí ngang (cross-track error) và sai số góc hướng
                lat_err = self.current_y - target_y
                dir_factor = 1.0 if self.current_lane_idx == 1 else -1.0
                yaw_err = math.atan2(math.sin(target_yaw - self.current_yaw), math.cos(target_yaw - self.current_yaw))

                # Tính góc bẻ lái hỗ trợ đưa xe về tim luống
                assist_steer_deg = float(np.clip(-12.0 * lat_err * dir_factor + math.degrees(yaw_err) * 0.4, -10.0, 10.0))
                self.smoothed_angle_deg = (
                    0.25 * assist_steer_deg + 0.75 * self.smoothed_angle_deg
                )
                if now_sec - getattr(self, '_last_global_assist_log_time', 0.0) >= 2.0:
                    self._last_global_assist_log_time = now_sec
                    self.get_logger().info(
                        f"🌾 [GLOBAL PLAN ASSIST] Conf thấp ({confidence*100:.1f}%) -> Hỗ trợ giữ tim Luống {self.current_lane_idx} (y={target_y:.2f}m, dy={lat_err:.2f}m, steer={self.smoothed_angle_deg:+.1f}°)"
                    )
            else:
                # Chỉ triệt tiêu góc lái về 0 khi thực sự ở bãi đất trống ngoài luống
                self.smoothed_angle_deg *= 0.90

            # Cập nhật controller nội bộ để lưu vết sai số
            now_sec = now.nanoseconds / 1e9
            if hasattr(self, '_prev_image_time'):
                dt_actual = now_sec - self._prev_image_time
            else:
                dt_actual = 0.067
            self._prev_image_time = now_sec
            dt_actual = np.clip(dt_actual, 0.001, 1.0)
            
            _lin_smc, _ang_smc = self.StartTracking(dt_actual)

            # ── LOGIC ĐIỀU HƯỚNG CNN: DỪNG XOAY KHI GÓC <= 0.3° ──
            # 1. Khi đang chạy bình thường (|góc| <= 0.3°): Xe chạy tiến 0.10 m/s, tự bù lái mềm để giữ luống.
            # 2. Khi góc lệch > 0.3°: Dừng tiến, xoay từ từ tại chỗ với lực vừa đủ (~0.28 rad/s, hãm về 0.18 rad/s).
            # 3. Khi góc giảm về <= 0.3°: Ngừng xoay, tiếp tục chạy thẳng và giãn cách an toàn 1.2s.
            stop_threshold = float(getattr(self, 'turn_in_place_threshold_deg', 0.30))
            resume_threshold = float(getattr(self, 'turn_in_place_resume_deg', 0.30))

            if self.is_adjusting_heading:
                # Kiểm tra đã thẳng hàng chưa (<= resume_threshold = 0.3°)
                if abs(self.smoothed_angle_deg) <= resume_threshold:
                    self.is_adjusting_heading = False
                    self._aligned_frame_count = 0
                    self.heading_adjust_cooldown_until = now_sec + 1.2
                    self.forward_resume_start_time = now_sec

                    # Hãm phanh êm dịu triệt tiêu quán tính quay trước khi tiến:
                    twist = Twist()
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.cmd_vel_pub.publish(twist)

                    self.get_logger().info(
                        f"✅ [ĐIỀU HƯỚNG CNN] ĐÃ THẲNG HÀNG "
                        f"(|góc|={abs(self.smoothed_angle_deg):.2f}° <= {resume_threshold:.2f}°)! "
                        f"Hãm xoay êm dịu và tăng tốc tiến mượt mà {self.linear_speed:.2f} m/s."
                    )
                    if self.enable_file_logging and self.telemetry_logger:
                        self.telemetry_logger.log_event(
                            "CNN_HEADING_CONFIRMED_ALIGNED",
                            f"Thẳng hàng |góc|={abs(self.smoothed_angle_deg):.2f}°. Chạy tiếp {self.linear_speed:.2f} m/s"
                        )
                else:
                    self._aligned_frame_count = 0
                    # Khóa hướng xoay chống đảo chiều liên tục: chỉ đổi hướng khi góc đổi dấu rõ rệt (> 0.5°)
                    if self.smoothed_angle_deg > 0.5 and self.heading_adjust_dir > 0:
                        self.heading_adjust_dir = -1.0
                    elif self.smoothed_angle_deg < -0.5 and self.heading_adjust_dir < 0:
                        self.heading_adjust_dir = 1.0

            else:
                can_trigger = (now_sec >= getattr(self, 'heading_adjust_cooldown_until', 0.0))
                if can_trigger and abs(self.smoothed_angle_deg) > stop_threshold:
                    self.is_adjusting_heading = True
                    self.heading_adjust_start_time = now_sec
                    self.heading_adjust_dir = -1.0 if self.smoothed_angle_deg > 0 else 1.0
                    self._aligned_frame_count = 0
                    self.get_logger().info(
                        f"🌾 [ĐIỀU HƯỚNG CNN] CNN trả góc lệch {self.smoothed_angle_deg:+.2f}° > {stop_threshold:.2f}°. "
                        f"Dừng tiến, xoay từ từ tại chỗ ({self.turn_angular_speed:.2f} rad/s) chờ thẳng hàng (<= {resume_threshold:.2f}°)..."
                    )
                    if self.enable_file_logging and self.telemetry_logger:
                        self.telemetry_logger.log_event(
                            "CNN_HEADING_STOP_ADJUST",
                            f"Lệch {self.smoothed_angle_deg:+.2f}° > {stop_threshold:.2f}°. Dừng tiến xoay căn chỉnh."
                        )

            if self.is_adjusting_heading:
                lin_speed = 0.0
                ang_vel = 0.0  # Vận tốc góc xoay từ từ do timer 20Hz kiểm soát độc lập
            else:
                # Tăng tốc tiến mềm (0.35s ramp-up) sau khi căn chỉnh xong để chống giật/trượt bánh:
                time_since_resume = now_sec - getattr(self, 'forward_resume_start_time', 0.0)
                fwd_ramp = min(1.0, time_since_resume / 0.35) if time_since_resume < 0.35 else 1.0
                lin_speed = self.linear_speed * fwd_ramp
                # Khi đang chạy thẳng, bù lái nhẹ nhàng liên tục để giữ luống ổn định không khựng xe:
                ang_vel = float(np.clip(-0.035 * self.smoothed_angle_deg, -0.10, 0.10))

            twist.linear.x = lin_speed
            twist.angular.z = ang_vel

            # ── Pure Perception & Mission Goals End-Of-Row Check ─────────────────────
            # 1. Điều kiện nhận biết hết hàng qua camera: 5 frame liên tiếp confidence < 30%
            if confidence < 0.30:
                self.eor_low_conf_frames += 1
            else:
                self.eor_low_conf_frames = 0
            trigger_confidence = (self.eor_low_conf_frames >= 5)

            # 2. Điều phối theo 4 Điểm Mốc Nhiệm Vụ (Mission Goals):
            if self.enable_mission_goals and warmup_done:
                if self.current_lane_idx == 1:
                    # Đang chạy Luống 1: kiểm tra đã đến cuối Luống 1 chưa (Goal 1: x = field_length)
                    dist_to_g1 = math.hypot(self.current_x - self.goal_1_x, self.current_y - self.goal_1_y)
                    reached_g1 = (dist_to_g1 <= self.goal_tolerance) or (self.current_x >= self.goal_1_x)

                    if reached_g1 or (trigger_confidence and self.current_x >= self.min_row_length) or (end_of_row and self.current_x >= self.min_row_length):
                        self.get_logger().info(
                            f"🎯 [MỐC GOAL 1] Đã đến cuối Luống 1 tại x={self.current_x:.2f}m (dist={self.distance_traveled:.2f}m)! "
                            f"Kích hoạt quay đầu Omega Turn chuyển sang Luống 2..."
                        )
                        if self.enable_file_logging and self.telemetry_logger:
                            self.telemetry_logger.log_event("GOAL_1_REACHED", f"Cuối Luống 1 tại x={self.current_x:.2f}m, dist={self.distance_traveled:.2f}m")
                        self.eor_detected = False
                        self.current_lane_idx = 2
                        self.low_confidence_counter = 0
                        self.smoothed_angle_deg  = 0.0
                        self.distance_traveled   = 0.0
                        self.inside_row          = False
                        self.row_start_x         = None
                        self.row_completed       = False
                        self.accumulated_turn_angle = 0.0
                        self.StopRobot()
                        self.transition_to_state(FSMState.UTURN_PLANNING, now)
                        return

                elif self.current_lane_idx == 2:
                    # Đang chạy Luống 2: chạy ngược chiều từ x = field_length về Goal 3 (x = 0)
                    dist_to_g3 = math.hypot(self.current_x - self.goal_3_x, self.current_y - self.goal_3_y)
                    reached_g3 = (dist_to_g3 <= self.goal_tolerance) or (self.current_x <= (self.goal_3_x + 0.15))

                    if (reached_g3 and self.distance_traveled >= self.min_row_length) or (trigger_confidence and self.distance_traveled >= self.min_row_length):
                        self.StopRobot()
                        self.get_logger().info(
                            f"🏆 [HOÀN THÀNH NHIỆM VỤ] Đã chạy xong Luống 2 về đích Goal 3 tại x={self.current_x:.2f}m, y={self.current_y:.2f}m! "
                            f"Dừng xe an toàn tuyệt đối."
                        )
                        if self.enable_file_logging and self.telemetry_logger:
                            self.telemetry_logger.log_event("MISSION_COMPLETE", f"Về đích Goal 3 x={self.current_x:.2f}m, y={self.current_y:.2f}m")
                        self.transition_to_state(FSMState.IDLE, now)
                        return

            elif warmup_done and getattr(self, 'has_seen_row', False):
                # Dự phòng nếu không bật mission goals (thuần nhận thức cảm biến)
                if trigger_confidence or end_of_row:
                    if not self.enable_uturn:
                        self.StopRobot()
                        self.has_seen_row = False
                        self.inside_row = False
                        self.transition_to_state(FSMState.IDLE, now)
                        self.get_logger().info(
                            f"🛑 [HẾT HÀNG] Đã chạy tới cuối luống! Dừng xe an toàn tại x={self.current_x:.2f}m."
                        )
                        return
                    else:
                        self.row_completed = True
                        self.eor_detected = True
                        self.eor_trigger_x = self.current_x
                        return

        # ── REACTIVE_AVOID ────────────────────────────────────────────
        elif current_state == FSMState.REACTIVE_AVOID:
            self.StopRobot()
            
            # Check if obstacle has cleared
            if not obstacle_detected:
                self.get_logger().info("Obstacle cleared! Resuming TRACKING...")
                if self.enable_file_logging and self.telemetry_logger:
                    self.telemetry_logger.log_event("OBSTACLE_CLEAR", "Vật cản đã rời khỏi tầm quét. Quay lại TRACKING")
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
            # Xác định chiều chuyển luống và tọa độ luống đích
            target_y = -abs(self.row_spacing) if self.turn_side.upper() == 'RIGHT' else abs(self.row_spacing)
            self.current_lane_y = target_y

            # Ghi nhận mốc xuất phát quay đầu từ cảm biến IMU
            self.uturn_start_imu_yaw = self.localization_manager.imu_yaw
            # Luống 2 ngược chiều Luống 1 (+180°):
            self.uturn_target_yaw = math.atan2(
                math.sin(self.uturn_start_imu_yaw + math.pi),
                math.cos(self.uturn_start_imu_yaw + math.pi)
            )
            self.uturn_accum_imu_yaw = 0.0
            self.uturn_accum_wheel_yaw = 0.0
            self._prev_imu_yaw_for_turn = self.uturn_start_imu_yaw
            self._prev_wheel_yaw_for_turn = None
            self._slip_warning_logged = False
            self.is_trimming_uturn_heading = False

            self.get_logger().info(
                f"🔄 [OMEGA TURN - IMU QUAY ĐẦU] Bắt đầu quay đầu 180° từ IMU Yaw={math.degrees(self.uturn_start_imu_yaw):+.1f}° "
                f"-> Hướng mục tiêu Luống 2: {math.degrees(self.uturn_target_yaw):+.1f}° "
                f"(turn_side={self.turn_side}, spacing={self.row_spacing:.2f}m, R1={self.omega_r1:.2f}m, alpha={self.omega_open_angle_deg}°)..."
            )
            
            # Reset U-turn turn tracking variables
            self.uturn_start_yaw = self.current_yaw
            self.accumulated_turn_angle = 0.0
            self._prev_yaw_for_turn = self.current_yaw

            # 1. Sinh quỹ đạo Omega Turn giải tích chống trượt bánh vi sai
            waypoints_obj = OmegaTurnPlanner.generate_path(
                start_x=self.current_x,
                start_y=self.current_y,
                start_yaw=self.current_yaw,
                row_spacing=self.row_spacing,
                turn_side=self.turn_side,
                open_angle_deg=self.omega_open_angle_deg,
                r1=self.omega_r1,
                clearance_dist=self.omega_clearance,
                lead_in_dist=self.omega_lead_in,
            )
            waypoints = waypoints_obj.waypoints if waypoints_obj else None

            if not waypoints:
                self.get_logger().warn("Omega Turn path failed! Fallback semicircular...")
                dir_x = 1.0 if math.cos(self.current_yaw) >= 0 else -1.0
                goal_x = self.current_x - dir_x * 0.50
                waypoints_obj = self.generate_backup_uturn_path(goal_x, target_y)
                waypoints = waypoints_obj.waypoints if hasattr(waypoints_obj, 'waypoints') else waypoints_obj

            if waypoints:
                self.pure_pursuit_controller.set_path(waypoints)
                self.last_visited_lane = 'lane_lower' if target_y < 0 else 'lane_upper'
                self.transition_to_state(FSMState.PATH_FOLLOWING, now)
                self.get_logger().info(f"🚀 Bắt đầu bám quỹ đạo Omega Turn ({len(waypoints)} điểm, R1={self.omega_r1:.2f}m, mở góc={self.omega_open_angle_deg}°)...")
            else:
                self.get_logger().error("UTurn path generation failed! Transitioning to RECOVERY...")
                self.transition_to_state(FSMState.RECOVERY, now)

        # ── PATH_FOLLOWING ────────────────────────────────────────────
        elif current_state == FSMState.PATH_FOLLOWING:
            # Use decoupled PurePursuitController wrapper (Task 8)
            lin_vel, ang_vel, finished, path_idx = self.FollowPath()
            
            is_uturn = (self.fsm.state_before_planning == FSMState.UTURN_PLANNING)
            
            if is_uturn:
                current_imu_yaw = self.localization_manager.imu_yaw
                turn_angle_deg = np.rad2deg(self.uturn_accum_imu_yaw)
                is_turned_around = (turn_angle_deg >= 140.0)

                # Kiểm tra sai số góc hướng so với trục chuẩn Luống 2 (target_yaw)
                yaw_err_to_row2 = math.atan2(
                    math.sin(self.uturn_target_yaw - current_imu_yaw),
                    math.cos(self.uturn_target_yaw - current_imu_yaw)
                )
                yaw_err_deg = math.degrees(yaw_err_to_row2)

                if finished or (confidence >= self.high_confidence_threshold and is_turned_around):
                    # Nếu xe đã xong cung Omega nhưng đầu xe còn lệch trục luống (> 2.0 độ):
                    if abs(yaw_err_deg) > 2.0:
                        if not self.is_trimming_uturn_heading:
                            self.is_trimming_uturn_heading = True
                            self.uturn_trim_start_time = now.nanoseconds / 1e9
                            self.StopRobot()
                            self.get_logger().info(
                                f"🔄 [NẮN CHỈNH HƯỚNG LUỐNG 2] Cung Omega hoàn tất nhưng đầu xe còn lệch {yaw_err_deg:+.1f}° so với trục luống -> IMU 50Hz tự động nắn chỉnh chuẩn 180°..."
                            )
                        return
                    else:
                        # Đầu xe đã thẳng đẹp vào Luống 2 (lệch <= 2.0 độ):
                        self.StopRobot()
                        start_deg = math.degrees(self.uturn_start_imu_yaw)
                        now_deg = math.degrees(current_imu_yaw)
                        accum_deg = math.degrees(self.uturn_accum_imu_yaw)
                        self.get_logger().info(
                            f"✅ [IMU XÁC NHẬN QUAY ĐẦU THÀNH CÔNG] Đã quay đúng 180° và căn thẳng vào Luống 2! "
                            f"(Start: {start_deg:+.1f}°, Hiện tại: {now_deg:+.1f}°, IMU xoay: {accum_deg:.1f}°, Lệch trục: {yaw_err_deg:+.2f}°). "
                            f"Chuyển sang TRACKING Luống 2!"
                        )
                        if self.enable_file_logging and self.telemetry_logger:
                            self.telemetry_logger.log_event(
                                "UTURN_SUCCESS_IMU",
                                f"Quay đầu 180° thành công: Start={start_deg:.1f}°, Current={now_deg:.1f}°, Total={accum_deg:.1f}°, Err={yaw_err_deg:.2f}°"
                            )
                        self.transition_after_path(now)
                else:
                    twist.linear.x = lin_vel
                    twist.angular.z = ang_vel
            else:
                lane_center = self.current_lane_y
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
            twist.linear.x  = self.turn_linear_speed
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

        # Khi đang trong chu kỳ căn chỉnh góc mềm, cmd_vel do timer 20Hz (_heading_adjust_timer_callback) kiểm soát
        if not self.is_adjusting_heading:
            self.cmd_vel_pub.publish(twist)

        # ── Telemetry File Logging & Standardized Terminal Status Output ──
        pose_info = self.localization_manager.get_pose()
        gps_info = pose_info.get('gps', {})
        imu_info = pose_info.get('imu', {})
        inf_ms = getattr(self, '_last_inference_ms', 0.0)
        fps_val = (1000.0 / inf_ms) if inf_ms > 0 else 0.0

        if self.enable_file_logging and self.telemetry_logger:
            self.telemetry_logger.log_telemetry({
                'fsm_state': current_state,
                'x': self.current_x,
                'y': self.current_y,
                'yaw': self.current_yaw,
                'steering_angle_deg': self.smoothed_angle_deg,
                'raw_steer_deg': raw_angle,
                'lane_offset': lane_offset,
                'linear_velocity': twist.linear.x,
                'angular_velocity': twist.angular.z,
                'imu_yaw': imu_info.get('yaw', 0.0),
                'imu_angular_vel_z': imu_info.get('angular_vel_z', 0.0),
                'imu_accel_x': imu_info.get('linear_accel_x', 0.0),
                'confidence': confidence,
                'inference_ms': inf_ms,
                'fps': fps_val,
                'distance_traveled': self.distance_traveled,
                'gps_latitude': gps_info.get('latitude', 0.0),
                'gps_longitude': gps_info.get('longitude', 0.0),
                'gps_altitude': gps_info.get('altitude', 0.0),
                'gps_dms': gps_info.get('dms', ''),
                'gps_status': gps_info.get('status', 'NO_FIX')
            })

        # Update state cache for the 1Hz terminal status logger
        display_steer_deg = self.smoothed_angle_deg
        if current_state != FSMState.TRACKING and abs(twist.linear.x) > 0.01:
            display_steer_deg = math.degrees(math.atan2(twist.angular.z * 0.58, twist.linear.x))
        self._last_image_time = time.time()
        self._latest_confidence = confidence
        self._latest_steer_deg = display_steer_deg
        self._latest_twist = twist
        self._latest_gps_info = gps_info
        self._latest_inf_ms = getattr(self, '_last_inference_ms', 0.0)

    def status_timer_callback(self):
        """Định kỳ in trạng thái trực quan chuẩn 1 Hz ra terminal khi đang tự hành."""
        now_sec = time.time()
        current_state = self.fsm.get_state()

        # 1. Chưa nhận được frame camera nào từ lúc bật: chỉ thông báo 1 lần, KHÔNG spam mỗi giây
        if self._last_image_time == 0.0:
            if not getattr(self, '_waiting_camera_notified', False):
                self.get_logger().info("⏳ Đang chờ nhận hình ảnh từ Camera...")
                self._waiting_camera_notified = True
            return

        # 2. Bị mất tín hiệu camera giữa chừng quá 3 giây: cảnh báo 1 lần
        if (now_sec - self._last_image_time) > 3.0:
            if not getattr(self, '_camera_lost_notified', False):
                self.get_logger().warn("⚠️ Mất tín hiệu Camera quá 3 giây! Xe đang tạm dừng.")
                self._camera_lost_notified = True
            return

        self._camera_lost_notified = False

        # 3. Khi xe đang hoạt động bình thường: in 1 dòng chuẩn, súc tích, cực kỳ dễ đọc
        inf_ms = self._latest_inf_ms
        fps_val = (1000.0 / inf_ms) if inf_ms > 0 else 0.0
        gps_status = self._latest_gps_info.get('status', 'NO_FIX')
        gps_source = self._latest_gps_info.get('source', '')
        gps_str = f" | GPS: {self._latest_gps_info.get('latitude', 0.0):.5f}°, {self._latest_gps_info.get('longitude', 0.0):.5f}°" if (gps_status == 'FIX' and gps_source == 'DIRECT_SENSOR') else ""

        display_state_str = "CHỈNH GÓC" if self.is_adjusting_heading else str(current_state)
        status_msg = (
            f"🌾 [AI Lái Xe] Luống {self.current_lane_idx} (x={self.current_x:4.2f}m) | "
            f"Góc: {self._latest_steer_deg:+5.2f}° | "
            f"Trạng thái: [{display_state_str:^10s}] | "
            f"Tin cậy: {self._latest_confidence*100:4.1f}% | "
            f"AI: {inf_ms:2.0f}ms ({fps_val:3.1f}FPS) | "
            f"V: {self._latest_twist.linear.x:.2f}m/s"
            f"{gps_str}"
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
        
        # Target lateral shift: Tối đa 0.04m (trong luống hẹp 1.0m với xe rộng 0.58m, không được lệch quá 4cm)
        shift_amount = 0.04
        if side == "LEFT":
            nudge_y = lane_center - (shift_amount if dir_x > 0 else -shift_amount)
            self.get_logger().info(f"Obstacle on LEFT at x={x_obs:.2f}m, y={obs_info['y_obs']:.2f}m -> Gentle weave to y={nudge_y:.2f}m")
        else:
            nudge_y = lane_center + (shift_amount if dir_x > 0 else -shift_amount)
            self.get_logger().info(f"Obstacle on RIGHT at x={x_obs:.2f}m, y={obs_info['y_obs']:.2f}m -> Gentle weave to y={nudge_y:.2f}m")

        # Hard clamp nudge_y trong biên an toàn cực hẹp
        nudge_y = float(np.clip(nudge_y, lane_center - 0.05, lane_center + 0.05))

        waypoints = []
        
        # ── Stage 1: Weave Out (kéo dài tối thiểu 0.85m để góc lái êm ái < 10 độ, không bẻ giật 53 độ)
        x_weave_end = x_obs - dir_x * 0.35
        weave_length = max(0.85, dir_x * (x_weave_end - rx))
        for t in np.linspace(0.05, 1.0, 10):
            s = 10.0 * (t**3) - 15.0 * (t**4) + 6.0 * (t**5)
            wp_x = rx + dir_x * (t * weave_length)
            wp_y = ry + s * (nudge_y - ry)
            waypoints.append([wp_x, wp_y])
            
        # ── Stage 2: Parallel Clearance Corridor (Past x_obs by 0.30m)
        x_clear_end = x_obs + dir_x * 0.30
        clear_length = max(0.40, dir_x * (x_clear_end - (rx + dir_x * weave_length)))
        for d in np.linspace(0.06, clear_length, 6):
            wp_x = rx + dir_x * (weave_length + d)
            wp_y = nudge_y
            waypoints.append([wp_x, wp_y])
            
        # ── Stage 3: Smooth Quintic Polynomial Return (kéo dài 0.70m trở về giữa luống)
        x_return_start = rx + dir_x * (weave_length + clear_length)
        return_length = 0.70
        for t in np.linspace(0.05, 1.0, 8):
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

    def destroy_node(self):
        """Clean shutdown hook to stop robot and record final session metrics."""
        try:
            self.StopRobot()
        except Exception:
            pass
        if getattr(self, 'enable_file_logging', False) and getattr(self, 'telemetry_logger', None):
            try:
                run_dur = 0.0
                if self.node_start_time is not None:
                    run_dur = (self.get_clock().now() - self.node_start_time).nanoseconds / 1e9
                summary_msg = (
                    f"cnn_driver node stopped cleanly. "
                    f"Total duration: {run_dur:.1f}s | "
                    f"Total distance: {self.distance_traveled:.2f}m | "
                    f"Final FSM: {self.fsm.get_state()}"
                )
                self.telemetry_logger.log_event("SYSTEM_STOP", summary_msg)
            except Exception:
                pass
        super().destroy_node()


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
