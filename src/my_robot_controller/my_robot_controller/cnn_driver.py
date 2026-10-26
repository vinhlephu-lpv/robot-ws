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



def normalize_angle(angle: float) -> float:
    """Normalizes an angle to the range [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class CnnDriverNode(Node):
    def __init__(self):
        super().__init__('cnn_driver_node')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('model_path', '')
        self.declare_parameter('input_height', 512)
        self.declare_parameter('input_width', 512)
        self.declare_parameter('num_threads', 0)
        self.declare_parameter('mask_threshold', 0.04)
        self.declare_parameter('roi_ratio', 0.80)
        self.declare_parameter('linear_speed', 0.075)
        self.declare_parameter('turn_linear_speed', 0.075)
        self.declare_parameter('turn_angular_speed', 0.38)
        self.declare_parameter('low_confidence_threshold', 0.35)
        self.declare_parameter('high_confidence_threshold', 0.50)
        self.declare_parameter('lambda_smc', 2.0)
        self.declare_parameter('k_smc', 3.5)
        self.declare_parameter('eta_smc', 0.6)
        self.declare_parameter('phi_smc', 0.5)
        self.declare_parameter('max_steering_angle_deg', 14.0)
        self.declare_parameter('turn_in_place_threshold_deg', 2.0)
        self.declare_parameter('turn_in_place_trigger_frames', 3)
        self.declare_parameter('turn_in_place_resume_deg', 1.2)
        self.declare_parameter('heading_adjust_aligned_frames', 3)
        self.declare_parameter('camera_trim_deg', 0.0)
        self.declare_parameter('row_spacing', 0.80)
        self.declare_parameter('ema_alpha', 0.45)
        self.declare_parameter('enable_uturn', True)
        self.declare_parameter('uturn_mode', 'PIVOT')
        self.declare_parameter('warmup_time', 3.0)
        self.declare_parameter('navigation_mode', 'auto_three_lanes')
        self.declare_parameter('min_row_length', 1.00)
        self.declare_parameter('max_row_length', 30.0)
        self.declare_parameter('low_conf_frames_threshold', 10)
        self.declare_parameter('drive_out_distance', 0.55)
        self.declare_parameter('min_turn_angle_deg', 130.0)
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
        self.declare_parameter('goal_2_y', 0.80)
        self.declare_parameter('goal_3_x', 0.0)
        self.declare_parameter('goal_3_y', 0.80)
        self.declare_parameter('turn_side', 'RIGHT')
        self.declare_parameter('omega_open_angle_deg', 35.0)
        self.declare_parameter('omega_r1', 0.85)
        self.declare_parameter('omega_clearance', 0.25)
        self.declare_parameter('omega_lead_in', 0.40)

        # ── Tham số Đánh Lái Liên Tục Trong Luống (Continuous Dynamic Steer) ────
        self.declare_parameter('tracking_steer_mode', 'PIVOT_STOP') # 'PIVOT_STOP' (mặc định) hoặc 'CONTINUOUS_STEER'
        self.declare_parameter('in_row_max_steer_deg', 6.5)         # độ — Giới hạn góc bẻ lái khi đi trong hàng (đủ lực nắn về tim luống)
        self.declare_parameter('in_row_max_yaw_dev_deg', 7.0)       # độ — Khóa lệch góc thân xe so với tim luống ban đầu <= 7.0 độ
        self.declare_parameter('steer_trigger_deg', 2.0)            # độ — Ngưỡng bắt đầu bẻ lái
        self.declare_parameter('steer_trigger_frames', 3)           # frames — Số frame lệch liên tiếp để bắt đầu bẻ lái (phản xạ nhanh)
        self.declare_parameter('steer_resume_deg', 1.2)             # độ — Ngưỡng thẳng hàng kết thúc bẻ lái
        self.declare_parameter('steer_aligned_frames', 2)           # frames — Số frame liên tiếp < steer_resume_deg để hồi thẳng ngay
        self.declare_parameter('steer_boost_speed', 0.115)          # m/s — Vận tốc bánh ngoài khi bẻ lái (~11.0 RPM)
        self.declare_parameter('steer_brake_speed', 0.015)          # m/s — Vận tốc bánh trong khi bẻ lái (~1.4 RPM, tạo Δv=0.10m/s)
        self.declare_parameter('steer_ramp_time', 0.15)             # s — Thời gian ramp gia tốc/giảm tốc nhanh và mượt

        p = self.get_parameter
        self.model_path               = p('model_path').value
        self.input_height             = p('input_height').value
        self.input_width              = p('input_width').value
        self.num_threads              = p('num_threads').value
        self.mask_threshold           = p('mask_threshold').value
        self.roi_ratio                = float(p('roi_ratio').value)
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
        self.turn_in_place_trigger_frames = int(p('turn_in_place_trigger_frames').value)
        self.turn_in_place_resume_deg    = float(p('turn_in_place_resume_deg').value)
        self.heading_adjust_aligned_frames = int(p('heading_adjust_aligned_frames').value)
        self.camera_trim_deg             = float(p('camera_trim_deg').value)
        self.row_spacing              = p('row_spacing').value
        self.ema_alpha                = p('ema_alpha').value
        self.enable_uturn             = p('enable_uturn').value
        self.uturn_mode               = p('uturn_mode').value
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

        # Tham số Đánh Lái Liên Tục & Giới Hạn Trong Luống
        self.tracking_steer_mode      = str(p('tracking_steer_mode').value).upper()
        self.in_row_max_steer_deg     = float(p('in_row_max_steer_deg').value)
        self.in_row_max_yaw_dev_deg   = float(p('in_row_max_yaw_dev_deg').value)
        self.steer_trigger_deg        = float(p('steer_trigger_deg').value)
        self.steer_trigger_frames     = int(p('steer_trigger_frames').value)
        self.steer_resume_deg         = float(p('steer_resume_deg').value)
        self.steer_aligned_frames     = int(p('steer_aligned_frames').value)
        self.steer_boost_speed        = float(p('steer_boost_speed').value)
        self.steer_brake_speed        = float(p('steer_brake_speed').value)
        self.steer_ramp_time          = float(p('steer_ramp_time').value)

        # Tự động đồng bộ tọa độ Y: Quy ước người dùng ((+) = Bên Phải, (-) = Bên Trái)
        # với hệ tọa độ ROS REP-103 nội bộ ((+) = Trái, (-) = Phải):
        if self.turn_side.upper() == 'RIGHT':
            self.goal_2_y_ros = -abs(self.goal_2_y)
            self.goal_3_y_ros = -abs(self.goal_3_y)
        else:
            self.goal_2_y_ros = abs(self.goal_2_y)
            self.goal_3_y_ros = abs(self.goal_3_y)

        if self.enable_file_logging:
            self.telemetry_logger = TelemetryLogger(log_dir=self.log_output_dir)
        else:
            self.telemetry_logger = None

        # ── CV Bridge ─────────────────────────────────────────────────
        self.bridge = CvBridge() if CvBridge is not None else None
        if self.bridge is None:
            self.get_logger().warn("CvBridge unavailable — using numpy fallback.")

        if self.image_topic == '/camera/disabled_on_slave':
            self.inference = None
        elif not self.use_hsv_mask:
            if not self.model_path or not os.path.exists(self.model_path):
                target_name = os.path.basename(self.model_path) if self.model_path else 'crop_row_cnn_best_final_int8.onnx'
                candidate_names = [target_name, 'crop_row_cnn_v2.onnx', 'crop_row_cnn_best_final_int8.onnx', 'crop_row_cnn_best_final.onnx']
                
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
                self.get_logger().warn(f"⚠️ Không tìm thấy file model ONNX cục bộ ({self.model_path}). Đang chuyển sang chế độ Slave nhận góc lái từ Laptop qua Wi-Fi (/crop_row/detection).")
                self.inference = None
            else:
                self.inference = InferenceHandler(
                    model_path=self.model_path,
                    mask_threshold=self.mask_threshold,
                    input_size=(self.input_height, self.input_width),
                    use_hsv_mask=self.use_hsv_mask,
                    num_threads=self.num_threads,
                    roi_ratio=self.roi_ratio
                )
        else:
            self.inference = None
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
        self.is_adjusting_heading       = False
        self.heading_adjust_stage       = None # None, 'BRAKE_DECEL', 'SETTLE_BEFORE_TURN', 'ROTATING', 'SETTLE_AFTER_TURN'
        self.heading_adjust_stage_start = 0.0
        self.heading_adjust_start_v     = 0.0
        self.heading_adjust_target_yaw  = 0.0
        self.heading_adjust_dir         = 0.0
        self.heading_adjust_start_time  = 0.0
        self.heading_adjust_cooldown_until = 0.0
        self._aligned_frame_count       = 0
        self.turn_in_place_trigger_count = 0
        self.forward_resume_start_time  = 0.0
        self.current_lane_y             = 0.0

        # ── Continuous dynamic steering state variables ───────────────
        self.steer_active_side        = None  # None (thẳng), 'RIGHT' (bẻ phải), 'LEFT' (bẻ trái)
        self.steer_current_v_l        = float(self.linear_speed)
        self.steer_current_v_r        = float(self.linear_speed)
        self.steer_straight_frames    = 0
        self.steer_trigger_counter    = 0
        self.steer_candidate_side     = None

        # ── Odometry tracking & Sensor Health ─────────────────────────
        self.distance_traveled  = 0.0    # m — cộng dồn từ đầu hàng
        self._prev_odom_x       = None
        self._prev_odom_y       = None
        self._odom_received     = False
        self._last_odom_time    = 0.0
        self._imu_received      = False
        self._last_imu_time     = 0.0
        self._scan_received     = False
        self._last_scan_time    = 0.0

        # ── Chu kỳ Chờ Ổn Định Khởi Động 3 Giây (Startup 3s Stabilization) ──
        self._stabilization_started     = False
        self._stabilization_start_time  = None
        self._is_stabilized             = False
        self._last_stab_tick            = -1
        self._last_sensor_wait_log_time = 0.0

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

        # ── Pivot U-Turn State Variables ──────────────────────────────
        self.pivot_stage              = 'EXIT_ROW'
        self.pivot_start_x            = 0.0
        self.pivot_start_y            = 0.0
        self.pivot_start_yaw          = 0.0
        self.pivot_turn_dir           = -1.0
        self.pivot_target_yaw_1       = 0.0
        self.pivot_target_yaw_2       = 0.0
        self.pivot_stage_start_time   = 0.0
        self.pivot_settle_until       = 0.0
        self.cross_start_x            = 0.0
        self.cross_start_y            = 0.0
        self.cross_start_time         = 0.0

        # ── Giám sát góc IMU vs Encoder khi xoay tại chỗ ──────────────
        self.current_wheel_yaw        = 0.0
        self.pivot_1_start_imu_yaw    = 0.0
        self.pivot_1_start_wheel_yaw  = 0.0
        self.pivot_2_start_imu_yaw    = 0.0
        self.pivot_2_start_wheel_yaw  = 0.0
        self.pivot_2_see_row_frames   = 0
        self.u_turn_see_row_frames    = 0

        self._latest_left_side_dist   = float('inf')
        self._latest_right_side_dist  = float('inf')

        # ── Quản lý 3 Pha Hoạt Động (Đầu hàng - Trong hàng - Thoát hàng) ──
        self.initial_alignment_done   = False
        self.row_reference_yaw        = None
        self._entry_align_counter     = 0
        self._exit_recovery_frames    = 0
        self._last_spike_warn_time    = 0.0
        self._last_yaw_clamp_log_time = 0.0

        # ── Image logging ─────────────────────────────────────────────
        self.last_img_save_time = 0.0
        self.output_dir = os.path.join(os.path.expanduser('~'), 'ros2_debug_imgs')

        # ── Publishers & Subscribers ──────────────────────────────────
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.gps_pub     = self.create_publisher(NavSatFix, '/localization/gps', 10)
        self.plan_pub    = self.create_publisher(RosPath, '/plan', 10)
        self.plan_timer  = self.create_timer(1.0, self.publish_mission_path)
        self.heading_adjust_timer = self.create_timer(0.05, self._heading_adjust_timer_callback)
        self.pivot_uturn_timer = self.create_timer(0.05, self._pivot_uturn_timer_callback)

        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, qos_profile_sensor_data)

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
            Float32MultiArray, '/crop_row/detection', self.remote_cnn_callback, qos_profile_sensor_data
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
        self._latest_raw_angle = 0.0
        self._latest_lane_offset = 0.0
        self._current_heading_adjust_w = 0.0
        self.status_timer = self.create_timer(1.0, self.status_timer_callback)
        self.telemetry_timer = self.create_timer(0.10, self._telemetry_timer_callback)

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
        self._latest_twist = twist
        self._current_heading_adjust_w = 0.0
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
        self.current_wheel_yaw = wheel_yaw

        is_uturn_active = (
            self.fsm.get_state() in [FSMState.UTURN_PLANNING, FSMState.UTURN_EXECUTION] or
            (self.fsm.get_state() == FSMState.PATH_FOLLOWING and self.fsm.state_before_planning == FSMState.UTURN_PLANNING) or
            self.is_trimming_uturn_heading or
            getattr(self, 'is_adjusting_heading', False)
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
        self._imu_received = True
        self._last_imu_time = time.time()
        self.localization_manager.update_imu(msg)
        current_imu_yaw = self.localization_manager.imu_yaw

        # 1. Tích lũy góc quay thực tế từ IMU khi xe đang quay đầu hoặc căn chỉnh góc
        is_uturn_active = (
            self.fsm.get_state() in [FSMState.UTURN_PLANNING, FSMState.UTURN_EXECUTION] or
            (self.fsm.get_state() == FSMState.PATH_FOLLOWING and self.fsm.state_before_planning == FSMState.UTURN_PLANNING) or
            self.is_trimming_uturn_heading or
            getattr(self, 'is_adjusting_heading', False)
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
                    turn_speed = max(0.20, min(float(self.turn_angular_speed), 0.45))
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
        self._last_odom_time = time.time()

        if self._prev_odom_x is not None:
            dx = x - self._prev_odom_x
            dy = y - self._prev_odom_y
            step_dist = math.sqrt(dx*dx + dy*dy)
            # Chỉ tích lũy quãng đường khi đang thực sự tịnh tiến trong TRACKING (không xoay căn chỉnh)
            if self.fsm.get_state() == FSMState.TRACKING and not getattr(self, 'is_adjusting_heading', False):
                # Kiểm tra vận tốc tiến tức thời để loại bỏ dịch chuyển ảo khi xoay tại chỗ
                if abs(getattr(self, '_latest_twist', Twist()).linear.x) > 0.01:
                    self.distance_traveled += step_dist

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
        Timer 20Hz: Xoay căn chỉnh góc dứt khoát có bù mô-men IMU vòng kín (Closed-loop IMU Anti-Stall).
        Đã loại bỏ hoàn toàn các giai đoạn giảm tốc ngắt quãng (BRAKE_DECEL, SETTLE_BEFORE_TURN, SETTLE_AFTER_TURN)
        vốn làm động cơ rơi vào vùng < 4 RPM gây kêu hộp số và khựng xe.
        """
        if not self.is_adjusting_heading or self.fsm.get_state() != FSMState.TRACKING:
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9
        total_elapsed = now_sec - self.heading_adjust_start_time

        # Kiểm tra góc đã xoay thực tế qua IMU:
        cur_imu_yaw = float(self.localization_manager.imu_yaw)
        start_imu_yaw = getattr(self, 'heading_adjust_start_imu_yaw', cur_imu_yaw)
        rotated_deg = abs(math.degrees(normalize_angle(cur_imu_yaw - start_imu_yaw)))
        target_rot_deg = getattr(self, 'heading_adjust_target_deg', 4.0)
        resume_threshold = float(getattr(self, 'turn_in_place_resume_deg', 1.5))

        # Thoát ngay khi đã xoay đủ góc mục tiêu IMU hoặc CNN đã báo thẳng hàng hoặc quá 3.5s:
        if rotated_deg >= target_rot_deg or abs(self.smoothed_angle_deg) <= resume_threshold or total_elapsed > 3.5:
            self.is_adjusting_heading = False
            self.heading_adjust_stage = None
            self._current_heading_adjust_w = 0.0
            self._heading_boost_w = 0.0
            self.heading_adjust_cooldown_until = now_sec + 1.5
            self._aligned_frame_count = 0
            self.forward_resume_start_time = now_sec
            if not getattr(self, 'initial_alignment_done', False):
                self.initial_alignment_done = True
                self.row_reference_yaw = float(self.current_yaw)
            twist = Twist()
            twist.linear.x = float(self.linear_speed)
            twist.angular.z = 0.0
            self._latest_twist = twist
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info(
                f"✅ [ĐIỀU HƯỚNG CNN HOÀN TẤT] IMU đã xoay {rotated_deg:.1f}° (Góc CNN còn {self.smoothed_angle_deg:+.2f}°) "
                f"-> Lăn thẳng mượt mà ở vận tốc {self.linear_speed:.3f} m/s!"
            )
            return

        # Giới hạn góc lệch thân xe trong hàng (chống quay lố vào vách thùng):
        if getattr(self, 'initial_alignment_done', False) and getattr(self, 'inside_row', False):
            ref_y = getattr(self, 'row_reference_yaw', None)
            if ref_y is not None:
                c_dev = math.degrees(normalize_angle(float(self.current_yaw) - ref_y))
                max_dev = float(getattr(self, 'in_row_max_yaw_dev_deg', 6.0))
                if c_dev >= max_dev and self.heading_adjust_dir > 0:
                    self.heading_adjust_dir = -1.0
                elif c_dev <= -max_dev and self.heading_adjust_dir < 0:
                    self.heading_adjust_dir = 1.0

        # Bù lực xoay vòng kín theo vận tốc góc IMU thực tế:
        base_w = max(0.75, float(getattr(self, 'turn_angular_speed', 0.85)))
        imu_wz = abs(float(getattr(self.localization_manager, 'imu_angular_vel_z', 0.0)))
        if total_elapsed > 0.15 and imu_wz < 0.18:
            self._heading_boost_w = min(0.50, getattr(self, '_heading_boost_w', 0.0) + 0.04)
        elif imu_wz > 0.55:
            self._heading_boost_w = max(0.0, getattr(self, '_heading_boost_w', 0.0) - 0.03)

        effective_w = min(1.30, base_w + getattr(self, '_heading_boost_w', 0.0))
        current_w = self.heading_adjust_dir * effective_w
        self._current_heading_adjust_w = current_w

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = current_w
        self._latest_twist = twist
        self.cmd_vel_pub.publish(twist)

    def _pivot_uturn_timer_callback(self):
        """
        Timer 20Hz điều phối chu trình quay đầu chữ U 90°-90° (Pivot U-Turn)
        kết hợp chặt chẽ giữa cảm biến IMU 50Hz (nắm chặt góc quay 90° thực tế, bù lực thắng ma sát cỏ)
        và YOLO nhận diện Mũi Tên Vàng (cuối hàng 1) + Mũi Tên Đen (đầu hàng 2).
        """
        if self.fsm.get_state() != FSMState.UTURN_EXECUTION or getattr(self, 'uturn_mode', 'PIVOT').upper() != 'PIVOT':
            return

        now = self.get_clock().now()
        now_sec = now.nanoseconds / 1e9

        # Nếu đang trong thời gian ổn định ngắn (0.15s) giữa các giai đoạn:
        if now_sec < getattr(self, 'pivot_settle_until', 0.0):
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self._latest_twist = twist
            self.cmd_vel_pub.publish(twist)
            return

        stage = getattr(self, 'pivot_stage', 'EXIT_ROW')
        elapsed_stage = now_sec - getattr(self, 'pivot_stage_start_time', now_sec)

        # =========================================================================
        # GIAI ĐOẠN 1: EXIT_ROW (Tiến thoát miệng luống nếu chưa có mốc Mũi Tên Vàng)
        # =========================================================================
        if stage == 'EXIT_ROW':
            dx = self.current_x - self.pivot_start_x
            dy = self.current_y - self.pivot_start_y
            dist_exit = math.hypot(dx, dy)
            # Khi kích hoạt bởi Mũi Tên Vàng (<= 1.0m), biển vàng đã nằm ngoài miệng hàng 1 và thẳng hàng ngang
            # với Mũi Tên Đen bên hàng 2 -> Xoay PIVOT_1 ngay lập tức (exit_target = 0.0m) để không vượt lố!
            if getattr(self, 'uturn_triggered_by_yellow', False):
                exit_target = 0.0
            else:
                exit_target = float(getattr(self, 'drive_out_distance', 0.45))

            v_exit = max(0.09, min(self.linear_speed, getattr(self, 'turn_linear_speed', 0.10)))
            max_exit_time = (exit_target / max(0.05, float(v_exit))) + 2.0
            is_cleared = (dist_exit >= exit_target) or (elapsed_stage >= max_exit_time)

            if is_cleared:
                self.StopRobot()
                self.pivot_settle_until = now_sec + 0.15
                self.pivot_stage = 'PIVOT_1'
                self.pivot_stage_start_time = now_sec + 0.15
                self.pivot_1_start_imu_yaw = float(self.localization_manager.imu_yaw)
                self.pivot_1_start_ekf_yaw = float(self.current_yaw)
                self.pivot_1_start_wheel_yaw = getattr(self, 'current_wheel_yaw', float(self.current_yaw))
                self.pivot_1_accum_imu_deg = 0.0
                self._pivot1_prev_imu_yaw = float(self.localization_manager.imu_yaw)
                self._pivot1_boost_w = 0.0

                # ── CHỐT MỐC GOAL 1 & GOAL 2 (+180° BÊN PHẢI) ──
                self.goal_1_x = self.current_x
                self.goal_1_y = self.current_y
                g1_yaw = self.pivot_start_yaw
                spacing = abs(self.row_spacing)

                self.goal_2_x = self.goal_1_x + spacing * math.sin(g1_yaw)
                self.goal_2_y = self.goal_1_y - spacing * math.cos(g1_yaw)
                self.goal_2_y_ros = self.goal_2_y
                self.current_lane_y = self.goal_2_y

                start_x_ref = getattr(self, 'row_start_x', 0.0)
                if start_x_ref is None:
                    start_x_ref = 0.0
                self.goal_3_x = start_x_ref
                self.goal_3_y = self.goal_2_y
                self.goal_3_y_ros = self.goal_2_y

                self.pivot_turn_dir = -1.0 if self.turn_side.upper() == 'RIGHT' else 1.0
                self.pivot_target_yaw_1 = normalize_angle(g1_yaw + self.pivot_turn_dir * (math.pi / 2.0))
                self.pivot_target_yaw_2 = normalize_angle(g1_yaw + math.pi)
                self.pivot_target_imu_1 = normalize_angle(self.pivot_1_start_imu_yaw + self.pivot_turn_dir * (math.pi / 2.0))
                self.pivot_target_imu_2 = normalize_angle(self.pivot_1_start_imu_yaw + math.pi)

                self.get_logger().info(
                    f"🎯 [BẮT ĐẦU PIVOT 1 - XOAY 90° LẦN 1] Vị trí: x={self.goal_1_x:.2f}m, y={self.goal_1_y:.2f}m "
                    f"(IMU Start={math.degrees(self.pivot_1_start_imu_yaw):+.1f}° -> Target 90°={math.degrees(self.pivot_target_imu_1):+.1f}°). "
                    f"Nắm chặt góc IMU xoay đủ 90° sang phải..."
                )
                if self.enable_file_logging and self.telemetry_logger:
                    self.telemetry_logger.log_event(
                        "GOAL_1_REACHED",
                        f"Goal 1: ({self.goal_1_x:.2f}, {self.goal_1_y:.2f}), Goal 2: ({self.goal_2_x:.2f}, {self.goal_2_y:.2f}), Spacing={spacing:.2f}m"
                    )
                return

            yaw_err = normalize_angle(self.pivot_start_yaw - self.current_yaw)
            twist = Twist()
            twist.linear.x = float(v_exit)
            twist.angular.z = float(np.clip(1.2 * yaw_err, -0.15, 0.15))
            self._latest_twist = twist
            self.cmd_vel_pub.publish(twist)

        # =========================================================================
        # GIAI ĐOẠN 2: PIVOT_1 (Xoay 90° lần 1 sau biển Mũi Tên Vàng — Nắm chặt góc IMU)
        # =========================================================================
        elif stage == 'PIVOT_1':
            cur_raw_imu = float(self.localization_manager.imu_yaw)
            cur_ekf_yaw = float(self.current_yaw)
            start_imu_yaw = getattr(self, 'pivot_1_start_imu_yaw', cur_raw_imu)
            start_ekf_yaw = getattr(self, 'pivot_1_start_ekf_yaw', cur_ekf_yaw)

            # Tích lũy góc quay IMU thực tế theo từng chu kỳ 20Hz:
            prev_imu = getattr(self, '_pivot1_prev_imu_yaw', cur_raw_imu)
            step_dyaw = normalize_angle(cur_raw_imu - prev_imu)
            self._pivot1_prev_imu_yaw = cur_raw_imu
            if step_dyaw * self.pivot_turn_dir > -0.02:
                self.pivot_1_accum_imu_deg = getattr(self, 'pivot_1_accum_imu_deg', 0.0) + abs(math.degrees(step_dyaw))

            delta_raw_imu_deg = abs(math.degrees(normalize_angle(cur_raw_imu - start_imu_yaw)))
            delta_ekf_deg = abs(math.degrees(normalize_angle(cur_ekf_yaw - start_ekf_yaw)))
            delta_imu_deg = max(delta_raw_imu_deg, delta_ekf_deg, getattr(self, 'pivot_1_accum_imu_deg', 0.0))

            target_imu_1 = getattr(self, 'pivot_target_imu_1', self.pivot_target_yaw_1)
            yaw_err_imu = normalize_angle(target_imu_1 - cur_raw_imu)
            yaw_err_ekf = normalize_angle(self.pivot_target_yaw_1 - cur_ekf_yaw)
            yaw_err_deg = math.degrees(yaw_err_imu) if abs(math.degrees(yaw_err_imu)) < abs(math.degrees(yaw_err_ekf)) else math.degrees(yaw_err_ekf)
            rem_deg = max(0.0, 90.0 - delta_imu_deg)

            cur_wheel_yaw = getattr(self, 'current_wheel_yaw', cur_ekf_yaw)
            start_wheel_yaw = getattr(self, 'pivot_1_start_wheel_yaw', cur_wheel_yaw)
            delta_wheel_deg = abs(math.degrees(normalize_angle(cur_wheel_yaw - start_wheel_yaw)))

            # Bắt buộc IMU phải thực sự xoay đủ >= 87.0° (hoặc >= 82.0° và sai lệch trục <= 3.0°):
            # Tuyệt đối không cắt sớm ở 10s khi xe mới quay 20° - 50°!
            turn_90_done = (delta_imu_deg >= 87.0) or (delta_imu_deg >= 82.0 and abs(yaw_err_deg) <= 3.0)
            safety_timeout = (elapsed_stage >= 35.0)

            if turn_90_done or safety_timeout:
                self.StopRobot()
                self.pivot_settle_until = now_sec + 0.20
                self.pivot_stage = 'CROSS_DRIVE'
                self.pivot_stage_start_time = now_sec + 0.20
                self.cross_start_x = self.current_x
                self.cross_start_y = self.current_y
                self.cross_hold_yaw = float(self.current_yaw)
                self.get_logger().info(
                    f"🛑 [PIVOT 1 HOÀN TẤT] Đã xoay đủ 90° lần 1! "
                    f"(IMU quay: {delta_imu_deg:.1f}°/90.0°, Lệch đích: {yaw_err_deg:+.2f}°, Bánh xe: {delta_wheel_deg:.1f}°). "
                    f"Bắt đầu CROSS_DRIVE chạy thẳng ngang tìm Mũi Tên Đen..."
                )
                if self.enable_file_logging and self.telemetry_logger:
                    self.telemetry_logger.log_event(
                        "PIVOT_STAGE_PIVOT_1_DONE",
                        f"Xoay 90° lần 1 xong: IMU={delta_imu_deg:.1f}°, Wheel={delta_wheel_deg:.1f}°, err={yaw_err_deg:.2f}°"
                    )
                return

            # ── BỘ ĐIỀU KHIỂN BÙ LỰC XOAY VÒNG KÍN THEO VẬN TỐC GÓC IMU ──
            imu_wz = abs(float(getattr(self.localization_manager, 'imu_angular_vel_z', 0.0)))
            base_w = max(0.85, float(getattr(self, 'turn_angular_speed', 0.85)))

            if elapsed_stage > 0.25 and imu_wz < 0.22:
                self._pivot1_boost_w = min(0.65, getattr(self, '_pivot1_boost_w', 0.0) + 0.045)
                if now_sec - getattr(self, '_last_pivot1_stall_warn', 0.0) >= 1.5:
                    self._last_pivot1_stall_warn = now_sec
                    self.get_logger().info(
                        f"💪 [PIVOT 1 - TĂNG LỰC BÙ MA SÁT] Đã quay {delta_imu_deg:.1f}°/90.0° (còn {rem_deg:.1f}°) "
                        f"-> Tăng lực xoay lên {min(1.45, base_w + self._pivot1_boost_w):.2f} rad/s để thắng ma sát cỏ!"
                    )
            elif imu_wz > 0.65:
                self._pivot1_boost_w = max(0.0, getattr(self, '_pivot1_boost_w', 0.0) - 0.03)

            boost_w = getattr(self, '_pivot1_boost_w', 0.0)
            if rem_deg > 18.0:
                w_mag = min(1.45, base_w + boost_w)
            elif rem_deg > 6.0:
                w_mag = max(0.68, min(1.10, base_w * 0.85 + boost_w * 0.5))
            else:
                w_mag = max(0.55, min(0.85, base_w * 0.70 + boost_w * 0.3))

            turn_dir = self.pivot_turn_dir
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = turn_dir * w_mag
            self._latest_twist = twist
            self.cmd_vel_pub.publish(twist)

        # =========================================================================
        # GIAI ĐOẠN 3: CROSS_DRIVE (Chạy ngang hướng tới biển Mũi Tên Đen)
        # =========================================================================
        elif stage == 'CROSS_DRIVE':
            dx = self.current_x - getattr(self, 'cross_start_x', self.current_x)
            dy = self.current_y - getattr(self, 'cross_start_y', self.current_y)
            dist_cross = math.hypot(dx, dy)
            target_spacing = abs(self.row_spacing)

            v_cross = max(0.095, float(getattr(self, 'turn_linear_speed', 0.10)))
            max_cross_time = 20.0

            # ── Kiểm tra Mũi Tên Đen (Vật mốc đầu Luống 2) qua YOLO ──
            r_cnn = getattr(self, '_remote_cnn_data', None)
            black_arrow_detected = False
            black_arrow_visible_far = False
            b_dist = -1.0
            b_steer_deg = 0.0
            if r_cnn is not None:
                b_trig = (r_cnn.get('trigger_black', 0.0) >= 1.0)
                b_dist = r_cnn.get('marker_dist', -1.0)
                b_class = r_cnn.get('marker_class', -1.0)
                b_conf = r_cnn.get('marker_conf', 0.0)
                b_steer_deg = float(r_cnn.get('arrow_steer_deg', 0.0))

                if b_class == 1.0 and b_conf >= 0.35 and 0.05 < b_dist <= 2.50:
                    black_arrow_visible_far = True
                    self._has_seen_black_arrow = True

                # Kích hoạt xoay 90° lần 2 khi đến gần Mũi Tên Đen (<= 1.08m):
                if dist_cross >= 0.10 or elapsed_stage >= 1.0:
                    if b_trig or (b_class == 1.0 and 0.05 < b_dist <= 1.08 and b_conf >= 0.35):
                        black_arrow_detected = True

            reach_black_marker = black_arrow_detected
            # Nếu đang nhìn thấy Mũi Tên Đen ở phía trước (> 1.08m), ưu tiên chạy tới đúng Mũi Tên Đen (tối đa 2.2m);
            # nếu không thấy Mũi Tên Đen thì dùng quãng đường row_spacing dự phòng:
            odom_limit = 2.20 if (black_arrow_visible_far or getattr(self, '_has_seen_black_arrow', False)) else max(1.05, target_spacing * 1.25)
            reach_odometry = (dist_cross >= odom_limit)
            reach_timeout = (elapsed_stage >= max_cross_time)

            if reach_black_marker or reach_odometry or reach_timeout:
                if reach_black_marker:
                    trigger_msg = f"🎯 [YOLO BLACK ARROW TRIGGER] Đã gặp MŨI TÊN ĐEN đầu Luống 2 (cự ly={b_dist:.2f}m <= 1.05m, dist_cross={dist_cross:.2f}m)!"
                elif reach_odometry:
                    trigger_msg = f"🛑 [ODOMETRY TRIGGER] Đã chạy ngang đủ khoảng cách ({dist_cross:.2f}m/{odom_limit:.2f}m)!"
                else:
                    trigger_msg = f"⚠️ [TIMEOUT TRIGGER] Hết thời gian chạy ngang ({elapsed_stage:.1f}s)!"

                self.StopRobot()
                self._has_seen_black_arrow = False
                self.pivot_settle_until = now_sec + 0.20
                self.pivot_stage = 'PIVOT_2'
                self.pivot_stage_start_time = now_sec + 0.20
                self.pivot_2_start_imu_yaw = float(self.localization_manager.imu_yaw)
                self.pivot_2_start_ekf_yaw = float(self.current_yaw)
                self.pivot_2_start_wheel_yaw = getattr(self, 'current_wheel_yaw', float(self.current_yaw))
                self.pivot_2_accum_imu_deg = 0.0
                self._pivot2_prev_imu_yaw = float(self.localization_manager.imu_yaw)
                self._pivot2_boost_w = 0.0
                self.pivot_target_imu_2 = normalize_angle(self.pivot_2_start_imu_yaw + self.pivot_turn_dir * (math.pi / 2.0))
                self.get_logger().info(
                    f"{trigger_msg} Bắt đầu PIVOT 2 xoay 90° lần 2 khóa thẳng vào Luống 2..."
                )
                if self.enable_file_logging and self.telemetry_logger:
                    self.telemetry_logger.log_event("PIVOT_STAGE_CROSS_DONE", f"{trigger_msg}")
                return

            # Giữ hướng thẳng ngang kết hợp hướng về tâm biển Mũi Tên Đen nếu nhìn thấy từ xa:
            hold_yaw = getattr(self, 'cross_hold_yaw', self.pivot_target_yaw_1)
            yaw_err = normalize_angle(hold_yaw - self.current_yaw)
            if black_arrow_visible_far:
                arrow_w = -math.radians(b_steer_deg) * 1.4
                ang_cmd = float(np.clip(0.7 * arrow_w + 0.8 * yaw_err, -0.35, 0.35))
            else:
                ang_cmd = float(np.clip(1.5 * yaw_err, -0.30, 0.30))

            twist = Twist()
            twist.linear.x = float(v_cross)
            twist.angular.z = ang_cmd
            self._latest_twist = twist
            self.cmd_vel_pub.publish(twist)

        # =========================================================================
        # GIAI ĐOẠN 4: PIVOT_2 (Xoay 90° lần 2 sau biển Mũi Tên Đen — Nắm chặt góc IMU)
        # =========================================================================
        elif stage == 'PIVOT_2':
            cur_raw_imu = float(self.localization_manager.imu_yaw)
            cur_ekf_yaw = float(self.current_yaw)
            start_imu_yaw = getattr(self, 'pivot_2_start_imu_yaw', cur_raw_imu)
            start_ekf_yaw = getattr(self, 'pivot_2_start_ekf_yaw', cur_ekf_yaw)

            prev_imu = getattr(self, '_pivot2_prev_imu_yaw', cur_raw_imu)
            step_dyaw = normalize_angle(cur_raw_imu - prev_imu)
            self._pivot2_prev_imu_yaw = cur_raw_imu
            if step_dyaw * self.pivot_turn_dir > -0.02:
                self.pivot_2_accum_imu_deg = getattr(self, 'pivot_2_accum_imu_deg', 0.0) + abs(math.degrees(step_dyaw))

            delta_raw_imu_deg = abs(math.degrees(normalize_angle(cur_raw_imu - start_imu_yaw)))
            delta_ekf_deg = abs(math.degrees(normalize_angle(cur_ekf_yaw - start_ekf_yaw)))
            delta_imu_deg = max(delta_raw_imu_deg, delta_ekf_deg, getattr(self, 'pivot_2_accum_imu_deg', 0.0))

            target_imu_2 = getattr(self, 'pivot_target_imu_2', self.pivot_target_yaw_2)
            yaw_err_imu = normalize_angle(target_imu_2 - cur_raw_imu)
            yaw_err_ekf = normalize_angle(self.pivot_target_yaw_2 - cur_ekf_yaw)
            yaw_err_deg = math.degrees(yaw_err_imu) if abs(math.degrees(yaw_err_imu)) < abs(math.degrees(yaw_err_ekf)) else math.degrees(yaw_err_ekf)
            rem_deg = max(0.0, 90.0 - delta_imu_deg)

            cur_wheel_yaw = getattr(self, 'current_wheel_yaw', cur_ekf_yaw)
            start_wheel_yaw = getattr(self, 'pivot_2_start_wheel_yaw', cur_wheel_yaw)
            delta_wheel_deg = abs(math.degrees(normalize_angle(cur_wheel_yaw - start_wheel_yaw)))

            total_u_turn_deg = 90.0 + delta_imu_deg
            conf = getattr(self, '_latest_confidence', 0.0)
            high_conf = getattr(self, 'high_confidence_threshold', 0.42)

            # Chỉ cho phép CNN bắt sớm Luống 2 khi lần xoay thứ 2 đã đạt ít nhất >= 78° (tổng >= 168°):
            if delta_imu_deg >= 78.0 and conf >= high_conf:
                self.pivot_2_see_row_frames = getattr(self, 'pivot_2_see_row_frames', 0) + 1
            else:
                self.pivot_2_see_row_frames = 0

            row_acquired_by_cnn = (delta_imu_deg >= 78.0 and self.pivot_2_see_row_frames >= 5)
            target_aligned = (delta_imu_deg >= 86.0) or (delta_imu_deg >= 80.0 and abs(yaw_err_deg) <= 3.0)
            safety_timeout = (elapsed_stage >= 35.0)

            if row_acquired_by_cnn or target_aligned or safety_timeout:
                self.StopRobot()
                self.current_lane_idx = 2
                self.current_lane_y = getattr(self, 'goal_2_y', (-abs(self.row_spacing) if self.turn_side.upper() == 'RIGHT' else abs(self.row_spacing)))
                self.inside_row = True
                self.has_seen_row = True
                self.initial_alignment_done = True
                self.row_reference_yaw = float(self.current_yaw)
                self.eor_detected = False
                self.low_confidence_counter = 0
                self.eor_low_conf_frames = 0
                self.smoothed_angle_deg = 0.0
                self.distance_traveled = 0.0
                self.accumulated_turn_angle = 0.0
                self.is_adjusting_heading = False
                self.uturn_triggered_by_yellow = False
                self.pure_pursuit_controller.reset()
                if hasattr(self, 'smc_controller') and self.smc_controller is not None:
                    self.smc_controller.reset()
                self.pivot_2_see_row_frames = 0

                self.transition_to_state(FSMState.TRACKING, now)
                trigger_reason = "CNN BẮT LUỐNG 2 (>=78°)" if row_acquired_by_cnn else ("IMU QUAY ĐỦ 90° LẦN 2 (TỔNG 180°)" if target_aligned else "TIMEOUT AN TOÀN 35s")
                log_msg = (
                    f"🌾 [PIVOT 2 HOÀN THÀNH - {trigger_reason}] Đã xoay {delta_imu_deg:.1f}°/90.0° (Tổng {total_u_turn_deg:.1f}°/180°, "
                    f"Lệch trục={yaw_err_deg:+.2f}°, Conf={conf*100:.1f}%). "
                    f"Bàn giao quyền điều khiển cho AI CNN bám thẳng vào Luống 2!"
                )
                self.get_logger().info(log_msg)
                if self.enable_file_logging and self.telemetry_logger:
                    self.telemetry_logger.log_event("PIVOT_UTURN_COMPLETE", log_msg)
                return

            # ── BỘ ĐIỀU KHIỂN BÙ LỰC XOAY VÒNG KÍN THEO VẬN TỐC GÓC IMU ──
            imu_wz = abs(float(getattr(self.localization_manager, 'imu_angular_vel_z', 0.0)))
            base_w = max(0.85, float(getattr(self, 'turn_angular_speed', 0.85)))

            if elapsed_stage > 0.25 and imu_wz < 0.22:
                self._pivot2_boost_w = min(0.65, getattr(self, '_pivot2_boost_w', 0.0) + 0.045)
                if now_sec - getattr(self, '_last_pivot2_stall_warn', 0.0) >= 1.5:
                    self._last_pivot2_stall_warn = now_sec
                    self.get_logger().info(
                        f"💪 [PIVOT 2 - TĂNG LỰC BÙ MA SÁT] Đã quay {delta_imu_deg:.1f}°/90.0° (còn {rem_deg:.1f}°) "
                        f"-> Tăng lực xoay lên {min(1.45, base_w + self._pivot2_boost_w):.2f} rad/s để thắng ma sát cỏ!"
                    )
            elif imu_wz > 0.65:
                self._pivot2_boost_w = max(0.0, getattr(self, '_pivot2_boost_w', 0.0) - 0.03)

            boost_w = getattr(self, '_pivot2_boost_w', 0.0)
            if rem_deg > 18.0:
                w_mag = min(1.45, base_w + boost_w)
            elif rem_deg > 6.0:
                w_mag = max(0.68, min(1.10, base_w * 0.85 + boost_w * 0.5))
            else:
                w_mag = max(0.55, min(0.85, base_w * 0.70 + boost_w * 0.3))

            turn_dir = self.pivot_turn_dir
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = turn_dir * w_mag
            self._latest_twist = twist
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
            self._current_heading_adjust_w = 0.0
            self._aligned_frame_count = 0
            if new_state == FSMState.TRACKING:
                self.tracking_controller.reset()
                self.steer_active_side = None
                self.steer_current_v_l = float(self.linear_speed)
                self.steer_current_v_r = float(self.linear_speed)
                self.steer_straight_frames = 0
                self.steer_trigger_counter = 0
                self.steer_candidate_side = None
                self.turn_in_place_trigger_count = 0
                self._raw_angle_history = []
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
                'latency_ms': float(msg.data[4]) if len(msg.data) > 4 else 0.0,
                'turn_trigger': float(msg.data[5]) if len(msg.data) > 5 else 0.0,
                'marker_dist': float(msg.data[6]) if len(msg.data) > 6 else -1.0,
                'marker_conf': float(msg.data[7]) if len(msg.data) > 7 else 0.0,
                'marker_class': float(msg.data[8]) if len(msg.data) > 8 else -1.0,
                'trigger_yellow': float(msg.data[9]) if len(msg.data) > 9 else 0.0,
                'trigger_black': float(msg.data[10]) if len(msg.data) > 10 else 0.0,
                'arrow_steer_deg': float(msg.data[11]) if len(msg.data) > 11 else 0.0,
            }
            self._remote_cnn_time = self.get_clock().now().nanoseconds / 1e9

            if getattr(self, '_waiting_camera_notified', False):
                self.get_logger().info("✅ Đã nhận tín hiệu góc lái AI từ Laptop (/crop_row/detection) qua Wi-Fi!")
                self._waiting_camera_notified = False

            # Kích hoạt chu kỳ điều khiển ngay lập tức khi nhận góc lái từ Laptop nếu không có camera cục bộ
            now_sec = time.time()
            if (now_sec - getattr(self, '_last_local_img_time', 0.0)) > 0.3:
                self._step_control(bgr_image=None, external_cnn=self._remote_cnn_data)

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
                self.get_logger().info("✅ Đã nhận tín hiệu Camera cục bộ!")
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

        self._last_local_img_time = time.time()
        self._step_control(bgr_image=bgr_image, external_cnn=None)

    def _step_control(self, bgr_image=None, external_cnn=None):
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        now_sec = now_ns / 1e9

        # ── Perception Processing via PerceptionManager ───────────────
        if external_cnn is None:
            is_remote_cnn = (self._remote_cnn_data is not None) and ((now_sec - self._remote_cnn_time) < 0.6)
            external_cnn = self._remote_cnn_data if is_remote_cnn else None

        self._is_remote_perception = (external_cnn is not None)

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
        if external_cnn is not None:
            self._last_inference_ms = external_cnn.get('latency_ms', 0.0)
        else:
            self._last_inference_ms = (time.time() - t_infer_start) * 1000.0
        
        confidence = perception["confidence"]
        obstacle_detected = perception["obstacle_detected"]
        end_of_row = perception["end_of_row_detected"]
        raw_angle = perception["heading_error"] + getattr(self, 'camera_trim_deg', 0.0)
        lane_center = perception["lane_center"]
        lane_offset = perception["lane_offset"]
        left_side_dist = perception.get("left_side_dist", float('inf'))
        right_side_dist = perception.get("right_side_dist", float('inf'))
        self._latest_left_side_dist = left_side_dist
        self._latest_right_side_dist = right_side_dist

        # ── FSM ───────────────────────────────────────────────────────
        twist        = Twist()
        now          = self.get_clock().now()

        # ── 1. Chu kỳ Khởi Động Ổn Định 3 Giây & Kiểm Tra Cảm Biến ────
        # Chờ đúng warmup_time (3.0s) sau khi nhận góc lái AI & xác nhận cảm biến hoạt động tốt trước khi lăn bánh
        if not getattr(self, '_is_stabilized', False):
            if not getattr(self, '_stabilization_started', False):
                self._stabilization_started = True
                self._stabilization_start_time = now_sec
                start_msg = (
                    f"⏳ [KHỞI ĐỘNG ỔN ĐỊNH - 3s] Đã nhận tín hiệu AI đầu tiên! "
                    f"Tạm giữ xe đứng yên {self.warmup_time:.1f}s để ổn định hệ thống & kiểm tra cảm biến..."
                )
                self.get_logger().info(start_msg)
                if self.enable_file_logging and self.telemetry_logger:
                    self.telemetry_logger.log_event("STARTUP_STABILIZATION", start_msg)

            elapsed_stab = now_sec - self._stabilization_start_time
            remaining_stab = max(0.0, self.warmup_time - elapsed_stab)

            odom_ok = self._odom_received and ((now_sec - getattr(self, '_last_odom_time', 0.0)) < 3.0)
            imu_ok = self._imu_received and ((now_sec - getattr(self, '_last_imu_time', 0.0)) < 3.0)
            lidar_ok = self._scan_received and ((now_sec - getattr(self, '_last_scan_time', 0.0)) < 3.0)
            ai_ok = (confidence >= self.low_confidence_threshold)

            # In thông tin kiểm tra cảm biến và đếm ngược mỗi 1 giây
            current_tick = int(elapsed_stab)
            if current_tick != getattr(self, '_last_stab_tick', -1):
                self._last_stab_tick = current_tick
                s_odom = "✅ OK" if odom_ok else "❌ Chưa nhận"
                s_imu  = "✅ OK" if imu_ok else "❌ Chưa nhận"
                s_lidar= "✅ OK" if lidar_ok else "⚠️ Chưa nhận"
                s_ai   = f"✅ {confidence*100:.0f}%" if ai_ok else f"⚠️ {confidence*100:.0f}%"
                rem_display = max(1, int(math.ceil(remaining_stab)))
                self.get_logger().info(
                    f"⏳ [ỔN ĐỊNH XE - {rem_display}s] "
                    f"Odom: [{s_odom}] | IMU: [{s_imu}] | LiDAR: [{s_lidar}] | AI: [{s_ai}]"
                )

            # Cập nhật cache trạng thái để các luồng telemetry ghi nhận đúng
            self._last_image_time = now_sec
            self._latest_confidence = confidence
            self._latest_raw_angle = raw_angle
            self._latest_lane_offset = lane_offset
            self._latest_steer_deg = raw_angle

            if remaining_stab > 0.0:
                self.StopRobot()
                self._latest_twist = Twist()
                return

            # Đã hết thời gian warmup_time (3s), kiểm tra các cảm biến bắt buộc (Odometry & IMU)
            if not odom_ok or not imu_ok:
                if now_sec - getattr(self, '_last_sensor_wait_log_time', 0.0) >= 1.0:
                    self._last_sensor_wait_log_time = now_sec
                    missing = []
                    if not odom_ok: missing.append("Odometry")
                    if not imu_ok: missing.append("IMU")
                    self.get_logger().warn(
                        f"⚠️ [CHỜ CẢM BIẾN] Đã chờ đủ {self.warmup_time:.1f}s nhưng thiếu dữ liệu: {', '.join(missing)}! Xe tiếp tục dừng chờ..."
                    )
                self.StopRobot()
                self._latest_twist = Twist()
                return

            # Cảm biến và AI đều đã sẵn sàng!
            self._is_stabilized = True
            self.node_start_time = now
            self.state_start_time = now
            self.row_start_x = self.current_x
            self.row_start_y = self.current_y
            self.distance_traveled = 0.0
            s_lidar_str = "OK" if lidar_ok else "OFF"
            ready_msg = (
                f"🚀 [SẴN SÀNG TỰ HÀNH] Cảm biến & AI đã ổn định 100%! "
                f"(Start: x={self.row_start_x:.2f}m, y={self.row_start_y:.2f}m | Odom: OK, IMU: OK, LiDAR: {s_lidar_str}, AI Conf: {confidence*100:.1f}%). "
                f"Bắt đầu lăn bánh tự hành vào luống bắp!"
            )
            self.get_logger().info(ready_msg)
            if self.enable_file_logging and self.telemetry_logger:
                self.telemetry_logger.log_event("SYSTEM_READY", ready_msg)

        if self.node_start_time is None:
            self.node_start_time = now
            self.state_start_time = now
        elapsed_state= (now - self.state_start_time).nanoseconds / 1e9
        elapsed_total= (now - self.node_start_time).nanoseconds / 1e9
        warmup_done  = getattr(self, '_is_stabilized', False) or (elapsed_total > self.warmup_time)

        current_state = self.fsm.get_state()

        # Save diagnostic frame every 3 seconds (sim time, if enabled)
        if self.save_debug_imgs and bgr_image is not None:
            if now_sec - self.last_img_save_time >= 3.0:
                self.last_img_save_time = now_sec
                try:
                    os.makedirs(self.output_dir, exist_ok=True)
                    if hasattr(self.inference, 'latest_mask') and self.inference.latest_mask is not None:
                        mask_resized = cv2.resize(self.inference.latest_mask, (bgr_image.shape[1], bgr_image.shape[0]))
                        binary_mask = (mask_resized >= self.inference.mask_threshold).astype(np.uint8) * 255
                        binary_mask_colored = cv2.cvtColor(binary_mask, cv2.COLOR_GRAY2BGR)
                        overlay_image = bgr_image.copy()
                        overlay_image[binary_mask > 128] = [255, 255, 0] # Cyan BGR
                        blend_image = cv2.addWeighted(bgr_image, 0.6, overlay_image, 0.4, 0)
                        h, w = bgr_image.shape[:2]
                        image_center = (w - 1) / 2.0
                        for y_start in range(0, h, 20):
                            cv2.line(blend_image, (int(image_center), y_start), (int(image_center), min(y_start + 10, h)), (0, 255, 0), 2)
                        lane_center_raw = lane_center * (w / self.inference.input_size[1])
                        lane_center_raw = np.clip(lane_center_raw, 0.0, w - 1.0)
                        line_top = int(h * 0.6)
                        cv2.line(blend_image, (int(lane_center_raw), h - 1), (int(lane_center_raw), line_top), (255, 191, 0), 3)
                        canvas = np.hstack((bgr_image, binary_mask_colored, blend_image))
                        cv2.putText(canvas, f"State: {current_state} | Conf: {confidence:.2f} | Dist: {self.distance_traveled:.2f}m | Steer: {self.smoothed_angle_deg:.2f} deg", 
                                    (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
                        filename = os.path.join(self.output_dir, f"frame_{int(now_sec)}.png")
                        cv2.imwrite(filename, canvas)
                        self.get_logger().debug(f"--- Saved diagnostic frame: {filename} ---")
                except Exception as save_err:
                    self.get_logger().error(f"Failed to save diagnostic frame: {save_err}")

        # Khi Camera/CNN nhìn thấy hàng (confidence >= low_confidence_threshold):
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
            # Nếu nhiệm vụ đã hoàn thành về đích Goal 3, dừng hẳn xe vĩnh viễn, không tự chạy lại
            if getattr(self, 'mission_completed', False):
                return
            # Ngay khi đã qua 3s ổn định cảm biến, chuyển sang TRACKING (kể cả khi đặt xe ở cuối hàng chỉ thấy biển vàng):
            if warmup_done:
                self.transition_to_state(FSMState.TRACKING, now)
            return

        # ── TRACKING ──────────────────────────────────────────────────
        elif current_state == FSMState.TRACKING:
            # 1. An toàn mũi xe (Bumper collision guard < 0.16m):
            front_min_dist = perception.get("front_min_dist", float('inf'))
            if front_min_dist < 0.16:
                self.StopRobot()
                self.get_logger().warn_throttle(
                    1.0, f"⚠️ Mũi xe sát vật cản ({front_min_dist:.2f}m < 0.16m)! Tạm dừng chờ thoáng..."
                )
                return

            # 1.5. Kiểm tra tín hiệu YOLO Mũi Tên VÀNG (Cuối hàng 1):
            is_yellow_trigger = False
            is_yellow_visible = False
            marker_dist = -1.0
            marker_conf = 0.0
            arrow_steer_deg = 0.0
            if external_cnn is not None:
                m_cls = external_cnn.get('marker_class', -1.0)
                marker_dist = external_cnn.get('marker_dist', -1.0)
                marker_conf = external_cnn.get('marker_conf', 0.0)
                arrow_steer_deg = float(external_cnn.get('arrow_steer_deg', 0.0))
                is_yellow_trigger = (
                    external_cnn.get('trigger_yellow', 0.0) >= 1.0
                    or (external_cnn.get('turn_trigger', 0.0) >= 1.0 and m_cls == 0.0)
                )
                if m_cls == 0.0 and marker_conf >= 0.35 and 0.05 < marker_dist <= 2.40:
                    is_yellow_visible = True

            # Nếu đang ở Luống 1 và YOLO báo đã tới Mũi Tên Vàng (<= 1.0m) -> Kích hoạt xoay 90° lần 1 ngay lập tức!
            if is_yellow_trigger and self.current_lane_idx == 1:
                self.get_logger().info(
                    f"🎯 [YOLO MỐC VÀNG TRIGGER] Đã tiến tới MŨI TÊN VÀNG cuối hàng 1 (cự ly={marker_dist:.2f}m <= 1.0m, Conf: {marker_conf*100:.1f}%)! "
                    f"Kích hoạt quay đầu 90°-90° (PIVOT_1 -> CROSS_DRIVE -> PIVOT_2)..."
                )
                if self.enable_file_logging and self.telemetry_logger:
                    self.telemetry_logger.log_event(
                        "YELLOW_ARROW_UTURN_TRIGGER",
                        f"Mũi tên vàng cự ly={marker_dist:.2f}m <= 1.0m (Conf={marker_conf*100:.1f}%) tại x={self.current_x:.2f}m"
                    )
                self.uturn_triggered_by_yellow = True
                self.is_adjusting_heading = False
                self.heading_adjust_stage = None
                self._current_heading_adjust_w = 0.0
                self.eor_detected = False
                self.low_confidence_counter = 0
                self.eor_low_conf_frames = 0
                self.smoothed_angle_deg = 0.0
                self.inside_row = False
                self.row_completed = False
                self.accumulated_turn_angle = 0.0
                self.StopRobot()
                self.transition_to_state(FSMState.UTURN_PLANNING, now)
                return

            now_sec = now.nanoseconds / 1e9
            in_row_max = float(getattr(self, 'in_row_max_steer_deg', 6.5))
            max_yaw_dev = float(getattr(self, 'in_row_max_yaw_dev_deg', 7.0))
            failed = (confidence < self.low_confidence_threshold)

            # ── ƯU TIÊN DẪN ĐƯỜNG KHI HẾT HÀNG / TIẾP CẬN MŨI TÊN VÀNG ──
            # Khi không còn thấy 2 hàng thùng (failed: conf < 30%) HOẶC đang tiến gần Mũi Tên Vàng cuối hàng:
            # Tuyệt đối KHÔNG bẻ lái trái/phải theo 1 thùng lẻ hay Global Plan Assist!
            # Chỉ nghe theo góc hướng tâm Mũi Tên Vàng của YOLO hoặc chạy thẳng 0.0°!
            if failed or (is_yellow_visible and confidence < 0.65):
                if self.is_adjusting_heading:
                    self.is_adjusting_heading = False
                    self.heading_adjust_stage = None
                    self._current_heading_adjust_w = 0.0
                self.steer_active_side = None
                self.steer_trigger_counter = 0
                self.turn_in_place_trigger_count = 0

                if is_yellow_visible:
                    self.smoothed_angle_deg = float(np.clip(arrow_steer_deg, -4.5, 4.5))
                else:
                    self.smoothed_angle_deg = 0.0

            # ── KHI ĐANG NHÌN THẤY 2 HÀNG THÙNG RÕ RÀNG (confidence >= 30%) ──
            elif not getattr(self, 'initial_alignment_done', False):
                effective_alpha = self.ema_alpha
                self.smoothed_angle_deg = (
                    effective_alpha * raw_angle
                    + (1.0 - effective_alpha) * self.smoothed_angle_deg
                )

                if (abs(self.smoothed_angle_deg) <= 1.5) and (confidence >= self.low_confidence_threshold):
                    self._entry_align_counter = getattr(self, '_entry_align_counter', 0) + 1
                else:
                    self._entry_align_counter = 0

                if (
                    getattr(self, '_entry_align_counter', 0) >= 3
                    or (self.distance_traveled >= 0.35 and confidence >= self.low_confidence_threshold)
                ):
                    self.initial_alignment_done = True
                    self.row_reference_yaw = float(self.current_yaw)
                    self._raw_angle_history = [float(self.smoothed_angle_deg)]
                    self.get_logger().info(
                        f"🚀 [ĐẦU HÀNG ĐÃ THẲNG] Đã căn chỉnh góc xuất phát thành công! "
                        f"(Khóa góc tim luống ref={math.degrees(self.row_reference_yaw):+.1f}°)."
                    )

            elif self.inside_row:
                ref_yaw = getattr(self, 'row_reference_yaw', None)
                if ref_yaw is None:
                    self.row_reference_yaw = float(self.current_yaw)
                    ref_yaw = self.row_reference_yaw

                yaw_dev = normalize_angle(float(self.current_yaw) - ref_yaw)
                yaw_dev_deg = math.degrees(yaw_dev)

                hist = getattr(self, '_raw_angle_history', [])
                hist.append(float(raw_angle))
                if len(hist) > 3:
                    hist = hist[-3:]
                self._raw_angle_history = hist
                median_angle = float(np.median(hist))

                is_contradicting_yaw = (abs(yaw_dev_deg) > 2.2) and (raw_angle * yaw_dev_deg < -3.5) and (abs(raw_angle) > 1.8)
                is_sudden_reversal_jump = (abs(raw_angle - self.smoothed_angle_deg) > 6.5) and (raw_angle * self.smoothed_angle_deg < -1.0)
                is_angle_spike = is_contradicting_yaw or is_sudden_reversal_jump

                if is_angle_spike:
                    if abs(yaw_dev_deg) > 1.0:
                        restoring_steer = float(np.clip(0.85 * yaw_dev_deg, -in_row_max, in_row_max))
                    else:
                        restoring_steer = float(np.clip(median_angle * 0.5, -in_row_max, in_row_max))

                    self.smoothed_angle_deg = 0.55 * restoring_steer + 0.45 * self.smoothed_angle_deg
                    self.smoothed_angle_deg = float(np.clip(self.smoothed_angle_deg, -in_row_max, in_row_max))
                else:
                    clamped_angle = float(np.clip(median_angle, -in_row_max, in_row_max))
                    self.smoothed_angle_deg = (
                        self.ema_alpha * clamped_angle
                        + (1.0 - self.ema_alpha) * self.smoothed_angle_deg
                    )
                    self.smoothed_angle_deg = float(np.clip(self.smoothed_angle_deg, -in_row_max, in_row_max))

                # Khóa cứng độ lệch góc thân xe chỉ khi đang thấy hàng rõ (confidence >= 0.45):
                if confidence >= 0.45:
                    if yaw_dev_deg > max_yaw_dev:
                        self.smoothed_angle_deg = max(self.smoothed_angle_deg, 2.5)
                    elif yaw_dev_deg < -max_yaw_dev:
                        self.smoothed_angle_deg = min(self.smoothed_angle_deg, -2.5)

            else:
                self.smoothed_angle_deg = (
                    self.ema_alpha * raw_angle
                    + (1.0 - self.ema_alpha) * self.smoothed_angle_deg
                )

            # Cập nhật controller nội bộ
            now_sec = now.nanoseconds / 1e9
            if hasattr(self, '_prev_image_time'):
                dt_actual = now_sec - self._prev_image_time
            else:
                dt_actual = 0.033
            self._prev_image_time = now_sec
            dt_actual = np.clip(dt_actual, 0.001, 1.0)
            
            _lin_smc, _ang_smc = self.StartTracking(dt_actual)

            # ── LOGIC ĐIỀU HƯỚNG CNN BÁM LUỐNG (TRACKING) ──
            if getattr(self, 'tracking_steer_mode', 'PIVOT_STOP') == 'CONTINUOUS_STEER':
                # =========================================================================
                # CHẾ ĐỘ ĐIỀU HƯỚNG LIÊN TỤC (CONTINUOUS STEER — KHÔNG DỪNG XE)
                # =========================================================================
                steer_trigger = float(getattr(self, 'steer_trigger_deg', 2.0))
                steer_trigger_frames = int(getattr(self, 'steer_trigger_frames', 3))
                steer_resume = float(getattr(self, 'steer_resume_deg', 1.2))
                required_straight_frames = int(getattr(self, 'steer_aligned_frames', 2))
                v_nom = float(self.linear_speed)
                v_boost = float(getattr(self, 'steer_boost_speed', 0.155))
                v_brake = max(0.050, float(getattr(self, 'steer_brake_speed', 0.055)))
                ramp_t = max(0.05, float(getattr(self, 'steer_ramp_time', 0.12)))

                cur_angle = float(self.smoothed_angle_deg)

                if failed and not is_yellow_visible:
                    self.steer_active_side = None
                    self.steer_trigger_counter = 0
                    self.steer_candidate_side = None
                elif self.steer_active_side is None:
                    if cur_angle > steer_trigger:
                        if getattr(self, 'steer_candidate_side', None) == 'RIGHT':
                            self.steer_trigger_counter = getattr(self, 'steer_trigger_counter', 0) + 1
                        else:
                            self.steer_candidate_side = 'RIGHT'
                            self.steer_trigger_counter = 1

                        if self.steer_trigger_counter >= steer_trigger_frames:
                            self.steer_active_side = 'RIGHT'
                            self.steer_straight_frames = 0
                            self.steer_trigger_counter = 0
                            self.steer_candidate_side = None
                            self.get_logger().info(
                                f"🌾 [ĐÁNH LÁI LIÊN TỤC] Lệch Phải ({cur_angle:+.2f}° > {steer_trigger}°): "
                                f"Bánh Phải {v_brake:.3f} m/s, Bánh Trái {v_boost:.3f} m/s (liền mạch)!"
                            )
                    elif cur_angle < -steer_trigger:
                        if getattr(self, 'steer_candidate_side', None) == 'LEFT':
                            self.steer_trigger_counter = getattr(self, 'steer_trigger_counter', 0) + 1
                        else:
                            self.steer_candidate_side = 'LEFT'
                            self.steer_trigger_counter = 1

                        if self.steer_trigger_counter >= steer_trigger_frames:
                            self.steer_active_side = 'LEFT'
                            self.steer_straight_frames = 0
                            self.steer_trigger_counter = 0
                            self.steer_candidate_side = None
                            self.get_logger().info(
                                f"🌾 [ĐÁNH LÁI LIÊN TỤC] Lệch Trái ({cur_angle:+.2f}° < -{steer_trigger}°): "
                                f"Bánh Trái {v_brake:.3f} m/s, Bánh Phải {v_boost:.3f} m/s (liền mạch)!"
                            )
                    else:
                        self.steer_trigger_counter = 0
                        self.steer_candidate_side = None
                else:
                    if abs(cur_angle) < steer_resume:
                        self.steer_straight_frames += 1
                        if self.steer_straight_frames >= required_straight_frames:
                            self.get_logger().info(
                                f"✅ [ĐÁNH LÁI LIÊN TỤC] ĐÃ THẲNG HÀNG (|góc|={abs(cur_angle):.2f}° < {steer_resume}°) -> 4 bánh đều {v_nom:.3f} m/s."
                            )
                            self.steer_active_side = None
                            self.steer_straight_frames = 0
                            self.steer_trigger_counter = 0
                            self.steer_candidate_side = None
                    else:
                        self.steer_straight_frames = 0
                        if self.steer_active_side == 'RIGHT' and cur_angle < -steer_trigger:
                            self.steer_active_side = 'LEFT'
                            self.steer_straight_frames = 0
                        elif self.steer_active_side == 'LEFT' and cur_angle > steer_trigger:
                            self.steer_active_side = 'RIGHT'
                            self.steer_straight_frames = 0

                if self.steer_active_side == 'RIGHT':
                    target_v_l = v_boost
                    target_v_r = v_brake
                elif self.steer_active_side == 'LEFT':
                    target_v_l = v_brake
                    target_v_r = v_boost
                else:
                    target_v_l = v_nom
                    target_v_r = v_nom

                max_dv = (max(v_nom, v_boost) / ramp_t) * dt_actual
                dv_l = target_v_l - getattr(self, 'steer_current_v_l', v_nom)
                dv_r = target_v_r - getattr(self, 'steer_current_v_r', v_nom)
                self.steer_current_v_l = float(getattr(self, 'steer_current_v_l', v_nom) + np.clip(dv_l, -max_dv, max_dv))
                self.steer_current_v_r = float(getattr(self, 'steer_current_v_r', v_nom) + np.clip(dv_r, -max_dv, max_dv))

                track_w = float(getattr(self, 'wheel_base', 0.58))
                twist = Twist()
                twist.linear.x = (self.steer_current_v_l + self.steer_current_v_r) / 2.0
                twist.angular.z = (self.steer_current_v_r - self.steer_current_v_l) / track_w
                self._latest_twist = twist

            else:
                # =========================================================================
                # CHẾ ĐỘ BÁM LUỐNG MƯỢT MÀ + XOAY DỨT KHOÁT KHI LỆCH LỚN (> 3.5°)
                # Đã loại bỏ hoàn toàn BRAKE_DECEL / SETTLE / fwd_ramp gây kêu hộp số!
                # =========================================================================
                stop_threshold = float(getattr(self, 'turn_in_place_threshold_deg', 3.5))
                resume_threshold = float(getattr(self, 'turn_in_place_resume_deg', 1.5))

                if self.is_adjusting_heading:
                    required_aligned_frames = int(getattr(self, 'heading_adjust_aligned_frames', 2))
                    is_frame_aligned = (confidence >= self.low_confidence_threshold) and (abs(self.smoothed_angle_deg) <= resume_threshold)

                    if is_frame_aligned:
                        self._aligned_frame_count += 1
                        if self._aligned_frame_count >= required_aligned_frames:
                            self.is_adjusting_heading = False
                            self.heading_adjust_stage = None
                            self._current_heading_adjust_w = 0.0
                            self._heading_boost_w = 0.0
                            self._aligned_frame_count = 0
                            self.heading_adjust_cooldown_until = now_sec + 1.5
                            self.forward_resume_start_time = now_sec
                            if not getattr(self, 'initial_alignment_done', False):
                                self.initial_alignment_done = True
                                self.row_reference_yaw = float(self.current_yaw)
                            self.get_logger().info(
                                f"✅ [ĐIỀU HƯỚNG CNN] ĐÃ THẲNG HÀNG (|góc CNN|={abs(self.smoothed_angle_deg):.2f}° <= {resume_threshold:.2f}°) "
                                f"-> Tiếp tục chạy thẳng liền mạch {self.linear_speed:.3f} m/s!"
                            )
                    elif failed or is_yellow_visible:
                        self.is_adjusting_heading = False
                        self.heading_adjust_stage = None
                        self._current_heading_adjust_w = 0.0
                        self._heading_boost_w = 0.0
                        self._aligned_frame_count = 0
                else:
                    can_trigger = (
                        (now_sec >= getattr(self, 'heading_adjust_cooldown_until', 0.0))
                        and (confidence >= max(0.38, self.low_confidence_threshold))
                        and (not is_yellow_visible)
                    )
                    required_trigger_frames = int(getattr(self, 'turn_in_place_trigger_frames', 3))
                    if can_trigger and abs(self.smoothed_angle_deg) > stop_threshold:
                        self.turn_in_place_trigger_count = getattr(self, 'turn_in_place_trigger_count', 0) + 1
                        if self.turn_in_place_trigger_count >= required_trigger_frames:
                            self.turn_in_place_trigger_count = 0
                            self.is_adjusting_heading = True
                            self.heading_adjust_stage = 'ROTATING'
                            self.heading_adjust_stage_start = now_sec
                            self.heading_adjust_start_time = now_sec
                            self.heading_adjust_start_imu_yaw = float(self.localization_manager.imu_yaw)
                            self.heading_adjust_target_deg = max(2.0, min(8.0, abs(self.smoothed_angle_deg) * 0.85))
                            self.heading_adjust_dir = -1.0 if self.smoothed_angle_deg > 0 else 1.0
                            self._current_heading_adjust_w = 0.0
                            self._heading_boost_w = 0.0
                            self._aligned_frame_count = 0

                            self.get_logger().info(
                                f"🌾 [CHỈNH GÓC DỨT KHOÁT] Góc lệch {self.smoothed_angle_deg:+.2f}° > {stop_threshold:.2f}° "
                                f"-> Xoay nhanh {self.heading_adjust_target_deg:.1f}° theo IMU về tim luống!"
                            )
                    else:
                        self.turn_in_place_trigger_count = 0

                if self.is_adjusting_heading:
                    lin_speed = getattr(self, '_latest_twist', Twist()).linear.x
                    ang_vel = getattr(self, '_latest_twist', Twist()).angular.z
                else:
                    # Chạy thẳng mượt mà liên tục ở vận tốc danh định, kết hợp lái mềm SMC khi thấy hàng hoặc biển vàng:
                    lin_speed = float(self.linear_speed)
                    if not failed or is_yellow_visible:
                        ang_vel = float(np.clip(_ang_smc, -0.35, 0.35))
                    else:
                        ang_vel = 0.0

                twist.linear.x = lin_speed
                twist.angular.z = ang_vel
                self._latest_twist = twist

            # ── Pure Perception & Mission Goals End-Of-Row Check ─────────────────────
            if confidence < self.low_confidence_threshold:
                self.eor_low_conf_frames += 1
            else:
                self.eor_low_conf_frames = max(0, self.eor_low_conf_frames - 2)
            trigger_confidence = (self.eor_low_conf_frames >= self.low_conf_frames_threshold)
            definite_vision_eor = (self.eor_low_conf_frames >= (self.low_conf_frames_threshold + 4)) or (
                trigger_confidence and confidence < 0.18
            )

            left_side_dist = getattr(self, '_latest_left_side_dist', float('inf'))
            right_side_dist = getattr(self, '_latest_right_side_dist', float('inf'))
            sides_cleared_eor = (left_side_dist > 0.38 and right_side_dist > 0.38) or (
                definite_vision_eor and max(left_side_dist, right_side_dist) > 0.42
            )
            perception_eor = (trigger_confidence or end_of_row) and sides_cleared_eor

            # 3. Điều phối theo Điểm Mốc Nhiệm Vụ (Mission Goals):
            if self.enable_mission_goals and warmup_done:
                if self.current_lane_idx == 1:
                    # Đang chạy Luống 1:
                    # TUYỆT ĐỐI KHÔNG tự ý quay đầu trong hàng khi hết thấy thùng (perception_eor)!
                    # Khi hết thấy thùng ở cuối Luống 1, xe tiếp tục tiến thẳng ra khỏi hàng cho tới khi
                    # YOLO phát hiện MŨI TÊN VÀNG ở cự ly <= 1.0m (xử lý ở khối is_yellow_trigger phía trên)
                    # hoặc vượt quá giới hạn an toàn max_row_length (25.0m).
                    max_limit_dist = getattr(self, 'max_row_length', 25.0)
                    if self.distance_traveled >= max_limit_dist:
                        self.get_logger().warn(
                            f"⚠️ [GIỚI HẠN AN TOÀN LUỐNG 1] Đã chạy {self.distance_traveled:.2f}m >= {max_limit_dist:.2f}m -> Kích hoạt U-Turn!"
                        )
                        self.uturn_triggered_by_yellow = False
                        self.is_adjusting_heading = False
                        self.heading_adjust_stage = None
                        self._current_heading_adjust_w = 0.0
                        self.eor_detected = False
                        self.low_confidence_counter = 0
                        self.eor_low_conf_frames = 0
                        self.smoothed_angle_deg = 0.0
                        self.inside_row = False
                        self.row_completed = False
                        self.accumulated_turn_angle = 0.0
                        self.StopRobot()
                        self.transition_to_state(FSMState.UTURN_PLANNING, now)
                        return

                elif self.current_lane_idx == 2:
                    # Đang chạy Luống 2: chạy cho đến khi CNN không còn thấy hàng cây nữa
                    max_limit_dist = getattr(self, 'max_row_length', 25.0)
                    exceeded_max_limit = (self.distance_traveled >= max_limit_dist)
                    cnn_still_confident = (confidence >= self.low_confidence_threshold)

                    can_finish = False
                    finish_reason = ""

                    if cnn_still_confident and not exceeded_max_limit:
                        pass
                    else:
                        if perception_eor and self.distance_traveled >= 0.60:
                            can_finish = True
                            finish_reason = f"CNN & LiDAR xác nhận hết hàng Luống 2 (conf < {self.low_confidence_threshold*100:.0f}%, dist={self.distance_traveled:.2f}m)"
                        elif exceeded_max_limit:
                            can_finish = True
                            finish_reason = f"Đạt giới hạn quãng đường tối đa Luống 2 (dist={self.distance_traveled:.2f}m >= {max_limit_dist:.2f}m)"

                    if can_finish:
                        self.mission_completed = True
                        self.StopRobot()
                        self.get_logger().info(
                            f"🏆 [GOAL 3 - HOÀN THÀNH NHIỆM VỤ] {finish_reason}! Vị trí: x={self.current_x:.2f}m, y={self.current_y:.2f}m. Dừng xe an toàn tuyệt đối."
                        )
                        if self.enable_file_logging and self.telemetry_logger:
                            self.telemetry_logger.log_event("MISSION_COMPLETE", f"{finish_reason} tại x={self.current_x:.2f}m, y={self.current_y:.2f}m")
                        self.transition_to_state(FSMState.IDLE, now)
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
            if getattr(self, 'uturn_mode', 'PIVOT').upper() == 'PIVOT':
                self.pivot_stage = 'EXIT_ROW'
                self.pivot_stage_start_time = now_sec
                self.pivot_settle_until = 0.0
                self.pivot_start_x = self.current_x
                self.pivot_start_y = self.current_y
                self.pivot_start_yaw = float(self.current_yaw)
                self.pivot_turn_dir = -1.0 if self.turn_side.upper() == 'RIGHT' else 1.0
                self._has_seen_black_arrow = False

                # Target 1: xoay 90 độ sang luống kế bên (vuông góc hàng)
                self.pivot_target_yaw_1 = normalize_angle(self.pivot_start_yaw + self.pivot_turn_dir * (math.pi / 2.0))
                # Target 2: xoay tiếp 90 độ (tổng 180 độ so với Luống 1) để khóa vào Luống 2
                self.pivot_target_yaw_2 = normalize_angle(self.pivot_start_yaw + math.pi)

                self.get_logger().info(
                    f"🔄 [PIVOT U-TURN] Kích hoạt quay đầu chữ U 90°-90° (IMU + YOLO):\n"
                    f"   📍 Xuất phát: x={self.pivot_start_x:.2f}m, y={self.pivot_start_y:.2f}m, EKF Yaw={math.degrees(self.pivot_start_yaw):+.1f}°\n"
                    f"   🎯 Hướng ngang Target 1 (90°): {math.degrees(self.pivot_target_yaw_1):+.1f}° (quay {self.turn_side})\n"
                    f"   🎯 Hướng Luống 2 Target 2 (180°): {math.degrees(self.pivot_target_yaw_2):+.1f}°"
                )
                if self.enable_file_logging and self.telemetry_logger:
                    self.telemetry_logger.log_event(
                        "PIVOT_UTURN_START",
                        f"Bắt đầu Pivot U-Turn: EKF Yaw={math.degrees(self.pivot_start_yaw):.1f}°, Target1={math.degrees(self.pivot_target_yaw_1):.1f}°, Target2={math.degrees(self.pivot_target_yaw_2):.1f}°"
                    )
                self.StopRobot()
                self.transition_to_state(FSMState.UTURN_EXECUTION, now)
                return

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
            if getattr(self, 'uturn_mode', 'PIVOT').upper() == 'PIVOT':
                # Pivot U-Turn 4 giai đoạn được điều phối liên tục và an toàn bởi self._pivot_uturn_timer_callback (20Hz).
                # Không phát lệnh lái từ image_callback nhưng tiếp tục để cập nhật telemetry cache và confidence.
                pass
            else:
                # Use forward velocity so it traces a smooth wider arc into the row
                twist.linear.x  = self.turn_linear_speed
                twist.angular.z = self.turn_direction * self.turn_angular_speed
                
                turn_angle_deg = np.rad2deg(self.accumulated_turn_angle)
                
                # Trao quyền sớm cho CNN bắt luống khi đã quay đủ góc (> 130 độ):
                if turn_angle_deg >= self.min_turn_angle_deg:
                    if confidence >= self.high_confidence_threshold:
                        self.u_turn_see_row_frames = getattr(self, 'u_turn_see_row_frames', 0) + 1
                        if self.u_turn_see_row_frames >= getattr(self, 'low_conf_frames_threshold', 15):
                            self.get_logger().info(
                                f"--- New crop row caught in UTURN_EXECUTION (conf={confidence:.2f}, turn={turn_angle_deg:.1f}°) → TRACKING ---"
                            )
                            self.StopRobot()
                            self.transition_after_path(now)
                            self.u_turn_see_row_frames = 0
                    else:
                        self.u_turn_see_row_frames = 0

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

        # Khi đang trong chu kỳ căn chỉnh góc hoặc Pivot U-Turn, cmd_vel và _latest_twist do timer 20Hz kiểm soát
        is_pivot_active = (current_state == FSMState.UTURN_EXECUTION and getattr(self, 'uturn_mode', 'PIVOT').upper() == 'PIVOT')
        if not self.is_adjusting_heading and not is_pivot_active:
            self.cmd_vel_pub.publish(twist)
            self._latest_twist = twist

        # ── State cache update for 10Hz Telemetry Logger & 1Hz Terminal Status ──
        pose_info = self.localization_manager.get_pose()
        gps_info = pose_info.get('gps', {})
        display_steer_deg = self.smoothed_angle_deg
        active_twist = getattr(self, '_latest_twist', twist)
        if current_state != FSMState.TRACKING and abs(active_twist.linear.x) > 0.01:
            display_steer_deg = math.degrees(math.atan2(active_twist.angular.z * 0.58, active_twist.linear.x))

        self._last_image_time = time.time()
        self._latest_confidence = confidence
        self._latest_steer_deg = display_steer_deg
        self._latest_gps_info = gps_info
        self._latest_inf_ms = getattr(self, '_last_inference_ms', 0.0)
        self._latest_raw_angle = raw_angle
        self._latest_lane_offset = lane_offset

    def _telemetry_timer_callback(self):
        """
        Ghi nhận dữ liệu telemetry định kỳ 10Hz (100ms) ra file CSV.
        Đảm bảo biểu đồ tốc độ (Linear & Angular Velocity) và quỹ đạo luôn dày đặc, liên tục,
        ngay cả khi xoay tại chỗ (Heading Adjust), quay đầu (U-Turn) hoặc khi camera trả frame thưa.
        """
        if not self.enable_file_logging or not self.telemetry_logger:
            return

        # Chỉ bắt đầu ghi sau khi xe đã nhận tín hiệu đầu tiên
        if self._last_image_time == 0.0:
            return

        current_state = self.fsm.get_state()
        if current_state == FSMState.IDLE and self.distance_traveled == 0.0:
            return

        if not getattr(self, '_is_stabilized', False):
            display_state = "STABILIZING"
            cmd_lin = 0.0
            cmd_ang = 0.0
        elif self.is_adjusting_heading:
            display_state = "HEADING_ADJUST"
            cmd_lin = 0.0
            cmd_ang = getattr(self, '_current_heading_adjust_w', 0.0)
        elif current_state == FSMState.TRACKING and getattr(self, 'tracking_steer_mode', 'PIVOT_STOP') == 'CONTINUOUS_STEER':
            active_side = getattr(self, 'steer_active_side', None)
            if active_side == 'RIGHT':
                display_state = "STEER_RIGHT"
            elif active_side == 'LEFT':
                display_state = "STEER_LEFT"
            else:
                display_state = "TRACKING_STRAIGHT"
            cmd_lin = getattr(self, '_latest_twist', Twist()).linear.x
            cmd_ang = getattr(self, '_latest_twist', Twist()).angular.z
        elif current_state == FSMState.UTURN_EXECUTION and getattr(self, 'uturn_mode', 'PIVOT').upper() == 'PIVOT':
            display_state = f"UTURN_{getattr(self, 'pivot_stage', 'EXEC')}"
            cmd_lin = getattr(self, '_latest_twist', Twist()).linear.x
            cmd_ang = getattr(self, '_latest_twist', Twist()).angular.z
        else:
            display_state = str(current_state)
            cmd_lin = getattr(self, '_latest_twist', Twist()).linear.x
            cmd_ang = getattr(self, '_latest_twist', Twist()).angular.z

        pose_info = self.localization_manager.get_pose()
        gps_info = pose_info.get('gps', {})
        imu_info = pose_info.get('imu', {})
        inf_ms = getattr(self, '_last_inference_ms', 0.0)
        fps_val = (1000.0 / inf_ms) if inf_ms > 0 else 0.0

        self.telemetry_logger.log_telemetry({
            'fsm_state': display_state,
            'x': self.current_x,
            'y': self.current_y,
            'yaw': self.current_yaw,
            'steering_angle_deg': self.smoothed_angle_deg,
            'raw_steer_deg': getattr(self, '_latest_raw_angle', 0.0),
            'lane_offset': getattr(self, '_latest_lane_offset', 0.0),
            'linear_velocity': cmd_lin,
            'angular_velocity': cmd_ang,
            'act_linear_velocity': pose_info.get('linear_velocity', 0.0),
            'act_angular_velocity': pose_info.get('angular_velocity', 0.0),
            'imu_yaw': imu_info.get('yaw', 0.0),
            'imu_angular_vel_z': imu_info.get('angular_vel_z', 0.0),
            'imu_accel_x': imu_info.get('linear_accel_x', 0.0),
            'confidence': getattr(self, '_latest_confidence', 0.0),
            'inference_ms': inf_ms,
            'fps': fps_val,
            'distance_traveled': self.distance_traveled,
            'gps_latitude': gps_info.get('latitude', 0.0),
            'gps_longitude': gps_info.get('longitude', 0.0),
            'gps_altitude': gps_info.get('altitude', 0.0),
            'gps_dms': gps_info.get('dms', ''),
            'gps_status': gps_info.get('status', 'NO_FIX')
        })

    def status_timer_callback(self):
        """Định kỳ in trạng thái trực quan chuẩn 1 Hz ra terminal khi đang tự hành."""
        now_sec = time.time()
        current_state = self.fsm.get_state()

        # 1. Chưa nhận được tín hiệu nào từ lúc bật: chỉ thông báo 1 lần, KHÔNG spam mỗi giây
        if self._last_image_time == 0.0:
            if not getattr(self, '_waiting_camera_notified', False):
                self.get_logger().info("⏳ Đang chờ nhận hình ảnh từ Camera hoặc góc lái AI từ Laptop (/crop_row/detection)...")
                self._waiting_camera_notified = True
            return

        # 2. Bị mất tín hiệu camera / laptop giữa chừng quá 6 giây (và không đang xoay U-Turn độc lập bằng IMU):
        is_pivot_active = (current_state == FSMState.UTURN_EXECUTION and getattr(self, 'uturn_mode', 'PIVOT').upper() == 'PIVOT')
        if (now_sec - self._last_image_time) > 6.0 and not is_pivot_active:
            if not getattr(self, '_camera_lost_notified', False):
                self.get_logger().warn("⚠️ Mất tín hiệu Camera / Laptop quá 6 giây! Xe đang tạm dừng.")
                self._camera_lost_notified = True
                self.StopRobot()
            return

        self._camera_lost_notified = False

        # Không in telemetry định kỳ 1Hz khi đang trong giai đoạn đếm ngược ổn định 3s
        if not getattr(self, '_is_stabilized', False):
            return

        # 3. Khi xe đang hoạt động bình thường: in 1 dòng chuẩn, súc tích, cực kỳ dễ đọc
        inf_ms = self._latest_inf_ms
        fps_val = (1000.0 / inf_ms) if inf_ms > 0 else 0.0
        gps_status = self._latest_gps_info.get('status', 'NO_FIX')
        gps_source = self._latest_gps_info.get('source', '')
        gps_str = f" | GPS: {self._latest_gps_info.get('latitude', 0.0):.5f}°, {self._latest_gps_info.get('longitude', 0.0):.5f}°" if (gps_status == 'FIX' and gps_source == 'DIRECT_SENSOR') else ""

        if self.is_adjusting_heading:
            display_state_str = "CHỈNH GÓC"
        elif current_state == FSMState.TRACKING and getattr(self, 'tracking_steer_mode', 'PIVOT_STOP') == 'CONTINUOUS_STEER':
            active_side = getattr(self, 'steer_active_side', None)
            if active_side == 'RIGHT':
                display_state_str = "BẺ PHẢI"
            elif active_side == 'LEFT':
                display_state_str = "BẺ TRÁI"
            else:
                display_state_str = "TIẾN THẲNG"
        elif current_state == FSMState.UTURN_EXECUTION and getattr(self, 'uturn_mode', 'PIVOT').upper() == 'PIVOT':
            display_state_str = f"U-TURN {getattr(self, 'pivot_stage', '')}"
        else:
            display_state_str = str(current_state)
        source_tag = "AI LAPTOP ➔ PI" if getattr(self, '_is_remote_perception', False) else "AI LÁI XE"
        status_msg = (
            f"🌾 [{source_tag}] Luống {self.current_lane_idx} (x={self.current_x:4.2f}m) | "
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
        self._scan_received = True
        self._last_scan_time = time.time()
        self.lidar_processor.update_scan(msg)
        if hasattr(self.lidar_processor, 'get_min_range_in_sector'):
            self._latest_left_side_dist = self.lidar_processor.get_min_range_in_sector(35.0, 85.0)
            self._latest_right_side_dist = self.lidar_processor.get_min_range_in_sector(-85.0, -35.0)

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
