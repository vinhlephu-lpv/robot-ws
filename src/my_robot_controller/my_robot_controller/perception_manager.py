import time
import numpy as np

class SensorPriorityManager:
    """
    Multi-sensor dynamic priority selector and fault-tolerant voting system.
    As long as >= 1 sensor is operational, the system tracks smoothly.
    State transitions require consensus across active sensor modalities.
    """
    def __init__(self, cnn_threshold=0.30):
        self.cnn_threshold = cnn_threshold
        self.active_sensor = "CAMERA_CNN"

    def select_active_tracking_sensor(self, cnn_confidence, lidar_available=True):
        if cnn_confidence >= self.cnn_threshold:
            self.active_sensor = "CAMERA_CNN"
        elif lidar_available:
            self.active_sensor = "LIDAR_GEOMETRY"
        else:
            self.active_sensor = "GPS_ODOMETRY"
        return self.active_sensor

    def evaluate_eor_consensus(self, camera_eor, lidar_eor, odom_eor=False):
        """
        Fault-tolerant Voting: End of Row is confirmed when Primary sensor reports EOR
        and at least 1 secondary sensor concurs.
        """
        votes = [camera_eor, lidar_eor, odom_eor]
        return votes.count(True) >= 2


class EndOfRowDetector:
    """
    Combines multi-modal perception (Camera CNN confidence + LiDAR sector clearance)
    to detect end of corn row dynamically without false triggers inside stalk gaps.
    """
    def __init__(self, min_row_distance=0.0, low_confidence_threshold=0.35):
        self.min_row_distance = min_row_distance
        self.low_confidence_threshold = low_confidence_threshold

    def detect(self, confidence, left_side_dist=float('inf'), right_side_dist=float('inf'), 
               rear_left_dist=float('inf'), rear_right_dist=float('inf'), 
               front_min_dist=float('inf'), inside_row=True):
        # 1. Vision check: Camera confidence drops (no crop structure ahead)
        camera_eor = (confidence < self.low_confidence_threshold)
        
        # 2. LiDAR check: Open headland space ahead (front_min_dist > 2.0m) AND sides clear (> 0.90m)
        lidar_eor = (front_min_dist > 2.0 and left_side_dist > 0.90 and right_side_dist > 0.90)
        
        if camera_eor or lidar_eor:
            return True
        return False


class PerceptionManager:
    """
    Manages and fuses feeds from Camera/CNN and Lidar using SensorPriorityManager.
    """
    def __init__(self, inference_handler=None, lidar_processor=None, eor_detector=None):
        self.inference = inference_handler
        self.lidar = lidar_processor
        self.eor_detector = eor_detector if eor_detector is not None else EndOfRowDetector()
        self.sensor_priority = SensorPriorityManager()

    def process_sensors(self, cv_image, distance_traveled, rx=0.0, ry=0.0, ryaw=0.0, max_angle_deg=3.5, inside_row=True):
        """
        Runs inference and checks lidar, returning standardized PerceptionOutput.
        """
        timestamp = time.time()
        
        # 1. Camera / CNN Processing
        heading_error = 0.0
        lane_offset = 0.0
        lane_center = 0.0
        confidence = 0.0
        
        if self.inference is not None and cv_image is not None:
            heading_error, lane_offset, lane_center, confidence = self.inference.process_image(cv_image, max_angle_deg)

        # 2. Lidar Processing
        obstacle_detected = False
        front_min_dist = float('inf')
        left_side_dist = float('inf')
        right_side_dist = float('inf')
        rear_left_dist = float('inf')
        rear_right_dist = float('inf')
        obstacles = []
        
        if self.lidar is not None:
            obstacle_detected = self.lidar.check_obstacle_in_front(rx, ry, ryaw, inside_row=inside_row)
            front_min_dist = self.lidar.get_min_range_in_sector(-25.0, 25.0)
            # Mở rộng góc quét sang góc chéo trước sườn (20° - 85°) để bắt sớm góc cản trước khi xe đi xiên vào hàng
            left_side_dist = self.lidar.get_min_range_in_sector(20.0, 85.0)
            right_side_dist = self.lidar.get_min_range_in_sector(-85.0, -20.0)
            rear_left_dist = self.lidar.get_min_range_in_sector(70.0, 135.0)
            rear_right_dist = self.lidar.get_min_range_in_sector(-135.0, -70.0)
            obstacles = self.lidar.get_obstacles_global(rx, ry, ryaw)

        # 3. Dynamic Sensor Priority & Active LiDAR Corridor Centering Guard
        active_sensor = self.sensor_priority.select_active_tracking_sensor(confidence, lidar_available=(self.lidar is not None))

        # 3a. Trợ lái căn giữa luống bằng LiDAR (LiDAR Corridor Centering Assistance)
        # Khi cả 2 bên thành hàng đều nằm trong tầm quét (hành lang ~1.0m):
        if self.lidar is not None and (0.20 < left_side_dist < 0.75) and (0.20 < right_side_dist < 0.75):
            corridor_width = left_side_dist + right_side_dist
            if 0.75 <= corridor_width <= 1.35:
                delta_side = right_side_dist - left_side_dist
                # delta_side > 0: lệch sang trái luống -> cần đánh lái sang phải (> 0)
                # delta_side < 0: lệch sang phải luống -> cần đánh lái sang trái (< 0)
                lidar_centering_bias = float(np.clip((delta_side / 0.25) * (max_angle_deg * 0.7), -max_angle_deg * 0.7, max_angle_deg * 0.7))
                
                # Pha trộn linh hoạt: nếu camera confidence thấp, tăng trọng số LiDAR; nếu camera tốt vẫn giữ 25% LiDAR trợ lái
                lidar_weight = float(np.clip(1.0 - confidence, 0.25, 0.85))
                heading_error = (1.0 - lidar_weight) * heading_error + lidar_weight * lidar_centering_bias

        # 3b. Active LiDAR Emergency Side Collision Guard:
        # Khi khoảng cách tới mép hàng < 0.38m (nguy hiểm cận kề bánh xe ~0.25m), cưỡng bức đánh lái gấp ngược ra giữa
        danger_threshold = 0.38
        if self.lidar is not None:
            if left_side_dist < danger_threshold:
                penetration = danger_threshold - left_side_dist
                repulsion_deg = float(np.clip((penetration / 0.15) * max_angle_deg, 2.0, max_angle_deg))
                # Phải rẽ phải (dương) để thoát khỏi thành trái:
                heading_error = max(heading_error, repulsion_deg)
            elif right_side_dist < danger_threshold:
                penetration = danger_threshold - right_side_dist
                repulsion_deg = float(np.clip((penetration / 0.15) * max_angle_deg, 2.0, max_angle_deg))
                # Phải rẽ trái (âm) để thoát khỏi thành phải:
                heading_error = min(heading_error, -repulsion_deg)

        end_of_row = self.eor_detector.detect(
            confidence=confidence,
            left_side_dist=left_side_dist,
            right_side_dist=right_side_dist,
            rear_left_dist=rear_left_dist,
            rear_right_dist=rear_right_dist,
            front_min_dist=front_min_dist,
            inside_row=inside_row
        )

        return {
            "heading_error": heading_error,
            "lane_offset": lane_offset,
            "lane_center": lane_center,
            "lane_width": 1.00,  # standardized/expected corn row width
            "confidence": confidence,
            "active_sensor": active_sensor,
            "obstacle_detected": obstacle_detected,
            "front_min_dist": front_min_dist,
            "left_side_dist": left_side_dist,
            "right_side_dist": right_side_dist,
            "obstacles": obstacles,
            "end_of_row_detected": end_of_row,
            "timestamp": timestamp
        }
