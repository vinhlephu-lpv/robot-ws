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
    def __init__(self, min_row_distance=2.0, low_confidence_threshold=0.30, consecutive_frames=15):
        self.min_row_distance = min_row_distance
        self.low_confidence_threshold = low_confidence_threshold
        self.consecutive_frames = consecutive_frames
        self.low_confidence_counter = 0
        self.lidar_clearance_counter = 0

    def detect(self, confidence, distance_traveled=0.0,
               left_side_dist=float('inf'), right_side_dist=float('inf'), 
               rear_left_dist=float('inf'), rear_right_dist=float('inf'), 
               front_min_dist=float('inf'), inside_row=True):
        if not inside_row:
            self.low_confidence_counter = 0
            self.lidar_clearance_counter = 0
            return False

        # 1. Vision check: Camera mất dấu luống bắp liên tục N frame (mặc định 15 frame cho laptop 12-15 FPS)
        if confidence < self.low_confidence_threshold:
            self.low_confidence_counter += 1
        else:
            self.low_confidence_counter = 0
        
        camera_eor = (self.low_confidence_counter >= self.consecutive_frames)
        
        # 2. LiDAR check: Kích hoạt khi phía trước trống (> 1.2m), hai bên sườn trống (> 0.60m)
        # VÀ Camera cũng mất dấu luống (confidence < low_confidence_threshold).
        if front_min_dist > 1.20 and left_side_dist > 0.60 and right_side_dist > 0.60 and (confidence < self.low_confidence_threshold):
            self.lidar_clearance_counter += 1
        else:
            self.lidar_clearance_counter = 0

        lidar_eor = (self.lidar_clearance_counter >= self.consecutive_frames)
        
        # 3. Yêu cầu an toàn thực địa: Chỉ cho phép nhận biết hết hàng khi ĐỒNG THỜI
        # LiDAR 2 bên sườn xác nhận khoảng trống (> 0.60m), chống kích hoạt non do thưa cây/bóng râm giữa luống:
        sides_cleared = (left_side_dist > 0.60 and right_side_dist > 0.60)
        
        if (camera_eor or lidar_eor) and sides_cleared:
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

    def process_sensors(self, cv_image, distance_traveled, rx=0.0, ry=0.0, ryaw=0.0, max_angle_deg=3.5, inside_row=True, external_cnn=None):
        """
        Runs inference and checks lidar, returning standardized PerceptionOutput.
        """
        timestamp = time.time()
        
        # 1. Camera / CNN Processing
        heading_error = 0.0
        lane_offset = 0.0
        lane_center = 0.0
        confidence = 0.0
        
        if external_cnn is not None:
            # Nhận trực tiếp kết quả tính toán AI từ Laptop (Offloading - giải phóng CPU Pi)
            heading_error = external_cnn.get('heading_error', 0.0)
            lane_offset = external_cnn.get('lane_offset', 0.0)
            lane_center = external_cnn.get('lane_center', 0.0)
            confidence = external_cnn.get('confidence', 0.0)
        elif self.inference is not None and cv_image is not None:
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
            front_min_dist = self.lidar.get_min_range_in_sector(-25.0, 25.0)
            left_side_dist = self.lidar.get_min_range_in_sector(20.0, 85.0)
            right_side_dist = self.lidar.get_min_range_in_sector(-85.0, -20.0)
            rear_left_dist = self.lidar.get_min_range_in_sector(70.0, 135.0)
            rear_right_dist = self.lidar.get_min_range_in_sector(-135.0, -70.0)
            obstacles = self.lidar.get_obstacles_global(rx, ry, ryaw)
            
            # Khi Camera/CNN nhìn thấy hàng (confidence >= 0.30):
            # Xe đang bám tim hàng do CNN dẫn đường. Cây/thùng ở đầu hàng hoặc dọc 2 bên luống
            # KHÔNG được coi là vật cản làm đứng xe! Chỉ coi là vật cản nếu sát mũi xe (< 0.16m).
            if confidence >= 0.30:
                obstacle_detected = (front_min_dist < 0.16)
            else:
                obstacle_detected = self.lidar.check_obstacle_in_front(rx, ry, ryaw, inside_row=inside_row)

        # 3. Điều hướng: Tin tưởng 100% Camera/CNN khi nhìn thấy hàng (confidence >= 0.30)
        # Cho phép xe bám vào hàng từ xa hoặc khi tiếp cận xéo góc mà không bị LiDAR đánh lái ngược.
        if confidence >= 0.30:
            # Camera đã thấy hàng rõ ràng -> Giữ nguyên 100% góc lái CNN để dẫn xe thẳng vào tim hàng
            pass
        elif self.lidar is not None:
            # Khi Camera tạm thời mất dấu (confidence < 0.30), dùng LiDAR 2 bên thành hàng làm trợ lái dự phòng
            if (0.15 < left_side_dist < 0.85) and (0.15 < right_side_dist < 0.85):
                corridor_width = left_side_dist + right_side_dist
                if 0.70 <= corridor_width <= 1.50:
                    delta_side = right_side_dist - left_side_dist
                    lidar_centering_bias = float(np.clip((delta_side / 0.20) * (max_angle_deg * 0.7), -max_angle_deg * 0.7, max_angle_deg * 0.7))
                    heading_error = lidar_centering_bias

            # Thoát hiểm khẩn cấp khi xe sắp va quẹt sát sườn (< 15cm) và camera không thấy hàng
            danger_threshold = 0.15
            if left_side_dist < danger_threshold:
                penetration = danger_threshold - left_side_dist
                repulsion_deg = float(np.clip((penetration / 0.05) * max_angle_deg, 2.0, max_angle_deg))
                heading_error = max(heading_error, repulsion_deg)
            elif right_side_dist < danger_threshold:
                penetration = danger_threshold - right_side_dist
                repulsion_deg = float(np.clip((penetration / 0.05) * max_angle_deg, 2.0, max_angle_deg))
                heading_error = min(heading_error, -repulsion_deg)

        # 4. Xác định cảm biến đang chiếm quyền điều khiển
        active_sensor = self.sensor_priority.select_active_tracking_sensor(
            confidence, lidar_available=(self.lidar is not None)
        )

        end_of_row = self.eor_detector.detect(
            confidence=confidence,
            distance_traveled=distance_traveled,
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
