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
    def __init__(self, min_row_distance=1.0, low_confidence_threshold=0.30, consecutive_frames=10):
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

        # 1. Vision check: Bộ đếm tích lũy mềm (Leaky Counter) chống 1 frame nhiễu xóa sạch tiến trình đếm hết hàng
        if confidence < self.low_confidence_threshold:
            self.low_confidence_counter += 1
        else:
            self.low_confidence_counter = max(0, self.low_confidence_counter - 2)
        
        camera_eor = (self.low_confidence_counter >= self.consecutive_frames)
        camera_definite_eor = (self.low_confidence_counter >= (self.consecutive_frames + 4))
        
        # 2. LiDAR check: Kích hoạt khi phía trước trống (> 1.0m), hai bên sườn trống (> 0.38m)
        # VÀ Camera cũng mất dấu luống (confidence < low_confidence_threshold).
        if front_min_dist > 1.00 and left_side_dist > 0.38 and right_side_dist > 0.38 and (confidence < self.low_confidence_threshold):
            self.lidar_clearance_counter += 1
        else:
            self.lidar_clearance_counter = max(0, self.lidar_clearance_counter - 1)

        lidar_eor = (self.lidar_clearance_counter >= self.consecutive_frames)
        
        # 3. Hai bên sườn thoáng (> 0.38m) HOẶC ít nhất 1 bên thoáng (> 0.42m) và Camera đã xác nhận hết hàng rõ rệt:
        # (Vì camera đặt ở mũi xe nhìn thấy hết hàng trước khi hông xe lọt hẳn qua thùng cuối cùng)
        sides_cleared = (left_side_dist > 0.38 and right_side_dist > 0.38) or (
            camera_definite_eor and max(left_side_dist, right_side_dist) > 0.42
        )
        
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
            left_side_dist = self.lidar.get_min_range_in_sector(35.0, 85.0)
            right_side_dist = self.lidar.get_min_range_in_sector(-85.0, -35.0)
            rear_left_dist = self.lidar.get_min_range_in_sector(70.0, 135.0)
            rear_right_dist = self.lidar.get_min_range_in_sector(-135.0, -70.0)
            obstacles = self.lidar.get_obstacles_global(rx, ry, ryaw)
            
            # Chỉ coi là vật cản nếu sát cản trước (< 0.16m) để không nhầm thùng carton hoặc biển mũi tên YOLO ở khoảng cách 0.4m - 1.0m là vật cản.
            obstacle_detected = (front_min_dist < 0.16)

        # 3. Điều hướng:
        # - Khi Camera/CNN nhìn thấy hàng (confidence >= 0.30): Giữ nguyên 100% góc lái CNN.
        # - Khi không thấy hàng (confidence < 0.30): Tuyệt đối KHÔNG để LiDAR tự bẻ lái lệch trái/phải
        #   khi chỉ còn 1 thùng ở cuối hàng; chỉ giữ phản xạ tránh va quẹt sát sườn khẩn cấp (< 14cm).
        if confidence >= 0.30:
            pass
        elif self.lidar is not None:
            danger_threshold = 0.14
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
