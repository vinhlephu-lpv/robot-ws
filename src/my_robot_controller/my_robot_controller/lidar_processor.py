import math

class LidarProcessor:
    """
    Decoupled Lidar processing and obstacle detection layer.
    """
    def __init__(self):
        self.latest_scan = None

    def update_scan(self, msg):
        self.latest_scan = msg

    def check_obstacle_in_front(self, max_dist=0.45, angle_range_deg=15.0, robot_half_width=0.15):
        if self.latest_scan is None:
            return False

        angle_min = self.latest_scan.angle_min
        angle_increment = self.latest_scan.angle_increment
        ranges = self.latest_scan.ranges
        angle_limit_rad = math.radians(angle_range_deg)

        for idx, r in enumerate(ranges):
            if math.isnan(r) or math.isinf(r) or r < 0.15 or r > max_dist:
                continue

            angle = angle_min + idx * angle_increment
            if abs(angle) < angle_limit_rad:
                y_lat = r * math.sin(angle)
                if abs(y_lat) < robot_half_width:
                    return True
        return False

    def get_obstacles_global(self, rx, ry, ryaw, max_dist=6.0):
        if self.latest_scan is None:
            return []

        obstacles = []
        angle_min = self.latest_scan.angle_min
        angle_increment = self.latest_scan.angle_increment
        ranges = self.latest_scan.ranges

        for idx, r in enumerate(ranges):
            if math.isnan(r) or math.isinf(r) or r < 0.15 or r > max_dist:
                continue

            angle = angle_min + idx * angle_increment
            
            # Local coordinates
            ox_local = r * math.cos(angle)
            oy_local = r * math.sin(angle)
            
            # Global coordinates
            ox_global = rx + ox_local * math.cos(ryaw) - oy_local * math.sin(ryaw)
            oy_global = ry + ox_local * math.sin(ryaw) + oy_local * math.cos(ryaw)
            
            obstacles.append([ox_global, oy_global])

        return obstacles

    def get_min_range_in_sector(self, min_angle_deg=-25.0, max_angle_deg=25.0):
        if self.latest_scan is None:
            return float('inf')

        angle_min = self.latest_scan.angle_min
        angle_increment = self.latest_scan.angle_increment
        ranges = self.latest_scan.ranges
        
        min_rad = math.radians(min_angle_deg)
        max_rad = math.radians(max_angle_deg)
        
        min_dist = float('inf')
        for idx, r in enumerate(ranges):
            if math.isnan(r) or math.isinf(r) or r < 0.15:
                continue
            angle = angle_min + idx * angle_increment
            if min_rad <= angle <= max_rad:
                if r < min_dist:
                    min_dist = r
        return min_dist
