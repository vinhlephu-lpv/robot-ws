import math

class LidarProcessor:
    """
    Decoupled Lidar processing and obstacle detection layer.
    """
    def __init__(self):
        self.latest_scan = None

    def update_scan(self, msg):
        self.latest_scan = msg

    def check_obstacle_in_front(self, rx=0.0, ry=0.5, ryaw=0.0, max_dist=1.50, inside_row=True):
        if self.latest_scan is None:
            return False

        lane_center = round(ry - 0.5) + 0.5
        dir_x = 1.0 if math.cos(ryaw) >= 0 else -1.0
        angle_min = self.latest_scan.angle_min
        angle_increment = self.latest_scan.angle_increment
        ranges = self.latest_scan.ranges

        for idx, r in enumerate(ranges):
            if math.isnan(r) or math.isinf(r) or r < 0.10 or r > max_dist:
                continue

            beam_yaw = ryaw + (angle_min + idx * angle_increment)
            xg = rx + r * math.cos(beam_yaw)
            yg = ry + r * math.sin(beam_yaw)
            
            fwd_dist = dir_x * (xg - rx)
            lat_err = abs(yg - lane_center)
            
            # True obstacle strictly in the driving lane corridor (ignoring stalks at >= 0.38m)
            if 0.10 < fwd_dist <= max_dist and lat_err <= 0.35:
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
            if math.isnan(r) or math.isinf(r) or r < 0.08:
                continue
            angle = angle_min + idx * angle_increment
            if min_rad <= angle <= max_rad:
                if r < min_dist:
                    min_dist = r
        return min_dist

    def get_obstacle_lateral_offset(self, max_dist=1.60, robot_half_width=0.35):
        """
        Returns average lateral y position (in robot frame) of obstacle in front corridor.
        Positive y = obstacle is on LEFT.
        Negative y = obstacle is on RIGHT.
        """
        if self.latest_scan is None:
            return 0.0
        angle_min = self.latest_scan.angle_min
        angle_increment = self.latest_scan.angle_increment
        ranges = self.latest_scan.ranges
        
        y_pts = []
        for idx, r in enumerate(ranges):
            if math.isnan(r) or math.isinf(r) or r < 0.08 or r > max_dist:
                continue
            angle = angle_min + idx * angle_increment
            x_fwd = r * math.cos(angle)
            y_lat = r * math.sin(angle)
            if 0.08 < x_fwd < max_dist and abs(y_lat) <= robot_half_width:
                y_pts.append(y_lat)
        if len(y_pts) > 0:
            return float(sum(y_pts) / len(y_pts))
        return 0.0

    def get_front_obstacle_info(self, rx=0.0, ry=0.5, ryaw=0.0, max_dist=1.80):
        """
        Extracts comprehensive obstacle geometric properties in global frame (longitudinal distance,
        lateral position, span, and optimal side to bypass).
        """
        if self.latest_scan is None:
            return {"detected": False, "dist": 1.0, "x_obs": rx + 1.0, "y_obs": ry, "lane_center": ry, "side": "LEFT"}

        lane_center = round(ry - 0.5) + 0.5
        dir_x = 1.0 if math.cos(ryaw) >= 0 else -1.0
        angle_min = self.latest_scan.angle_min
        angle_increment = self.latest_scan.angle_increment
        ranges = self.latest_scan.ranges

        obs_points = []
        for idx, r in enumerate(ranges):
            if math.isnan(r) or math.isinf(r) or r < 0.10 or r > max_dist:
                continue

            beam_yaw = ryaw + (angle_min + idx * angle_increment)
            xg = rx + r * math.cos(beam_yaw)
            yg = ry + r * math.sin(beam_yaw)
            
            fwd_dist = dir_x * (xg - rx)
            lat_err = abs(yg - lane_center)
            
            if 0.10 < fwd_dist <= max_dist and lat_err <= 0.35:
                obs_points.append((fwd_dist, xg, yg))

        if not obs_points:
            return {"detected": False, "dist": 1.0, "x_obs": rx + 1.0, "y_obs": lane_center, "lane_center": lane_center, "side": "LEFT"}

        # Sort by distance
        obs_points.sort(key=lambda p: p[0])
        min_dist = obs_points[0][0]
        x_obs = obs_points[0][1]
        
        # Average Y of front obstacle cluster
        front_cluster = [p for p in obs_points if p[0] <= min_dist + 0.25]
        y_obs = float(sum(p[2] for p in front_cluster) / len(front_cluster))
        
        # When moving in forward row (+X): yg > lane_center means obstacle on LEFT
        # When moving in return row (-X): yg > lane_center means obstacle on RIGHT
        is_fwd = (dir_x > 0)
        if is_fwd:
            side = "LEFT" if (y_obs >= lane_center) else "RIGHT"
        else:
            side = "RIGHT" if (y_obs >= lane_center) else "LEFT"

        return {
            "detected": True,
            "dist": min_dist,
            "x_obs": x_obs,
            "y_obs": y_obs,
            "lane_center": lane_center,
            "side": side
        }
