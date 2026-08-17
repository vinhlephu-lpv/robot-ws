import time
import math

class Path:
    """
    Standardized Path Object for Robot Navigation.
    """
    def __init__(self, waypoints, planner_type="RRTStar"):
        self.waypoints = waypoints
        self.planner_type = planner_type
        self.timestamp = time.time()
        self.length = self._calculate_length()
        self.curvature = self._calculate_curvature()

    def _calculate_length(self):
        if not self.waypoints or len(self.waypoints) < 2:
            return 0.0
        
        total_len = 0.0
        for i in range(len(self.waypoints) - 1):
            p1 = self.waypoints[i]
            p2 = self.waypoints[i+1]
            total_len += math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
        return total_len

    def _calculate_curvature(self):
        """
        Calculates maximum local curvature along the path.
        For three points p1, p2, p3, curvature = 4 * Area / (a * b * c).
        """
        if not self.waypoints or len(self.waypoints) < 3:
            return 0.0
        
        max_k = 0.0
        for i in range(len(self.waypoints) - 2):
            p1 = self.waypoints[i]
            p2 = self.waypoints[i+1]
            p3 = self.waypoints[i+2]
            
            # Side lengths
            a = math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
            b = math.sqrt((p3[0] - p2[0])**2 + (p3[1] - p2[1])**2)
            c = math.sqrt((p3[0] - p1[0])**2 + (p3[1] - p1[1])**2)
            
            if a < 1e-5 or b < 1e-5 or c < 1e-5:
                continue
                
            # Area using cross product of vectors p2-p1 and p3-p1
            # area = 0.5 * |(x2-x1)(y3-y1) - (x3-x1)(y2-y1)|
            area = 0.5 * abs((p2[0] - p1[0]) * (p3[1] - p1[1]) - (p3[0] - p1[0]) * (p2[1] - p1[1]))
            
            # R = (a*b*c) / (4 * area)
            # k = 1 / R = (4 * area) / (a*b*c)
            k = (4.0 * area) / (a * b * c)
            if k > max_k:
                max_k = k
                
        return max_k

    def get_waypoints(self):
        return self.waypoints

    def __len__(self):
        return len(self.waypoints)

    def __getitem__(self, index):
        return self.waypoints[index]
