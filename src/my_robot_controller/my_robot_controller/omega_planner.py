#!/usr/bin/env python3
"""
Omega Turn (Bulb / Keyhole Turn) Planner for Agricultural Field Headland Maneuvers.

Quỹ đạo Omega Turn cho phép xe tự hành nông nghiệp chuyển từ luống hiện tại
sang luống kế bên với góc quay đảo chiều 180 độ (+180°), chuyển động tiến liên tục
(v > 0) với bán kính quay lớn (R >= 0.85m), triệt tiêu hoàn toàn hiện tượng xoay tại chỗ
và chống trượt bánh vi sai trên nền đất/cỏ.
"""

import math
import numpy as np
from my_robot_controller.path_utils import Path
from nav_msgs.msg import Path as RosPath
from geometry_msgs.msg import PoseStamped


class OmegaTurnPlanner:
    """
    Bộ sinh quỹ đạo Omega Turn mượt mà (3 cung tròn tiếp tuyến liên tục C1 + C2 + C3
    kết hợp đoạn thoát luống và đoạn dẫn thẳng vào tim luống kế tiếp).
    """

    @staticmethod
    def calculate_r2(row_spacing: float, r1: float, alpha_rad: float) -> float:
        """
        Tính bán kính R2 giải tích chính xác để điểm cuối quỹ đạo chạm đúng
        tim luống kế tiếp cách row_spacing mét:
            R2 = (row_spacing + 2*R1*(1 - cos(alpha))) / (2*cos(alpha))
        """
        cos_a = math.cos(alpha_rad)
        if cos_a < 1e-4:
            return row_spacing / 2.0
        r2 = (row_spacing + 2.0 * r1 * (1.0 - cos_a)) / (2.0 * cos_a)
        return max(r2, 0.40)

    @classmethod
    def generate_path(
        cls,
        start_x: float,
        start_y: float,
        start_yaw: float,
        row_spacing: float = 1.20,
        turn_side: str = 'RIGHT',
        open_angle_deg: float = 35.0,
        r1: float = 0.85,
        clearance_dist: float = 0.25,
        lead_in_dist: float = 0.40,
        step_size: float = 0.05
    ) -> Path:
        """
        Sinh danh sách waypoints dạng Path object cho một vòng quay đầu Omega Turn.

        Tham số:
            start_x, start_y, start_yaw: Tọa độ và hướng xe khi thoát luống (Goal 1)
            row_spacing: Khoảng cách giữa 2 tâm luống (m) (mặc định 1.20m)
            turn_side: 'RIGHT' (sang luống bên phải, dy = -w) hoặc 'LEFT' (sang trái, dy = +w)
            open_angle_deg: Góc mở rộng cua ban đầu (độ) (mặc định 35 độ)
            r1: Bán kính cung mở góc (m) (mặc định 0.85m)
            clearance_dist: Đoạn thẳng chạy thoát đầu luống (m) (mặc định 0.25m)
            lead_in_dist: Đoạn dẫn thẳng vào tim luống kế tiếp (m) (mặc định 0.40m)
            step_size: Khoảng cách giữa 2 waypoint nội suy (m)
        """
        alpha = math.radians(open_angle_deg)
        w = abs(row_spacing)
        r2 = cls.calculate_r2(w, r1, alpha)

        side_sign = -1.0 if turn_side.upper() == 'RIGHT' else 1.0

        local_pts = []

        # ── 1. Đoạn chạy thẳng thoát đầu luống (Clearance) ───────────────
        if clearance_dist > 0.001:
            for d in np.arange(0.0, clearance_dist, step_size):
                local_pts.append((float(d), 0.0, 0.0))

        xc = clearance_dist
        yc = 0.0

        # ── 2. Cung 1: Mở góc sang phía đối diện ─────────────────────────
        # Nếu turn_side == 'RIGHT': mở góc sang TRÁI (CCW), alpha > 0
        # Nếu turn_side == 'LEFT': mở góc sang PHẢI (CW), alpha < 0
        arc1_len = r1 * alpha
        n1 = max(4, int(arc1_len / step_size))

        for t in np.linspace(0.0, alpha, n1):
            x = xc + r1 * math.sin(t)
            y = yc + side_sign * (-1.0) * r1 * (1.0 - math.cos(t))  # dy > 0 nếu turn_side == RIGHT
            yaw = side_sign * (-1.0) * t
            local_pts.append((float(x), float(y), float(yaw)))

        # ── 3. Cung 2: Ôm cua lớn sang hướng mục tiêu ───────────────────
        # Quét góc: pi + 2*alpha
        arc2_len = r2 * (math.pi + 2.0 * alpha)
        n2 = max(10, int(arc2_len / step_size))

        # Tọa độ tâm O2
        O2_x = xc + (r1 + r2) * math.sin(alpha)
        O2_y = yc + side_sign * (-1.0) * (r1 - (r1 + r2) * math.cos(alpha))

        if side_sign < 0:  # Turn RIGHT
            start_ang2 = math.pi / 2.0 + alpha
            end_ang2 = -math.pi / 2.0 - alpha
            for ang in np.linspace(start_ang2, end_ang2, n2)[1:]:
                x = O2_x + r2 * math.cos(ang)
                y = O2_y + r2 * math.sin(ang)
                yaw = ang - math.pi / 2.0
                yaw = math.atan2(math.sin(yaw), math.cos(yaw))
                local_pts.append((float(x), float(y), float(yaw)))
        else:  # Turn LEFT
            start_ang2 = -math.pi / 2.0 - alpha
            end_ang2 = math.pi / 2.0 + alpha
            for ang in np.linspace(start_ang2, end_ang2, n2)[1:]:
                x = O2_x + r2 * math.cos(ang)
                y = O2_y + r2 * math.sin(ang)
                yaw = ang + math.pi / 2.0
                yaw = math.atan2(math.sin(yaw), math.cos(yaw))
                local_pts.append((float(x), float(y), float(yaw)))

        # ── 4. Cung 3: Khép lái trả thẳng đầu xe ─────────────────────────
        O3_x = xc
        O3_y = yc + side_sign * w - side_sign * (-1.0) * r1

        arc3_len = r1 * alpha
        n3 = max(4, int(arc3_len / step_size))

        if side_sign < 0:  # Turn RIGHT
            start_ang3 = math.pi / 2.0 - alpha
            end_ang3 = math.pi / 2.0
            for ang in np.linspace(start_ang3, end_ang3, n3)[1:]:
                x = O3_x + r1 * math.cos(ang)
                y = O3_y + r1 * math.sin(ang)
                yaw = ang + math.pi / 2.0
                yaw = math.atan2(math.sin(yaw), math.cos(yaw))
                local_pts.append((float(x), float(y), float(yaw)))
        else:  # Turn LEFT
            start_ang3 = -math.pi / 2.0 + alpha
            end_ang3 = -math.pi / 2.0
            for ang in np.linspace(start_ang3, end_ang3, n3)[1:]:
                x = O3_x + r1 * math.cos(ang)
                y = O3_y + r1 * math.sin(ang)
                yaw = ang - math.pi / 2.0
                yaw = math.atan2(math.sin(yaw), math.cos(yaw))
                local_pts.append((float(x), float(y), float(yaw)))

        # ── 5. Đoạn dẫn thẳng vào tim luống (Lead-in straight) ───────────
        target_local_y = side_sign * w
        target_local_yaw = math.pi  # 180° đảo chiều
        if lead_in_dist > 0.001:
            for d in np.arange(step_size, lead_in_dist + 1e-4, step_size):
                local_pts.append((float(xc - d), float(target_local_y), float(target_local_yaw)))

        # ── Chuyển đổi tọa độ cục bộ sang tọa độ toàn cục (Global Frame) ──
        cos0 = math.cos(start_yaw)
        sin0 = math.sin(start_yaw)

        global_waypoints = []
        for lx, ly, _ in local_pts:
            gx = start_x + lx * cos0 - ly * sin0
            gy = start_y + lx * sin0 + ly * cos0
            global_waypoints.append([round(gx, 4), round(gy, 4)])

        return Path(global_waypoints, planner_type="OmegaTurn")

    @classmethod
    def generate_full_mission_path(
        cls,
        field_length: float = 3.0,
        row_spacing: float = 1.20,
        turn_side: str = 'RIGHT',
        open_angle_deg: float = 35.0,
        r1: float = 0.85,
        clearance_dist: float = 0.25,
        lead_in_dist: float = 0.40,
        step_size: float = 0.05
    ) -> Path:
        """
        Tạo toàn bộ lộ trình nhiệm vụ 2 luống hoàn chỉnh:
        - Luống 1: Đi từ (0.0, 0.0) đến (L, 0.0)
        - Vòng Omega Turn: Từ (L, 0.0) sang (L, -w)
        - Luống 2: Đi từ (L, -w) về lại (0.0, -w)
        """
        waypoints = []

        # 1. Luống 1: (0.0, 0.0) -> (L, 0.0)
        for x in np.arange(0.0, field_length, step_size):
            waypoints.append([round(float(x), 4), 0.0])
        waypoints.append([round(field_length, 4), 0.0])

        # 2. Vòng Omega Turn
        omega_path = cls.generate_path(
            start_x=field_length,
            start_y=0.0,
            start_yaw=0.0,
            row_spacing=row_spacing,
            turn_side=turn_side,
            open_angle_deg=open_angle_deg,
            r1=r1,
            clearance_dist=clearance_dist,
            lead_in_dist=lead_in_dist,
            step_size=step_size
        )
        if omega_path and omega_path.waypoints:
            for wp in omega_path.waypoints:
                waypoints.append(wp)

        # 3. Luống 2: từ điểm kết thúc Omega về lại x = 0.0 ở y = -w (hoặc +w)
        target_y = -abs(row_spacing) if turn_side.upper() == 'RIGHT' else abs(row_spacing)
        last_x = waypoints[-1][0] if waypoints else field_length
        for x in np.arange(last_x - step_size, -0.01, -step_size):
            waypoints.append([round(float(x), 4), round(target_y, 4)])
        waypoints.append([0.0, round(target_y, 4)])

        return Path(waypoints, planner_type="FullMissionOmega")

    @staticmethod
    def to_ros_path(waypoints, frame_id: str = "odom", stamp=None) -> RosPath:
        """
        Chuyển đổi danh sách waypoints [[x, y], ...] thành thông điệp ROS 2 nav_msgs/msg/Path.
        """
        path_msg = RosPath()
        path_msg.header.frame_id = frame_id
        if stamp is not None:
            path_msg.header.stamp = stamp

        for pt in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            if stamp is not None:
                pose.header.stamp = stamp
            pose.pose.position.x = float(pt[0])
            pose.pose.position.y = float(pt[1])
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        return path_msg
