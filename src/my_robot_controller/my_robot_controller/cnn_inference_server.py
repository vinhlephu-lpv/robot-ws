#!/usr/bin/env python3
"""
CNN Inference Server (Chạy trên Laptop để gánh toàn bộ tác vụ AI cho Pi).
Chức năng:
  - Nhận luồng ảnh nén (/camera/compressed) hoặc raw (/camera/color/image_raw) từ Pi qua Wi-Fi/LAN.
  - Chạy mô hình ONNX trên Laptop với CPU/GPU mạnh (tốc độ 30 - 60+ FPS, độ trễ ~10-15ms).
  - Trích xuất: góc lệch hướng (heading_error), độ lệch tim luống (lane_offset), tọa độ tâm (lane_center), độ tin cậy (confidence).
  - Bắn kết quả về Pi qua topic '/crop_row/detection' (Float32MultiArray).
  - Node 'cnn_driver' trên Pi nhận kết quả này để ra quyết định điều khiển (dừng xoay IMU, né cản, bẻ lái).

Sử dụng trên Laptop:
  ros2 run my_robot_controller cnn_server
  (hoặc dùng lệnh tắt: laptop-cnn)
"""

import os
import sys
import time
import math
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32MultiArray

from .inference_handler import InferenceHandler

try:
    from .controllers import TrackingControllerSMC
except ImportError:
    try:
        from my_robot_controller.controllers import TrackingControllerSMC
    except ImportError:
        TrackingControllerSMC = None

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None


def vel_to_duty(velocity_ms: float, max_linear_speed: float = 0.18, min_duty_cycle: float = 22.0) -> float:
    """Chuyển vận tốc (m/s) -> duty cycle có dấu (-100 .. +100) theo chuẩn driver BTS7960."""
    clamped = max(-max_linear_speed, min(max_linear_speed, velocity_ms))
    raw_duty = clamped / max_linear_speed * 100.0
    if abs(raw_duty) > 0.5 and abs(raw_duty) < min_duty_cycle:
        raw_duty = min_duty_cycle if raw_duty > 0 else -min_duty_cycle
    return raw_duty


def draw_hud_3panel(bgr_orig, mask_prob, lane_center, conf, heading_err, lane_off, 
                    v_lin, w_ang, rpm_l, rpm_r, duty_l, duty_r, esp_cmd, inference_ms,
                    state_name, state_color, roi_ratio=0.80, mask_thresh=0.35, max_steer=14.0):
    """
    Renders 3-panel vehicle HUD:
    [1: Camera Gốc] | [2: Mặt Nạ CNN (ROI 80%)] | [3: Dashboard Điều Khiển Xe Thật]
    Kích thước tối ưu: 1260x360 (420x360 mỗi panel), siêu nhẹ, siêu mượt, vừa vặn mọi màn hình Laptop.
    """
    h_orig, w_orig = bgr_orig.shape[:2]
    vis_h, vis_w = 360, 420

    # Panel 1: Original resized to match HUD aspect
    p1 = cv2.resize(bgr_orig, (vis_w, vis_h))

    # Panel 2: Colored Segmentation Mask (384x384 -> vis_w x vis_h)
    mask_vis = cv2.resize(mask_prob, (vis_w, vis_h))
    bin_mask = (mask_vis >= mask_thresh).astype(np.uint8)
    
    p2 = np.zeros((vis_h, vis_w, 3), dtype=np.uint8)
    p2[:] = [25, 45, 25] # Nền cỏ xanh đậm thực tế
    p2[bin_mask > 0] = [230, 180, 0] # Hàng thùng carton / luống màu vàng đậm
    
    # Boundary contour overlay
    contours, _ = cv2.findContours(bin_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(p2, contours, -1, (255, 255, 255), 1)

    # Vạch giới hạn ROI 80% trên Panel 2
    y_roi = int(round(vis_h * (1.0 - roi_ratio)))
    p2[0:y_roi, :] = (p2[0:y_roi, :].astype(np.uint16) * 4 // 10).astype(np.uint8)
    for x in range(0, vis_w, 16):
        cv2.line(p2, (x, y_roi), (min(x + 8, vis_w), y_roi), (0, 180, 255), 1)
    cv2.putText(p2, f"CUT TOP {int((1.0-roi_ratio)*100)}% (BO QUA HAU CANH)", (10, max(y_roi - 6, 18)), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (160, 160, 255), 1, cv2.LINE_AA)
    cv2.putText(p2, f"VUNG ROI {int(roi_ratio*100)}% DUNG SUY LUAN", (10, y_roi + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 200), 1, cv2.LINE_AA)

    # Panel 3: Live Vehicle HUD (100% Robot Driving Simulation)
    p3 = p1.copy()
    mask_idx = bin_mask > 0
    if np.any(mask_idx):
        p3[mask_idx] = ((p3[mask_idx].astype(np.uint16) * 65 + np.uint16([255, 200, 0]) * 35) // 100).astype(np.uint8)

    # Vạch ROI trên Panel 3
    for x in range(0, vis_w, 16):
        cv2.line(p3, (x, y_roi), (min(x + 8, vis_w), y_roi), (0, 215, 255), 1)

    # Guide Lines
    img_center_x = int((vis_w - 1) * 0.5)
    w_mask = mask_prob.shape[1]
    target_center_x = int(np.clip(lane_center * (vis_w / float(w_mask)), 0, vis_w - 1))

    # 1. Image Center (Mũi xe / Tâm trục robot) - Nét đứt màu xanh lá
    for y in range(0, vis_h, 16):
        cv2.line(p3, (img_center_x, y), (img_center_x, min(y + 8, vis_h)), (0, 255, 0), 2)

    # 2. Detected Row Center (Tim luống do AI phát hiện) - Đường nét liền vàng/xanh ngọc
    cv2.line(p3, (target_center_x, vis_h - 1), (target_center_x, y_roi), (0, 215, 255), 2)

    # 3. Steering Target Vector (Mũi tên bẻ lái từ tâm xe đến tim luống)
    arrow_y = int(vis_h * 0.74)
    cv2.arrowedLine(p3, (img_center_x, arrow_y), (target_center_x, arrow_y), (0, 0, 255), 2, tipLength=0.22)

    # 4. Dashboard Bar trên cùng Panel 3
    dash_h = 96
    p3[0:dash_h, :] = (p3[0:dash_h, :].astype(np.uint16) * 2 // 10).astype(np.uint8)

    # Line 1: State Badge
    cv2.putText(p3, f"STATUS: {state_name}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.44, state_color, 1, cv2.LINE_AA)
    
    # Line 2: Vision Metrics
    cv2.putText(p3, f"Conf: {conf*100:4.1f}% | Offset: {lane_off:+.3f} | Lai: {heading_err:+.2f} deg (Max +/-{max_steer:.0f} deg)", 
                (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 255), 1, cv2.LINE_AA)
    
    # Line 3: Kinematics & Motor RPM
    cv2.putText(p3, f"Speed: v={v_lin:.3f} m/s | w={w_ang:+.3f} rad/s | L={rpm_l:+.1f} RPM | R={rpm_r:+.1f} RPM", 
                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 255, 200), 1, cv2.LINE_AA)
    
    # Line 4: BTS7960 PWM & ESP32 Protocol & FPS
    fps_val = 1000.0 / max(1.0, inference_ms)
    cv2.putText(p3, f"PWM: L={duty_l:+.0f}% R={duty_r:+.0f}% | ESP: {repr(esp_cmd).strip()} | {inference_ms:.1f}ms ({fps_val:.1f} FPS)", 
                (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (0, 220, 255), 1, cv2.LINE_AA)

    # Add labels to top of panels
    cv2.putText(p1, f"[1] CAMERA GOC ({w_orig}x{h_orig} -> D435)", (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(p2, f"[2] CNN MASK 384x384 (ROI: {int(roi_ratio*100)}%)", (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(p3, "[3] DIEU KHIEN XE THAT (REALTIME)", (10, dash_h + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 1, cv2.LINE_AA)

    # Combine 3 panels horizontally (Tổng 1260 x 360)
    combined = np.hstack((p1, p2, p3))
    return combined


class CnnInferenceServer(Node):
    def __init__(self):
        super().__init__('cnn_inference_server')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('model_path', 'models/crop_row_cnn_best_final_int8.onnx')
        self.declare_parameter('input_height', 384)
        self.declare_parameter('input_width', 384)
        self.declare_parameter('mask_threshold', 0.30)
        self.declare_parameter('roi_ratio', 0.80)
        self.declare_parameter('max_steering_angle_deg', 14.0)
        self.declare_parameter('num_threads', 0)
        self.declare_parameter('show_window', False)

        p = self.get_parameter
        self.model_path = p('model_path').value
        self.input_height = p('input_height').value
        self.input_width = p('input_width').value
        self.mask_threshold = p('mask_threshold').value
        self.roi_ratio = float(p('roi_ratio').value)
        self.max_steering_angle_deg = p('max_steering_angle_deg').value
        self.num_threads = p('num_threads').value
        self.show_window = p('show_window').value

        # Tự động tìm model ONNX nếu đường dẫn tương đối
        if not os.path.exists(self.model_path):
            candidate_names = [os.path.basename(self.model_path), 'crop_row_cnn_best_final_int8.onnx', 'crop_row_cnn_best_final.onnx']
            try:
                from ament_index_python.packages import get_package_share_directory
                share_dir = get_package_share_directory('my_robot_controller')
                for name in candidate_names:
                    cand = os.path.join(share_dir, 'models', name)
                    if os.path.exists(cand):
                        self.model_path = cand
                        break
            except Exception:
                pass

        if not os.path.exists(self.model_path):
            current_dir = os.path.dirname(os.path.abspath(__file__))
            for name in candidate_names:
                cand = os.path.abspath(os.path.join(current_dir, '..', 'models', name))
                if os.path.exists(cand):
                    self.model_path = cand
                    break

        self.get_logger().info(f"💻 [CNN Server] Nạp mô hình ONNX: {self.model_path}")
        self.inference = InferenceHandler(
            model_path=self.model_path,
            mask_threshold=self.mask_threshold,
            input_size=(self.input_height, self.input_width),
            use_hsv_mask=False,
            num_threads=self.num_threads,
            roi_ratio=self.roi_ratio
        )

        self.bridge = CvBridge() if CvBridge is not None else None

        # Thông số mô phỏng điều khiển & động học xe thật (differential drive)
        self.linear_speed = 0.08
        self.turn_angular_speed = 0.25
        self.turn_in_place_thresh = 8.0
        self.low_conf_thresh = 0.35
        self.wheel_base = 0.40
        self.wheel_d = 0.20
        self.wheel_circ = math.pi * self.wheel_d
        self.max_linear_speed = 0.18
        self.min_duty_cycle = 22.0

        if TrackingControllerSMC is not None:
            try:
                self.controller = TrackingControllerSMC()
                self.controller.initialize(
                    lambda_smc=2.5,
                    k_smc=4.2,
                    eta_smc=0.8,
                    phi_smc=0.4,
                    linear_speed=self.linear_speed,
                    turn_angular_speed=self.turn_angular_speed
                )
            except Exception:
                self.controller = None
        else:
            self.controller = None

        # Cấu hình cửa sổ hiển thị HUD (1 cửa sổ duy nhất, không tạo tab dư thừa)
        self.window_name = "AI CNN Crop Row - Laptop HUD"
        self._window_created = False
        self._last_gui_time = 0.0

        # ── Publishers ────────────────────────────────────────────────
        # Topic bắn kết quả phát hiện về Pi (gọn nhẹ ~20 bytes, truyền cực nhanh qua Wi-Fi)
        qos_det = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )
        self.detection_pub = self.create_publisher(Float32MultiArray, '/crop_row/detection', qos_det)

        # ── Subscribers ───────────────────────────────────────────────
        # 1. Nhận ảnh nén JPEG từ Pi (Ưu tiên: siêu nhẹ trên Wi-Fi ~20KB/frame)
        qos_compressed = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )
        self.compressed_sub = self.create_subscription(
            CompressedImage, '/camera/compressed', self.compressed_image_callback, qos_compressed
        )

        # 2. Nhận ảnh raw (Tương thích với cả RELIABLE và BEST_EFFORT từ camera_publisher)
        qos_raw = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            durability=DurabilityPolicy.VOLATILE
        )
        self.raw_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.raw_image_callback, qos_raw
        )

        # ── Metrics ───────────────────────────────────────────────────
        self._last_process_time = time.time()
        self._frame_count = 0
        self._fps_timer = self.create_timer(1.0, self._log_fps)
        self._latest_fps = 0.0
        self._latest_latency_ms = 0.0

        self.get_logger().info("🚀 [CNN Server Laptop] Sẵn sàng xử lý AI! Đang lắng nghe luồng ảnh từ Camera...")

    def compressed_image_callback(self, msg: CompressedImage):
        """Giải nén JPEG và chạy inference."""
        self._last_compressed_time = time.time()
        t_start = time.time()
        try:
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            bgr_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if bgr_img is None:
                return
            self._process_and_publish(bgr_img, t_start)
        except Exception as e:
            self.get_logger().warn(f"Lỗi giải nén: {e}", throttle_duration_sec=2.0)

    def raw_image_callback(self, msg: Image):
        """Xử lý trực tiếp ảnh raw."""
        # Nếu đã có luồng compressed đang chạy thì bỏ qua raw để tránh xử lý trùng
        now = time.time()
        if (now - getattr(self, '_last_compressed_time', 0.0)) < 0.2:
            return

        t_start = time.time()
        try:
            if self.bridge is not None:
                img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            elif msg.encoding in ('rgb8', 'RGB8'):
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            else:
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            self._process_and_publish(img, t_start)
        except Exception as e:
            self.get_logger().warn(f"Lỗi đọc raw: {e}", throttle_duration_sec=2.0)

    def _process_and_publish(self, bgr_img: np.ndarray, t_start: float):
        heading_error, lane_offset, lane_center, confidence = self.inference.process_image(
            bgr_img, self.max_steering_angle_deg
        )
        latency_ms = (time.time() - t_start) * 1000.0
        self._latest_latency_ms = latency_ms

        # Bắn kết quả về Pi
        out_msg = Float32MultiArray()
        # [0]: heading_error (độ)
        # [1]: lane_offset (-1.0 đến 1.0)
        # [2]: lane_center (pixel X)
        # [3]: confidence (0.0 đến 1.0)
        # [4]: latency_ms (thời gian tính toán)
        out_msg.data = [
            float(heading_error),
            float(lane_offset),
            float(lane_center),
            float(confidence),
            float(latency_ms)
        ]
        self.detection_pub.publish(out_msg)
        self._frame_count += 1

        # Hiển thị cửa sổ HUD trực quan 3 khung hình OpenCV trên Laptop
        if self.show_window:
            now_gui = time.time()
            if now_gui - self._last_gui_time < 0.033:  # Điều tiết render ~30 FPS để tối ưu tài nguyên máy
                return
            self._last_gui_time = now_gui

            # Kiểm tra xem người dùng có bấm nút [X] đóng cửa sổ không
            if self._window_created:
                try:
                    prop = cv2.getWindowProperty(self.window_name, cv2.WND_PROP_VISIBLE)
                    if prop < 1:
                        self.get_logger().info("🛑 Người dùng đã bấm [X] đóng cửa sổ HUD. Tiếp tục chạy ngầm tối đa FPS.")
                        self.show_window = False
                        cv2.destroyAllWindows()
                        self._window_created = False
                except Exception:
                    self.get_logger().info("🛑 Cửa sổ HUD đã đóng. Tiếp tục chạy ngầm tối đa FPS.")
                    self.show_window = False
                    cv2.destroyAllWindows()
                    self._window_created = False

            # Khởi tạo cửa sổ 1 LẦN DUY NHẤT
            if self.show_window and not self._window_created:
                try:
                    cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
                    cv2.resizeWindow(self.window_name, 1260, 360)
                    self._window_created = True
                    self.get_logger().info("🌾 [CNN Server] Kích hoạt cửa sổ HUD 3 khung hình OpenCV trực quan.")
                except Exception as e:
                    self.get_logger().warn(f"Không thể mở cửa sổ GUI: {e}")
                    self.show_window = False

            if self.show_window and self._window_created:
                # 1. Tính toán trạng thái FSM và Động học xe mô phỏng (100% khớp cnn_driver & test-img)
                if confidence < self.low_conf_thresh:
                    state_name = "LOST / EOR (MAT DAU / HET HANG)"
                    state_color = (0, 0, 255)  # Đỏ
                    v_lin = 0.0
                    w_ang = 0.0
                elif abs(heading_error) > self.turn_in_place_thresh:
                    state_name = f"DUNG TIEN - XOAY TAI CHO (|goc|={abs(heading_error):.1f}° > {self.turn_in_place_thresh:.1f}°)"
                    state_color = (0, 215, 255)  # Vàng cam
                    v_lin = 0.0
                    turn_dir = -1.0 if heading_error > 0 else 1.0
                    w_ang = turn_dir * self.turn_angular_speed
                else:
                    state_name = f"TIEN BAM LUONG SMC (|goc|={abs(heading_error):.1f}° <= {self.turn_in_place_thresh:.1f}°)"
                    state_color = (0, 255, 0)  # Xanh lá
                    v_lin = self.linear_speed
                    if self.controller is not None:
                        try:
                            self.controller.reset()
                            cmd = self.controller.compute_command(heading_error, dt_actual=0.067)
                            w_ang = float(cmd.get("angular_velocity", 0.0))
                        except Exception:
                            w_ang = float(np.clip(-0.045 * heading_error, -0.6, 0.6))
                    else:
                        w_ang = float(np.clip(-0.045 * heading_error, -0.6, 0.6))

                # 2. Differential Drive Kinematics (Vận tốc bánh & RPM)
                v_left = v_lin - (w_ang * self.wheel_base / 2.0)
                v_right = v_lin + (w_ang * self.wheel_base / 2.0)
                rpm_left = (v_left / self.wheel_circ) * 60.0
                rpm_right = (v_right / self.wheel_circ) * 60.0
                esp_cmd = f"V {rpm_left:.1f} {rpm_right:.1f}\n"

                # 3. BTS7960 Motor PWM Duty Cycle
                v_l_bts = v_left
                v_r_bts = v_right
                if v_lin > 0.03:
                    min_fwd = 0.035
                    min_v = min(v_l_bts, v_r_bts)
                    if min_v < min_fwd:
                        shift = min_fwd - min_v
                        v_l_bts += shift
                        v_r_bts += shift
                duty_l = vel_to_duty(v_l_bts, self.max_linear_speed, self.min_duty_cycle)
                duty_r = vel_to_duty(v_r_bts, self.max_linear_speed, self.min_duty_cycle)

                # 4. Lấy mặt nạ CNN từ inference handler
                mask_prob = getattr(self.inference, 'latest_mask', None)
                if mask_prob is None:
                    mask_prob = np.zeros((self.input_height, self.input_width), dtype=np.float32)

                # 5. Vẽ HUD 3 Panel (Camera gốc | CNN Mask | Dashboard xe thật)
                hud_canvas = draw_hud_3panel(
                    bgr_orig=bgr_img,
                    mask_prob=mask_prob,
                    lane_center=lane_center,
                    conf=confidence,
                    heading_err=heading_error,
                    lane_off=lane_offset,
                    v_lin=v_lin,
                    w_ang=w_ang,
                    rpm_l=rpm_left,
                    rpm_r=rpm_right,
                    duty_l=duty_l,
                    duty_r=duty_r,
                    esp_cmd=esp_cmd,
                    inference_ms=latency_ms,
                    state_name=state_name,
                    state_color=state_color,
                    roi_ratio=self.roi_ratio,
                    mask_thresh=self.mask_threshold,
                    max_steer=self.max_steering_angle_deg
                )

                cv2.imshow(self.window_name, hud_canvas)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord('q'), ord('Q'), 27):  # 'q' hoặc ESC -> Đóng vĩnh viễn cửa sổ
                    self.get_logger().info("🛑 Người dùng nhấn phím 'q' / ESC. Đóng cửa sổ HUD và chuyển sang chạy ngầm.")
                    self.show_window = False
                    cv2.destroyAllWindows()
                    self._window_created = False

    def _log_fps(self):
        fps = self._frame_count
        self._frame_count = 0
        self._latest_fps = fps
        if fps > 0:
            self.get_logger().info(
                f"⚡ [Laptop CNN Worker] Tốc độ: {fps} FPS | Độ trễ suy luận: {self._latest_latency_ms:.1f} ms"
            )


def main(args=None):
    rclpy.init(args=args)
    node = CnnInferenceServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
