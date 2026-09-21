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
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32MultiArray

from .inference_handler import InferenceHandler

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None


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

        # ── Publishers ────────────────────────────────────────────────
        # Topic bắn kết quả phát hiện về Pi (gọn nhẹ ~20 bytes, truyền cực nhanh qua Wi-Fi)
        qos_det = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE
        )
        self.detection_pub = self.create_publisher(Float32MultiArray, '/crop_row/detection', qos_det)
        self.debug_img_pub = self.create_publisher(CompressedImage, '/crop_row/debug_mask/compressed', qos_det)

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

        # Hiển thị cửa sổ debug nếu bật show_window
        if self.show_window:
            h, w = bgr_img.shape[:2]
            hud = bgr_img.copy()

            # 1. Overlay Mask bám luống (màu xanh lá cây / ngọc bích trên luống bắp)
            if hasattr(self.inference, 'latest_mask') and self.inference.latest_mask is not None:
                try:
                    mask = self.inference.latest_mask
                    if mask.shape[:2] != (h, w):
                        mask = cv2.resize(mask, (w, h), interpolation=cv2.INTER_LINEAR)
                    bin_mask = (mask >= self.mask_threshold)
                    overlay = hud.copy()
                    overlay[bin_mask] = [0, 255, 120]  # Spring green
                    hud = cv2.addWeighted(hud, 0.70, overlay, 0.30, 0)
                except Exception:
                    pass

            # 2. Vạch tim ảnh tham chiếu (Màu vàng đứt đoạn)
            img_center_x = int((w - 1) * 0.5)
            for y_seg in range(int(h * 0.3), h, 20):
                cv2.line(hud, (img_center_x, y_seg), (img_center_x, min(y_seg + 10, h)), (0, 255, 255), 2)

            # 3. Vạch tim luống AI bám theo (Màu cam / xanh dương đậm nét)
            lane_x_px = int(lane_center * (w / float(self.input_width)))
            lane_x_px = max(0, min(w - 1, lane_x_px))
            cv2.line(hud, (lane_x_px, h - 1), (lane_x_px, int(h * 0.35)), (255, 128, 0), 3)

            # 4. Thanh trạng thái trên đỉnh (Header HUD)
            cv2.rectangle(hud, (0, 0), (w, 65), (20, 20, 20), -1)
            cv2.line(hud, (0, 65), (w, 65), (0, 255, 0), 2)

            state_color = (0, 255, 0) if confidence >= 0.35 else (0, 165, 255)
            cv2.putText(
                hud,
                f"GOC LAI: {heading_error:+5.1f} deg | TIN CAY: {int(confidence*100)}% | {self._latest_fps:.0f} FPS",
                (12, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                state_color,
                2
            )
            cv2.putText(
                hud,
                f"Wi-Fi -> Pi [/crop_row/detection] | Do tre: {latency_ms:.1f}ms | Tam: {lane_x_px}px",
                (12, 52),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                (200, 200, 200),
                1
            )

            # 5. Thước đo góc lái đồ họa (Steering Angle Gauge) ở đáy ảnh
            gauge_w = 240
            gauge_x0 = (w - gauge_w) // 2
            gauge_y = h - 25
            cv2.rectangle(hud, (gauge_x0 - 5, gauge_y - 12), (gauge_x0 + gauge_w + 5, gauge_y + 12), (30, 30, 30), -1)
            cv2.line(hud, (gauge_x0, gauge_y), (gauge_x0 + gauge_w, gauge_y), (100, 100, 100), 2)
            cv2.line(hud, (gauge_x0 + gauge_w // 2, gauge_y - 8), (gauge_x0 + gauge_w // 2, gauge_y + 8), (255, 255, 255), 2)
            
            # Con trỏ góc lái
            indicator_x = int(gauge_x0 + (gauge_w / 2) + (heading_error / self.max_steering_angle_deg) * (gauge_w / 2))
            indicator_x = max(gauge_x0, min(gauge_x0 + gauge_w, indicator_x))
            cv2.circle(hud, (indicator_x, gauge_y), 6, (0, 255, 255), -1)
            cv2.circle(hud, (indicator_x, gauge_y), 7, (0, 0, 255), 1)

            cv2.imshow("🌾 AI CROP ROW CNN (LAPTOP WORKER)", hud)
            cv2.waitKey(1)

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
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
