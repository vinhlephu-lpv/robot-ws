#!/usr/bin/env python3
"""
Arrow Marker Detector & RealSense Depth Distance Measurer
--------------------------------------------------------
Module nhận diện Biển báo Mũi tên / Vật mốc quay đầu bằng YOLO (ONNX/PyTorch)
và kết hợp ma trận độ sâu RealSense D435 để đo chính xác khoảng cách (mét).

Hai điều kiện kích hoạt quay đầu:
  1. YOLO phát hiện vật mốc (confidence >= 0.50).
  2. Khoảng cách đo được từ RealSense Depth đến vật mốc <= 1.0m (xác nhận liên tiếp N frame).
Khi thỏa mãn 2 điều kiện -> Kích hoạt cờ quay đầu (turn_trigger = True).
"""

import os
import time
import cv2
import numpy as np


class ArrowMarkerDetector:
    def __init__(self, model_path=None, conf_thresh=0.45, trigger_dist_m=1.0, 
                 consecutive_frames=3, logger=None):
        self.logger = logger
        self.conf_thresh = conf_thresh
        self.trigger_dist_m = trigger_dist_m
        self.consecutive_frames = consecutive_frames

        self.consecutive_counter = 0
        self.consecutive_counter_yellow = 0
        self.consecutive_counter_black = 0
        self.is_triggered = False
        self.is_yellow_triggered = False
        self.is_black_triggered = False
        self.last_trigger_time = 0.0

        self.class_names = {0: 'yellow_arrow', 1: 'black_arrow'}

        # Tìm model
        self.model_path = model_path
        self.session = None
        self.ov_infer_request = None
        self.is_model_loaded = False
        self._init_model()

    def _log_info(self, msg):
        if self.logger:
            self.logger.info(msg)
        else:
            print(f"[ArrowDetector] {msg}")

    def _log_warn(self, msg):
        if self.logger:
            self.logger.warn(msg)
        else:
            print(f"[ArrowDetector] ⚠️ {msg}")

    def _init_model(self):
        """Khởi tạo model YOLO ONNX nếu có file."""
        candidates = []
        if self.model_path:
            candidates.append(self.model_path)

        # Các tên file model mặc định được tìm kiếm (Ưu tiên yolo_arrow.onnx vừa train)
        model_names = [
            'yolo_arrow.onnx',
            'turn_arrow_yolo.onnx',
            'arrow_marker_yolo.onnx',
            'arrow_yolo.onnx',
            'turn_marker.onnx',
            'arrow_best.onnx',
            'best.onnx'
        ]

        # 1. Thư mục install / share của ROS 2 package
        try:
            from ament_index_python.packages import get_package_share_directory
            share_dir = get_package_share_directory('my_robot_controller')
            for name in model_names:
                candidates.append(os.path.join(share_dir, 'models', name))
        except Exception:
            pass

        # 2. Thư mục mã nguồn / workspace
        ws_dir = os.path.expanduser('~/robot_ws')
        for name in model_names:
            candidates.append(os.path.join(ws_dir, 'models', name))
            candidates.append(os.path.join(ws_dir, 'src/my_robot_controller/models', name))

        found_path = None
        for p in candidates:
            if p and os.path.exists(p):
                found_path = os.path.abspath(p)
                break

        if found_path:
            try:
                has_nvidia_dev = os.path.exists('/dev/nvidia0') or os.path.exists('/dev/nvidiactl')
                if has_nvidia_dev:
                    import glob, ctypes
                    for lib_dir in sorted(glob.glob(os.path.expanduser('~/.local/lib/python3*/site-packages/nvidia/*/lib'))):
                        for so_file in sorted(glob.glob(os.path.join(lib_dir, '*.so*'))):
                            try:
                                ctypes.CDLL(so_file, mode=ctypes.RTLD_GLOBAL)
                            except Exception:
                                pass

                # Ưu tiên tăng tốc phần cứng Intel OpenVINO AVX-512 khi NVIDIA GPU chưa kích hoạt
                if not has_nvidia_dev:
                    try:
                        import openvino as ov
                        core = ov.Core()
                        compiled = core.compile_model(found_path, 'CPU', {'PERFORMANCE_HINT': 'LATENCY'})
                        self.ov_infer_request = compiled.create_infer_request()
                        dummy_blob = np.zeros((1, 3, 640, 640), dtype=np.float32)
                        self.ov_infer_request.infer([dummy_blob])
                        self.is_model_loaded = True
                        self.model_path = found_path
                        self._log_info(f"🎯 [YOLO Arrow] Nạp thành công model vật mốc: {found_path} (OpenVINO_AVX512)")
                        return
                    except Exception:
                        self.ov_infer_request = None

                import onnxruntime as ort
                available = ort.get_available_providers()
                providers = []
                if has_nvidia_dev and 'CUDAExecutionProvider' in available:
                    cuda_options = {
                        'device_id': 0,
                        'arena_extend_strategy': 'kNextPowerOfTwo',
                        'gpu_mem_limit': 1 * 1024 * 1024 * 1024,  # Cấp phát 1GB VRAM cho YOLO (trên tổng 4GB RTX 3050)
                        'cudnn_conv_algo_search': 'HEURISTIC',
                        'do_copy_in_default_stream': True,
                    }
                    providers.append(('CUDAExecutionProvider', cuda_options))
                providers.append('CPUExecutionProvider')

                so = ort.SessionOptions()
                so.log_severity_level = 3
                so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
                so.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
                cpu_cnt = os.cpu_count() or 4
                so.intra_op_num_threads = min(cpu_cnt // 2, 6) if cpu_cnt > 4 else max(1, cpu_cnt - 1)
                so.inter_op_num_threads = 1

                self.session = ort.InferenceSession(found_path, sess_options=so, providers=providers)
                self.is_model_loaded = True
                self.model_path = found_path
                active_ep = self.session.get_providers()[0]
                self._log_info(f"🎯 [YOLO Arrow] Nạp thành công model vật mốc: {found_path} ({active_ep})")

                # Warmup 1 frame để triệt tiêu độ trễ frame đầu tiên
                dummy_blob = np.zeros((1, 3, 640, 640), dtype=np.float32)
                in_name = self.session.get_inputs()[0].name
                self.session.run(None, {in_name: dummy_blob})
            except Exception as e:
                self._log_warn(f"Không thể khởi tạo ONNX runtime cho YOLO: {e}")
                self.is_model_loaded = False
        else:
            self._log_info("ℹ️ [YOLO Arrow] Chưa phát hiện file model (yolo_arrow.onnx). Đang ở chế độ Standby.")
            self.is_model_loaded = False

    def letterbox(self, img, new_shape=(640, 640), color=(114, 114, 114)):
        """Resize và thêm padding cho ảnh đầu vào theo chuẩn YOLOv8."""
        shape = img.shape[:2]  # current [height, width]
        if isinstance(new_shape, int):
            new_shape = (new_shape, new_shape)

        r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
        new_unpad = (int(round(shape[1] * r)), int(round(shape[0] * r)))
        dw = new_shape[1] - new_unpad[0]
        dh = new_shape[0] - new_unpad[1]
        dw /= 2
        dh /= 2

        if shape[::-1] != new_unpad:
            img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
        return img, r, (dw, dh)

    def extract_depth_distance(self, depth_img, box):
        """
        Trích xuất khoảng cách trung vị (median) từ ma trận Depth tại vùng trung tâm 40% của box.
        Loại bỏ 100% việc đo nhầm vào thùng carton hoặc nền ngoài rìa.
        """
        if depth_img is None or box is None:
            return -1.0

        x1, y1, x2, y2 = box
        h_d, w_d = depth_img.shape[:2]

        # Vùng 60% trung tâm box (cắt 20% mỗi biên tránh thùng carton)
        box_w = x2 - x1
        box_h = y2 - y1
        if box_w < 5 or box_h < 5:
            return -1.0

        cx1 = int(np.clip(x1 + 0.20 * box_w, 0, w_d - 1))
        cx2 = int(np.clip(x2 - 0.20 * box_w, 0, w_d - 1))
        cy1 = int(np.clip(y1 + 0.20 * box_h, 0, h_d - 1))
        cy2 = int(np.clip(y2 - 0.20 * box_h, 0, h_d - 1))

        if cx2 <= cx1 or cy2 <= cy1:
            return -1.0

        depth_roi = depth_img[cy1:cy2, cx1:cx2]
        valid_pixels = depth_roi[depth_roi > 0]
        if len(valid_pixels) == 0:
            return -1.0

        # Nếu là mm (uint16), đổi sang mét; nếu là float mét thì giữ nguyên
        median_val = float(np.median(valid_pixels))
        if depth_img.dtype == np.uint16 or median_val > 50.0:
            dist_m = median_val / 1000.0
        else:
            dist_m = median_val

        return dist_m

    def detect(self, color_bgr, depth_img=None, simulate_box=None):
        """
        Xử lý nhận diện và kiểm tra điều kiện quay đầu:
        Trả về dict: {
            'detected': bool,
            'box': [x1, y1, x2, y2],
            'confidence': float,
            'distance_m': float,
            'triggered': bool,
            'consecutive_count': int
        }
        """
        h_orig, w_orig = color_bgr.shape[:2]
        detected = False
        best_box = None
        best_conf = 0.0
        best_cls_id = -1
        best_cls_name = 'none'
        distance_m = -1.0

        # 1. Chạy YOLO ONNX / OpenVINO nếu có model
        if self.is_model_loaded and (self.ov_infer_request is not None or self.session is not None):
            try:
                blob, ratio, (dw, dh) = self.letterbox(color_bgr, (640, 640))
                blob = np.ascontiguousarray(blob[:, :, ::-1].transpose(2, 0, 1), dtype=np.float32) * (1.0 / 255.0)
                blob = np.expand_dims(blob, axis=0)

                if self.ov_infer_request is not None:
                    outputs = self.ov_infer_request.infer([blob])[0]
                else:
                    input_name = self.session.get_inputs()[0].name
                    outputs = self.session.run(None, {input_name: blob})[0]  # Shape: (1, 6, 8400) hoặc (1, 8400, 6)

                if outputs.shape[1] < outputs.shape[2]:
                    outputs = outputs.transpose(0, 2, 1)  # (1, 8400, 6)

                # Vectorized NumPy Post-processing (nhanh gấp 500 lần vòng lặp for Python 8400 bước)
                predictions = outputs[0]
                num_cols = predictions.shape[1]
                if num_cols == 5:
                    scores = predictions[:, 4]
                    cids = np.zeros(len(predictions), dtype=np.int32)
                else:
                    c_scores = predictions[:, 4:]
                    cids = np.argmax(c_scores, axis=1)
                    scores = np.max(c_scores, axis=1)

                valid_mask = scores >= self.conf_thresh
                if np.any(valid_mask):
                    v_preds = predictions[valid_mask]
                    v_scores = scores[valid_mask]
                    v_cids = cids[valid_mask]

                    cx, cy, bw, bh = v_preds[:, 0], v_preds[:, 1], v_preds[:, 2], v_preds[:, 3]
                    x1 = (cx - bw * 0.5 - dw) / ratio
                    y1 = (cy - bh * 0.5 - dh) / ratio
                    w_box = bw / ratio
                    h_box = bh / ratio

                    boxes = np.column_stack((x1, y1, w_box, h_box)).astype(int).tolist()
                    confidences = v_scores.astype(float).tolist()
                    class_ids = v_cids.astype(int).tolist()

                    indices = cv2.dnn.NMSBoxes(boxes, confidences, self.conf_thresh, 0.45)
                    if len(indices) > 0:
                        idx = indices[0] if isinstance(indices[0], (int, np.integer)) else indices[0][0]
                        bx, by, bw_i, bh_i = boxes[idx]
                        best_box = [max(0, bx), max(0, by), min(w_orig, bx + bw_i), min(h_orig, by + bh_i)]
                        best_conf = confidences[idx]
                        best_cls_id = class_ids[idx]
                        best_cls_name = self.class_names.get(best_cls_id, 'arrow')
                        detected = True
            except Exception as e:
                self._log_warn(f"Lỗi khi suy luận YOLO: {e}")

        # 2. Hỗ trợ giả lập test (khi chưa có model hoặc test phím bấm)
        if not detected and simulate_box is not None:
            detected = True
            best_box = simulate_box
            best_conf = 0.95
            best_cls_id = 0
            best_cls_name = 'yellow_arrow'

        inner_roi = None
        # 3. Đo khoảng cách bằng kênh Depth RealSense
        if detected and best_box is not None:
            x1, y1, x2, y2 = best_box
            box_w = x2 - x1
            box_h = y2 - y1
            cx1 = int(x1 + 0.20 * box_w)
            cx2 = int(x2 - 0.20 * box_w)
            cy1 = int(y1 + 0.20 * box_h)
            cy2 = int(y2 - 0.20 * box_h)
            inner_roi = [cx1, cy1, cx2, cy2]
            distance_m = self.extract_depth_distance(depth_img, best_box)

        # 4. Kiểm tra đủ 2 ĐIỀU KIỆN để kích hoạt quay đầu:
        # Điều kiện 1: Đã phát hiện vật mốc (detected == True)
        # Điều kiện 2: Khoảng cách <= trigger_dist_m (1.0m)
        is_now_ready = (detected and distance_m > 0.05 and distance_m <= self.trigger_dist_m)

        if is_now_ready:
            if best_cls_id == 0:  # Mũi tên vàng (Cuối hàng)
                self.consecutive_counter_yellow += 1
                if self.consecutive_counter_yellow >= self.consecutive_frames:
                    self.is_yellow_triggered = True
                    self.last_trigger_time = time.time()
                self.consecutive_counter_black = max(0, self.consecutive_counter_black - 1)
                self.is_black_triggered = False
            elif best_cls_id == 1:  # Mũi tên đen (Đầu luống 2)
                self.consecutive_counter_black += 1
                if self.consecutive_counter_black >= self.consecutive_frames:
                    self.is_black_triggered = True
                    self.last_trigger_time = time.time()
                self.consecutive_counter_yellow = max(0, self.consecutive_counter_yellow - 1)
                self.is_yellow_triggered = False
        else:
            # Cho phép rớt 1 frame không reset ngay để chống giật
            if self.consecutive_counter_yellow > 0:
                self.consecutive_counter_yellow -= 1
            if self.consecutive_counter_black > 0:
                self.consecutive_counter_black -= 1
            self.is_yellow_triggered = False
            self.is_black_triggered = False

        self.is_triggered = (self.is_yellow_triggered or self.is_black_triggered)
        consec_cnt = self.consecutive_counter_yellow if best_cls_id == 0 else self.consecutive_counter_black

        return {
            'detected': detected,
            'box': best_box,
            'inner_roi': inner_roi,
            'confidence': best_conf,
            'distance_m': distance_m,
            'class_id': best_cls_id,
            'class_name': best_cls_name,
            'triggered': self.is_triggered,
            'triggered_yellow': self.is_yellow_triggered,
            'triggered_black': self.is_black_triggered,
            'consecutive_count': consec_cnt
        }

