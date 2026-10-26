import os
import cv2
import numpy as np


class InferenceHandler:
    def __init__(
        self,
        model_path: str,
        mask_threshold: float = 0.35,
        input_size: tuple[int, int] = (512, 512),
        use_hsv_mask: bool = False,
        num_threads: int = 0,
        roi_ratio: float = 0.80
    ):
        self.model_path = model_path
        self.mask_threshold = mask_threshold  # kept for compatibility, used in binary segmentation
        self.input_size = input_size
        self.use_hsv_mask = use_hsv_mask
        self.num_threads = num_threads
        self.roi_ratio = float(roi_ratio)
        self.session = None
        self.ov_infer_request = None
        self.input_name = None
        self.output_names = None
        self.latest_mask = None
        # True nếu model đã nhúng sẵn Sigmoid (output là xác suất [0,1]) -> KHÔNG áp Sigmoid lần 2
        # (model v2 crop_row_cnn_v2.onnx). False với model cũ xuất logits.
        self.output_is_prob = False
        self.active_provider = 'CPUExecutionProvider'

        # Pre-allocated memory buffers for optimal throughput
        self._input_buffer = np.zeros((1, 3, self.input_size[0], self.input_size[1]), dtype=np.float32)
        self._scale_inv_255 = np.float32(1.0 / 255.0)
        self._morph_kernel = np.ones((3, 3), dtype=np.uint8)

        if not self.use_hsv_mask:
            self.load_model()

    def load_model(self):
        # Kiểm tra xem phần cứng GPU NVIDIA có sẵn sàng và node /dev/nvidia* có tồn tại không
        has_nvidia_dev = os.path.exists('/dev/nvidia0') or os.path.exists('/dev/nvidiactl')
        if has_nvidia_dev:
            # Tự động nạp thư viện CUDA/cuDNN nếu được cài đặt qua pip (nvidia-* packages)
            import glob
            import ctypes
            for lib_dir in sorted(glob.glob(os.path.expanduser('~/.local/lib/python3*/site-packages/nvidia/*/lib'))):
                for so_file in sorted(glob.glob(os.path.join(lib_dir, '*.so*'))):
                    try:
                        ctypes.CDLL(so_file, mode=ctypes.RTLD_GLOBAL)
                    except Exception:
                        pass

        # ƯU TIÊN TĂNG TỐC INTEL OPENVINO AVX-512 VNNI (Khi NVIDIA GPU chưa có driver hoặc chạy trên CPU Intel)
        if not has_nvidia_dev:
            try:
                import openvino as ov
                ov_model_path = self.model_path
                # Tự động ưu tiên bản INT8 QDQ đã hiệu chuẩn (nhanh gấp 4.5 lần trên Intel AVX-512 VNNI: ~25ms = 40 FPS)
                if ov_model_path.endswith('crop_row_cnn_v2.onnx'):
                    int8_cand = ov_model_path.replace('crop_row_cnn_v2.onnx', 'crop_row_cnn_v2_int8.onnx')
                    if os.path.exists(int8_cand):
                        ov_model_path = int8_cand
                    else:
                        ws_int8 = os.path.expanduser('~/robot_ws/src/my_robot_controller/models/crop_row_cnn_v2_int8.onnx')
                        if os.path.exists(ws_int8):
                            ov_model_path = ws_int8

                core = ov.Core()
                ov_config = {'PERFORMANCE_HINT': 'LATENCY'}
                if self.num_threads > 0:
                    ov_config['INFERENCE_NUM_THREADS'] = str(self.num_threads)
                compiled = core.compile_model(ov_model_path, 'CPU', ov_config)
                self.ov_infer_request = compiled.create_infer_request()
                in_shape = compiled.input(0).partial_shape
                if in_shape.rank.is_static and len(in_shape) == 4:
                    if in_shape[2].is_static and in_shape[3].is_static:
                        self.input_size = (in_shape[2].get_length(), in_shape[3].get_length())
                        self._input_buffer = np.zeros((1, 3, self.input_size[0], self.input_size[1]), dtype=np.float32)

                is_int8 = 'int8' in os.path.basename(ov_model_path).lower()
                self.active_provider = 'OpenVINO_AVX512_INT8' if is_int8 else 'OpenVINO_AVX512_FP32'
                print(f"🚀 [InferenceHandler] Đã kích hoạt tăng tốc phần cứng Intel OpenVINO ({self.active_provider}): {os.path.basename(ov_model_path)}")

                dummy_in = np.zeros((1, 3, self.input_size[0], self.input_size[1]), dtype=np.float32)
                warmup_out = self.ov_infer_request.infer([dummy_in])[0]
                self.output_is_prob = bool(warmup_out.min() >= -0.01 and warmup_out.max() <= 1.01)
                if self.output_is_prob:
                    print(f"🎯 [InferenceHandler] Xác nhận mô hình đã nhúng Sigmoid (Output: [0.0, 1.0]). Giữ nguyên xác suất gốc.")
                else:
                    print(f"🎯 [InferenceHandler] Mô hình xuất Logits thô (min={warmup_out.min():.2f}). Tự động tính Sigmoid.")
                return
            except Exception as e:
                self.ov_infer_request = None

        import onnxruntime

        # Nếu có GPU NVIDIA khả dụng mà đường dẫn đang trỏ vào bản _int8.onnx (chứa nút ConvInteger của CPU),
        # tự động chuyển sang bản gốc FP32 (crop_row_cnn_v2.onnx) để chạy 100% trên nhân CUDA (~17ms = 58 FPS)
        if has_nvidia_dev and self.model_path.endswith('_int8.onnx'):
            fp32_cand = self.model_path.replace('_int8.onnx', '.onnx')
            if os.path.exists(fp32_cand):
                self.model_path = fp32_cand
            else:
                v2_cand = os.path.join(os.path.dirname(self.model_path), 'crop_row_cnn_v2.onnx')
                if os.path.exists(v2_cand):
                    self.model_path = v2_cand

        available = onnxruntime.get_available_providers()
        providers = []
        if has_nvidia_dev and 'CUDAExecutionProvider' in available:
            cuda_options = {
                'device_id': 0,
                'arena_extend_strategy': 'kNextPowerOfTwo',
                'gpu_mem_limit': 2 * 1024 * 1024 * 1024,  # Cấp phát 2GB VRAM cho CNN (trên tổng 4GB RTX 3050)
                'cudnn_conv_algo_search': 'HEURISTIC',
                'do_copy_in_default_stream': True,
            }
            providers.append(('CUDAExecutionProvider', cuda_options))
        elif has_nvidia_dev and 'TensorrtExecutionProvider' in available:
            try:
                import tensorrt
                providers.append('TensorrtExecutionProvider')
            except ImportError:
                pass
        if 'OpenVINOExecutionProvider' in available:
            providers.append('OpenVINOExecutionProvider')
        providers.append('CPUExecutionProvider')

        # Session optimization options
        so = onnxruntime.SessionOptions()
        so.log_severity_level = 3
        so.graph_optimization_level = onnxruntime.GraphOptimizationLevel.ORT_ENABLE_ALL
        so.execution_mode = onnxruntime.ExecutionMode.ORT_SEQUENTIAL
        so.enable_mem_pattern = True
        so.enable_cpu_mem_arena = True

        # CPU Thread configuration
        if self.num_threads > 0:
            so.intra_op_num_threads = self.num_threads
        else:
            cpu_cnt = os.cpu_count() or 4
            # Trên hệ thống 6-12 core (như i5-11400H 12 threads), dùng 6 threads tối ưu nhất
            if cpu_cnt <= 4:
                so.intra_op_num_threads = max(1, cpu_cnt - 1)
            else:
                so.intra_op_num_threads = min(cpu_cnt // 2, 6)
        so.inter_op_num_threads = 1

        self.session = onnxruntime.InferenceSession(self.model_path, sess_options=so, providers=providers)
        input_meta = self.session.get_inputs()[0]
        self.input_name = input_meta.name
        self.output_names = [self.session.get_outputs()[0].name]

        # Tự động đồng bộ kích thước input_size nếu model có fixed shape (ví dụ 384x384)
        if len(input_meta.shape) == 4:
            in_h, in_w = input_meta.shape[2], input_meta.shape[3]
            if isinstance(in_h, int) and isinstance(in_w, int) and in_h > 0 and in_w > 0:
                self.input_size = (in_h, in_w)
                self._input_buffer = np.zeros((1, 3, self.input_size[0], self.input_size[1]), dtype=np.float32)

        active_ep = self.session.get_providers()[0]
        self.active_provider = active_ep
        if 'CUDA' in active_ep:
            print(f"🚀 [InferenceHandler] Đã kích hoạt tăng tốc phần cứng NVIDIA GPU: {active_ep}")
        else:
            print(f"ℹ️ [InferenceHandler] Đang chạy với provider: {active_ep} (threads={so.intra_op_num_threads})")

        # Warmup session & Tự động phát hiện mô hình đã nhúng Sigmoid hay xuất Logits thô
        dummy_in = np.zeros((1, 3, self.input_size[0], self.input_size[1]), dtype=np.float32)
        try:
            warmup_out = self.session.run(self.output_names, {self.input_name: dummy_in})[0]
            # Nếu output nằm hoàn toàn trong [0.0, 1.0] -> Mô hình đã nhúng Sigmoid (như crop_row_cnn_v2)
            self.output_is_prob = bool(warmup_out.min() >= -0.01 and warmup_out.max() <= 1.01)
            if self.output_is_prob:
                print(f"🎯 [InferenceHandler] Xác nhận mô hình đã nhúng Sigmoid (Output: [0.0, 1.0]). Giữ nguyên xác suất gốc.")
            else:
                print(f"🎯 [InferenceHandler] Mô hình xuất Logits thô (min={warmup_out.min():.2f}). Tự động tính Sigmoid.")
        except Exception as e:
            print(f"⚠️ [InferenceHandler] Lỗi warmup mô hình: {e}")

    def preprocess_image(self, bgr_image: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Converts BGR image to normalized RGB tensor using pre-allocated buffer."""
        rgb = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
        resized = cv2.resize(rgb, (self.input_size[1], self.input_size[0]), interpolation=cv2.INTER_LINEAR)
        float_img = resized.astype(np.float32) * self._scale_inv_255
        self._input_buffer[0] = np.transpose(float_img, (2, 0, 1))
        return self._input_buffer, rgb

    def predict_mask(self, input_tensor: np.ndarray, enable_tta: bool = False) -> np.ndarray:
        """Runs model inference and returns calibrated probability mask."""
        if self.ov_infer_request is not None:
            raw_out = self.ov_infer_request.infer([input_tensor])[0][0, 0]
        else:
            ort_outs = self.session.run(self.output_names, {self.input_name: input_tensor})
            raw_out = ort_outs[0][0, 0]   # (H, W)

        if self.output_is_prob:
            probs = np.clip(raw_out, 0.0, 1.0)
        else:
            # Fast numerically stable Sigmoid cho mô hình xuất logits thô
            probs = 1.0 / (1.0 + np.exp(-np.clip(raw_out, -20.0, 20.0)))

        if enable_tta:
            input_tensor_flip = np.flip(input_tensor, axis=3)
            if self.ov_infer_request is not None:
                raw_flip = self.ov_infer_request.infer([input_tensor_flip])[0][0, 0]
            else:
                ort_outs_flip = self.session.run(self.output_names, {self.input_name: input_tensor_flip})
                raw_flip = ort_outs_flip[0][0, 0]
            if self.output_is_prob:
                probs_flip = np.clip(raw_flip, 0.0, 1.0)
            else:
                probs_flip = 1.0 / (1.0 + np.exp(-np.clip(raw_flip, -20.0, 20.0)))
            probs_flip = np.flip(probs_flip, axis=1)
            probs = 0.5 * (probs + probs_flip)

        return probs

    # ------------------------------------------------------------------
    # Optimized Lane Tracking & Confidence
    # ------------------------------------------------------------------
    def find_lane_center(self, mask: np.ndarray) -> float:
        """Finds the X coordinate of the lane center using optimized row scanning."""
        if mask.ndim == 3:
            mask = mask[..., 0]

        h, w = mask.shape
        image_center = (w - 1) * 0.5

        # Bottom ROI (80% height to capture full carton boxes / crop rows)
        roi_ratio = getattr(self, 'roi_ratio', 0.80)
        y0 = int(h * (1.0 - roi_ratio))
        roi = mask[y0:, :]

        binary = (roi >= self.mask_threshold).astype(np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, self._morph_kernel, iterations=1)

        centers: list[float] = []
        min_lane_width = max(12, int(0.06 * w))
        center_idx = int(round(image_center))

        # Sample every 2nd row for 2x speedup with high fidelity at 512x512
        sampled_rows = binary[::2] if h >= 384 else binary

        for row in sampled_rows:
            cols = np.flatnonzero(row > 0)
            if cols.size < 2:
                continue

            # Prioritize left/right boundaries around center
            left_cols = cols[cols < center_idx]
            right_cols = cols[cols > center_idx]

            if left_cols.size > 0 and right_cols.size > 0:
                left = float(left_cols[-1])
                right = float(right_cols[0])
                if (right - left) >= min_lane_width:
                    centers.append((left + right) * 0.5)
                    continue

            # Fast run-length segment detection bounded by crops
            diffs = np.diff(np.pad(row.astype(np.int16), (1, 1), 'constant', constant_values=1))
            starts = np.where(diffs == -1)[0]
            ends = np.where(diffs == 1)[0] - 1
            valid = (starts > 0) & (ends < w - 1)
            if np.any(valid):
                widths = ends[valid] - starts[valid] + 1
                # Bắt buộc bề rộng khoảng trống phải >= min_lane_width (loại bỏ khe hở nhiễu)
                valid_w = valid & (widths >= min_lane_width)
                if np.any(valid_w):
                    v_starts = starts[valid_w]
                    v_ends = ends[valid_w]
                    centers_x = (v_starts + v_ends) * 0.5
                    dists = np.abs(centers_x - image_center)
                    best_idx = np.argmin(dists)
                    centers.append(float(centers_x[best_idx]))

        # Cần ít nhất 5 hàng quét tìm thấy lối đi 2 bên mới coi là tìm thấy luống hoàn chỉnh
        if len(centers) >= 5:
            return float(np.clip(np.median(centers), 0.0, w - 1.0))

        # Phân tích liên thông 2D (Connected Components Analysis) khi chỉ bắt được 1 hàng hoặc hàng chéo:
        # Bán bề rộng xe thực tế: ~0.30m (track width 0.58m, bánh xe 0.63m).
        # Hành lang 1.0m cần chừa tâm xe cách hàng cây ít nhất 0.50m (tương ứng ~0.38*w trên ảnh).
        half_lane_px = 0.38 * w  # Nửa bề rộng hành lang an toàn (~146px trên ảnh 384px, ~195px trên ảnh 512px)
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary)

        # Lọc bỏ các cụm nhiễu nhỏ (tự động co giãn theo độ phân giải h x w)
        area_scale = (h * w) / (512.0 * 512.0)
        min_comp_area = max(20, int(200 * area_scale))
        valid_comps = []
        for i in range(1, num_labels):
            area = stats[i, cv2.CC_STAT_AREA]
            if area >= min_comp_area:
                pts = np.argwhere(labels == i)
                y_max = pts[:, 0].max()
                y_min = pts[:, 0].min()
                # Tọa độ chân hàng ở cận cảnh đáy ảnh (ngay trước mũi xe)
                bot_pts = pts[pts[:, 0] >= y_max - 25, 1]
                bot_x = float(bot_pts.mean()) if len(bot_pts) > 0 else float(centroids[i, 0])
                # Tọa độ ngọn hàng ở xa
                top_pts = pts[pts[:, 0] <= y_min + 25, 1]
                top_x = float(top_pts.mean()) if len(top_pts) > 0 else float(centroids[i, 0])
                valid_comps.append({
                    "area": area,
                    "cx": float(centroids[i, 0]),
                    "cy": float(centroids[i, 1]),
                    "bot_x": bot_x,
                    "top_x": top_x,
                    "dx_up": top_x - bot_x,
                    "bbox": stats[i]
                })

        if valid_comps:
            left_comps = [c for c in valid_comps if c["bot_x"] < center_idx]
            right_comps = [c for c in valid_comps if c["bot_x"] >= center_idx]

            # Trường hợp 1: Có các cụm rõ rệt nằm ở cả 2 bên tâm ảnh (Đủ 2 hàng Trái & Phải)
            if left_comps and right_comps:
                c_left = max(left_comps, key=lambda c: c["cx"])["cx"]
                c_right = min(right_comps, key=lambda c: c["cx"])["cx"]
                if (c_right - c_left) >= min_lane_width:
                    return float((c_left + c_right) * 0.5)

            # Trường hợp 2: Chỉ có 1 hàng đơn lẻ (hoặc hàng chéo chiếm ưu thế khi xe chạy sát mép)
            # Áp dụng phối hợp tọa độ chân hàng (bot_x), tâm (cx) và độ dốc phối cảnh (dx_up = top_x - bot_x):
            # - Hàng BÊN TRÁI: Thường nằm ở nửa trái, hoặc có độ dốc nghiêng sang phải về điểm tụ xa (dx_up > 0)
            #   => Lối đi giữa hàng nằm ở bên PHẢI hàng này (+ half_lane)
            # - Hàng BÊN PHẢI: Thường nằm ở nửa phải, hoặc có độ dốc nghiêng sang trái về điểm tụ xa (dx_up < 0)
            #   => Lối đi giữa hàng nằm ở bên TRÁI hàng này (- half_lane)
            main_comp = max(valid_comps, key=lambda c: c["area"])
            bot_x = main_comp["bot_x"]
            dx_up = main_comp["dx_up"]
            
            is_left_row = False
            if bot_x < 0.40 * w:
                is_left_row = True
            elif bot_x > 0.60 * w:
                is_left_row = False
            else:
                # Gốc hàng nằm ở vùng giữa (do xe đang đi chéo áp sát hàng):
                # Dùng hướng nghiêng phối cảnh về điểm tụ
                if dx_up > 3.0:
                    is_left_row = True   # Nghiêng [/] -> Hàng bên trái
                elif dx_up < -3.0:
                    is_left_row = False  # Nghiêng [\] -> Hàng bên phải
                else:
                    is_left_row = (main_comp["cx"] < center_idx)

            if is_left_row:
                target_center = bot_x + half_lane_px
            else:
                target_center = bot_x - half_lane_px

            return float(np.clip(target_center, 0.0, w - 1.0))

        return image_center

    def compute_vehicle_clearance_bias(self, mask: np.ndarray, max_angle_deg: float) -> float:
        """
        Bảo toàn góc nhìn luống: lane_center đã tính toán chính xác tâm lối đi giữa 2 hàng.
        Không cộng thêm sai số giả lập cận cảnh để tránh xe bị giật lệch tâm về một phía.
        """
        return 0.0

    def compute_steering_angle(self, mask: np.ndarray, max_angle_deg: float = 3.5) -> float:
        """Translates offset of lane center to steering angle (in degrees), factoring in vehicle footprint."""
        _, w = mask.shape[:2]
        lane_center = self.find_lane_center(mask)
        image_center = (w - 1) * 0.5

        offset = (lane_center - image_center) / max(image_center, 1.0)
        base_angle = float(np.clip(offset * max_angle_deg, -max_angle_deg, max_angle_deg))
        clearance_bias = self.compute_vehicle_clearance_bias(mask, max_angle_deg)
        return float(np.clip(base_angle + clearance_bias, -max_angle_deg, max_angle_deg))

    def compute_row_confidence(self, mask_prob: np.ndarray, roi_ratio: float = None) -> float:
        """Computes a confidence score based on crop row lane detection density."""
        if mask_prob.ndim == 3:
            mask_prob = mask_prob[..., 0]
            
        h, w = mask_prob.shape
        if roi_ratio is None:
            roi_ratio = getattr(self, 'roi_ratio', 0.80)
        y0 = int(h * (1.0 - roi_ratio))
        roi = mask_prob[y0:, :]
        
        binary = (roi >= self.mask_threshold).astype(np.uint8)
        binary_closed = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, self._morph_kernel, iterations=1)
        roi_h = binary_closed.shape[0]

        # Kiểm tra mật độ vật thể ở vùng cận & trung cảnh (65% đáy ROI ngay trước mũi xe):
        # Khi xe đã chạy lố qua cặp thùng cuối cùng, vùng đáy ảnh sẽ trống trơn (< 1.2% diện tích),
        # giúp triệt tiêu hiện tượng bóng nắng/vật thể ở xa phía trên làm treo confidence không chịu báo hết hàng.
        near_roi = binary_closed[int(roi_h * 0.35):, :]
        near_fg_ratio = float(np.count_nonzero(near_roi)) / max(1.0, float(near_roi.size))

        valid_rows = 0
        sampled_rows = binary_closed[::2] if h >= 384 else binary_closed
        total_rows = sampled_rows.shape[0]
        center_idx = int(w * 0.5)
        min_lane_width = max(12, int(0.06 * w))
        min_side_px = max(3, int(0.01 * w))

        for row in sampled_rows:
            cols = np.flatnonzero(row > 0)
            if cols.size < (2 * min_side_px):
                continue

            left_cols = cols[cols < center_idx]
            right_cols = cols[cols > center_idx]

            # Check 1: Lane contains the image center (cần tối thiểu min_side_px mỗi bên để loại nhiễu chấm lẻ)
            if left_cols.size >= min_side_px and right_cols.size >= min_side_px:
                if (float(right_cols[0]) - float(left_cols[-1])) >= min_lane_width:
                    valid_rows += 1
                    continue

            # Check 2: Shifted lane bounded by real segments (mỗi vách dày >= min_side_px)
            diffs = np.diff(np.pad(row.astype(np.int16), (1, 1), 'constant', constant_values=1))
            starts = np.where(diffs == -1)[0]
            ends = np.where(diffs == 1)[0] - 1
            valid = (starts >= min_side_px) & (ends < w - min_side_px)
            if np.any(valid):
                widths = ends[valid] - starts[valid] + 1
                if np.max(widths) >= min_lane_width:
                    valid_rows += 1

        # Confidence Score: 1.0 if valid lane is detected on >50% of ROI rows
        row_score = valid_rows / (total_rows * 0.5 + 1e-6)
        confidence = float(np.clip(row_score, 0.0, 1.0))

        # Hỗ trợ nhận diện 2 bên vách luống khi các thùng đặt thưa:
        # Chỉ giữ confidence >= 0.35 khi thực sự có cụm thùng ở CẢ 2 BÊN Trái và Phải của tâm ảnh.
        # Tuyệt đối KHÔNG bơm confidence lên > 30% khi chỉ còn 1 chiếc thùng lẻ loi ở 1 bên tại cuối hàng
        # (tránh việc đặt xe ở cuối hàng chỉ thấy 1 thùng bên phải lại tưởng còn hàng rồi tự bẻ lái sang trái!).
        if confidence < 0.35 and near_fg_ratio >= 0.018:
            area_scale = (h * w) / (512.0 * 512.0)
            min_single_area = max(250, int(1200 * area_scale))
            num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary_closed)
            has_left_wall = False
            has_right_wall = False
            best_valid_area = 0
            for i in range(1, num_labels):
                area = stats[i, cv2.CC_STAT_AREA]
                comp_top = stats[i, cv2.CC_STAT_TOP]
                comp_h = stats[i, cv2.CC_STAT_HEIGHT]
                comp_bot = comp_top + comp_h
                cx = float(centroids[i, 0])
                if area >= min_single_area and comp_bot >= int(0.45 * roi_h) and comp_h >= int(0.18 * roi_h):
                    if cx < center_idx:
                        has_left_wall = True
                    else:
                        has_right_wall = True
                    if area > best_valid_area:
                        best_valid_area = area
            if has_left_wall and has_right_wall and best_valid_area > 0:
                pair_score = min(0.65, 0.38 + (best_valid_area / max(1.0, 6000.0 * area_scale)) * 0.25)
                confidence = max(confidence, pair_score)

        # Nếu cận cảnh trước mũi xe đã trống hoàn toàn (< 1.2% diện tích): ép giảm confidence để báo hết hàng (EOR)
        if near_fg_ratio < 0.012:
            confidence *= (near_fg_ratio / 0.012)

        return float(np.clip(confidence, 0.0, 1.0))

    def process_image(self, bgr_image: np.ndarray, max_angle_deg: float = 3.5) -> tuple[float, float, float, float]:
        """
        Runs full pre-processing, CNN inference (or HSV segmentation), lane center calculation,
        and confidence estimation.
        Returns (heading_error, lane_offset, lane_center, confidence).
        """
        if self.use_hsv_mask:
            # Color-based segmentation for simulation
            hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
            lower_green = np.array([35, 30, 30])
            upper_green = np.array([85, 255, 255])
            mask = cv2.inRange(hsv, lower_green, upper_green)
            mask_prob = (mask * self._scale_inv_255).astype(np.float32)
            mask_prob = cv2.resize(mask_prob, (self.input_size[1], self.input_size[0]), interpolation=cv2.INTER_LINEAR)
            confidence = self.compute_row_confidence(mask_prob)
        else:
            input_tensor, _ = self.preprocess_image(bgr_image)
            mask_prob = self.predict_mask(input_tensor, enable_tta=False)
            confidence = self.compute_row_confidence(mask_prob)
            
            # Domain Adaptation Check: Only fallback to HSV green segmentation in simulation (when use_hsv_mask is enabled)
            # In real world, ground is green grass and boxes are white/cardboard, so never overwrite CNN with green HSV
            if self.use_hsv_mask and confidence < 0.30:
                hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
                lower_green = np.array([35, 30, 30])
                upper_green = np.array([85, 255, 255])
                mask_hsv = cv2.inRange(hsv, lower_green, upper_green)
                hsv_prob = (mask_hsv * self._scale_inv_255).astype(np.float32)
                hsv_prob = cv2.resize(hsv_prob, (self.input_size[1], self.input_size[0]), interpolation=cv2.INTER_LINEAR)
                
                hsv_conf = self.compute_row_confidence(hsv_prob)
                if hsv_conf > confidence:
                    mask_prob = hsv_prob
                    confidence = hsv_conf

        _, w = mask_prob.shape[:2]
        image_center = (w - 1) * 0.5

        # Khi không có luống cây phía trước (confidence < 0.30):
        # Khoá góc lái CNN = 0.0° (chạy thẳng tuyệt đối), tránh nhiễu cỏ/thùng lẻ ở rìa kéo lệch xe!
        if confidence < 0.30:
            self.latest_mask = mask_prob
            return 0.0, 0.0, float(image_center), confidence

        # Lane center and heading error factoring in vehicle footprint clearance
        lane_center = self.find_lane_center(mask_prob)
        lane_offset = (lane_center - image_center) / max(image_center, 1.0)
        base_heading_error = float(np.clip(lane_offset * max_angle_deg, -max_angle_deg, max_angle_deg))
        
        # Trừ hao kích thước xe thật ở cận cảnh (Vehicle Footprint Clearance Bias)
        clearance_bias = self.compute_vehicle_clearance_bias(mask_prob, max_angle_deg)
        heading_error = float(np.clip(base_heading_error + clearance_bias, -max_angle_deg, max_angle_deg))
        
        self.latest_mask = mask_prob
        return heading_error, lane_offset, lane_center, confidence
