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
        num_threads: int = 0
    ):
        self.model_path = model_path
        self.mask_threshold = mask_threshold  # kept for compatibility, used in binary segmentation
        self.input_size = input_size
        self.use_hsv_mask = use_hsv_mask
        self.num_threads = num_threads
        self.session = None
        self.input_name = None
        self.output_names = None
        self.latest_mask = None

        # Pre-allocated memory buffers for optimal throughput
        self._input_buffer = np.zeros((1, 3, self.input_size[0], self.input_size[1]), dtype=np.float32)
        self._scale_inv_255 = np.float32(1.0 / 255.0)
        self._morph_kernel = np.ones((3, 3), dtype=np.uint8)

        if not self.use_hsv_mask:
            self.load_model()

    def load_model(self):
        import onnxruntime
        print(f"[InferenceHandler] Loading ONNX model from {self.model_path}")

        available = onnxruntime.get_available_providers()
        providers = []
        for ep in ['CUDAExecutionProvider', 'TensorrtExecutionProvider', 'OpenVINOExecutionProvider']:
            if ep in available:
                providers.append(ep)
        providers.append('CPUExecutionProvider')

        # Session optimization options
        so = onnxruntime.SessionOptions()
        so.graph_optimization_level = onnxruntime.GraphOptimizationLevel.ORT_ENABLE_ALL
        so.execution_mode = onnxruntime.ExecutionMode.ORT_SEQUENTIAL
        so.enable_mem_pattern = True
        so.enable_cpu_mem_arena = True

        # CPU Thread configuration
        if self.num_threads > 0:
            so.intra_op_num_threads = self.num_threads
        else:
            cpu_cnt = os.cpu_count() or 4
            # On 4-core systems (e.g. Raspberry Pi), use 3 threads leaving 1 for ROS & LiDAR.
            # On 6-12+ core systems, cap at 6 to avoid diminishing returns.
            if cpu_cnt <= 4:
                so.intra_op_num_threads = max(1, cpu_cnt - 1)
            else:
                so.intra_op_num_threads = min(cpu_cnt // 2, 6)
        so.inter_op_num_threads = 1

        self.session = onnxruntime.InferenceSession(self.model_path, sess_options=so, providers=providers)
        self.input_name = self.session.get_inputs()[0].name
        self.output_names = [self.session.get_outputs()[0].name]
        print(f"[InferenceHandler] Model loaded successfully. Providers: {self.session.get_providers()} | Threads: {so.intra_op_num_threads} | Input: {self.input_size}")

    def preprocess_image(self, bgr_image: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Converts BGR image to normalized RGB tensor using pre-allocated buffer."""
        rgb = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
        resized = cv2.resize(rgb, (self.input_size[1], self.input_size[0]), interpolation=cv2.INTER_LINEAR)
        float_img = resized.astype(np.float32) * self._scale_inv_255
        self._input_buffer[0] = np.transpose(float_img, (2, 0, 1))
        return self._input_buffer, rgb

    def predict_mask(self, input_tensor: np.ndarray, enable_tta: bool = False) -> np.ndarray:
        """Runs model inference and applies numerically stable Sigmoid to get probability mask."""
        ort_outs = self.session.run(self.output_names, {self.input_name: input_tensor})
        logits = ort_outs[0][0, 0]   # (H, W)
        # Fast numerically stable Sigmoid
        probs = 1.0 / (1.0 + np.exp(-np.clip(logits, -20.0, 20.0)))

        if enable_tta:
            input_tensor_flip = np.flip(input_tensor, axis=3)
            ort_outs_flip = self.session.run(self.output_names, {self.input_name: input_tensor_flip})
            logits_flip = ort_outs_flip[0][0, 0]
            probs_flip = 1.0 / (1.0 + np.exp(-np.clip(logits_flip, -20.0, 20.0)))
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

        # Bottom ROI (70% height to capture full carton boxes / crop rows)
        roi_ratio = 0.70
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
        half_lane_px = 0.22 * w  # Nửa bề rộng hành lang 1.0m (~112.6px trên ảnh 512px)
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary)

        # Lọc bỏ các cụm nhiễu nhỏ (< 200 pixel)
        valid_comps = []
        for i in range(1, num_labels):
            area = stats[i, cv2.CC_STAT_AREA]
            if area >= 200:
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

            # Trường hợp 2: Chỉ có 1 hàng đơn lẻ (hoặc hàng chéo chiếm ưu thế)
            # Áp dụng quy tắc phối cảnh hội tụ điểm tụ:
            # - Hàng có chân ở bên trái mũi xe (bot_x < center_idx) hoặc dấu sắc [/] -> HÀNG BÊN TRÁI
            #   => Lối đi xe chạy nằm ở bên PHẢI hàng này (+ half_lane)
            # - Hàng có chân ở bên phải mũi xe (bot_x >= center_idx) hoặc dấu huyền [\] -> HÀNG BÊN PHẢI
            #   => Lối đi xe chạy nằm ở bên TRÁI hàng này (- half_lane)
            main_comp = max(valid_comps, key=lambda c: c["area"])
            if main_comp["bot_x"] < center_idx:
                return float(np.clip(main_comp["cx"] + half_lane_px, 0.0, w - 1.0))
            else:
                return float(np.clip(main_comp["cx"] - half_lane_px, 0.0, w - 1.0))

        return image_center

    def compute_steering_angle(self, mask: np.ndarray, max_angle_deg: float = 3.5) -> float:
        """Translates offset of lane center to steering angle (in degrees)."""
        _, w = mask.shape[:2]
        lane_center = self.find_lane_center(mask)
        image_center = (w - 1) * 0.5

        offset = (lane_center - image_center) / max(image_center, 1.0)
        angle = float(np.clip(offset * max_angle_deg, -max_angle_deg, max_angle_deg))
        return angle

    def compute_row_confidence(self, mask_prob: np.ndarray, roi_ratio: float = 0.70) -> float:
        """Computes a confidence score based on crop row lane detection density."""
        if mask_prob.ndim == 3:
            mask_prob = mask_prob[..., 0]
            
        h, w = mask_prob.shape
        y0 = int(h * (1.0 - roi_ratio))
        roi = mask_prob[y0:, :]
        
        binary = (roi >= self.mask_threshold).astype(np.uint8)
        binary_closed = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, self._morph_kernel, iterations=1)
        
        valid_rows = 0
        sampled_rows = binary_closed[::2] if h >= 384 else binary_closed
        total_rows = sampled_rows.shape[0]
        center_idx = int(w * 0.5)
        min_lane_width = max(12, int(0.06 * w))
        
        for row in sampled_rows:
            cols = np.flatnonzero(row > 0)
            if cols.size < 2:
                continue
                
            left_cols = cols[cols < center_idx]
            right_cols = cols[cols > center_idx]
            
            # Check 1: Lane contains the image center
            if left_cols.size > 0 and right_cols.size > 0:
                if (float(right_cols[0]) - float(left_cols[-1])) >= min_lane_width:
                    valid_rows += 1
                    continue
            
            # Check 2: Shifted lane
            diffs = np.diff(np.pad(row.astype(np.int16), (1, 1), 'constant', constant_values=1))
            starts = np.where(diffs == -1)[0]
            ends = np.where(diffs == 1)[0] - 1
            valid = (starts > 0) & (ends < w - 1)
            if np.any(valid):
                widths = ends[valid] - starts[valid] + 1
                if np.max(widths) >= min_lane_width:
                    valid_rows += 1
                    
        # Confidence Score: 1.0 if valid lane is detected on >50% of ROI rows
        row_score = valid_rows / (total_rows * 0.5 + 1e-6)
        confidence = float(np.clip(row_score, 0.0, 1.0))

        # Hỗ trợ bám 1 hàng (Single-Row Confidence):
        # Nếu chưa đủ 2 hàng (confidence thấp) nhưng có 1 hàng cây/thùng rõ nét (diện tích lớn)
        if confidence < 0.35:
            num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary_closed)
            max_area = max([stats[i, cv2.CC_STAT_AREA] for i in range(1, num_labels)], default=0)
            if max_area >= 1200:
                # 1 hàng rất rõ nét -> Đạt mức confidence 0.50 ~ 0.60 (Bám 1 hàng an toàn)
                single_score = min(0.60, 0.40 + (max_area / 10000.0) * 0.20)
                confidence = max(confidence, single_score)

        return confidence

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

        # Lane center and heading error without duplicate computations
        lane_center = self.find_lane_center(mask_prob)
        _, w = mask_prob.shape[:2]
        image_center = (w - 1) * 0.5
        lane_offset = (lane_center - image_center) / max(image_center, 1.0)
        heading_error = float(np.clip(lane_offset * max_angle_deg, -max_angle_deg, max_angle_deg))
        
        self.latest_mask = mask_prob
        return heading_error, lane_offset, lane_center, confidence
