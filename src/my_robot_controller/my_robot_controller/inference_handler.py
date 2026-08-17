import os
import cv2
import numpy as np


class InferenceHandler:
    def __init__(self, model_path: str, mask_threshold: float = 0.35, input_size: tuple = (384, 384), use_hsv_mask: bool = False):
        self.model_path = model_path
        self.mask_threshold = mask_threshold  # kept for compatibility, not used in dynamic mode
        self.input_size = input_size
        self.use_hsv_mask = use_hsv_mask
        self.session = None
        if not self.use_hsv_mask:
            self.load_model()

    def load_model(self):
        import onnxruntime
        print(f"[InferenceHandler] Loading ONNX model from {self.model_path}")
        providers = ['CPUExecutionProvider']
        if 'CUDAExecutionProvider' in onnxruntime.get_available_providers():
            providers.insert(0, 'CUDAExecutionProvider')
        self.session = onnxruntime.InferenceSession(self.model_path, providers=providers)
        print(f"[InferenceHandler] Model loaded successfully. Providers: {self.session.get_providers()}")

    def preprocess_image(self, bgr_image: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Converts BGR image to normalized RGB tensor."""
        rgb = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
        resized = cv2.resize(rgb, (self.input_size[1], self.input_size[0]), interpolation=cv2.INTER_LINEAR)
        img = resized.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))   # HWC -> CHW
        input_tensor = np.expand_dims(img, axis=0)  # CHW -> BCHW
        return input_tensor, rgb

    def predict_mask(self, input_tensor: np.ndarray, enable_tta: bool = True) -> np.ndarray:
        """Runs model inference and applies Sigmoid to get probability mask."""
        ort_inputs = {self.session.get_inputs()[0].name: input_tensor}
        ort_outs = self.session.run(None, ort_inputs)
        logits = ort_outs[0][0, 0]   # (H, W)
        probs = 1.0 / (1.0 + np.exp(-logits))

        if enable_tta:
            input_tensor_flip = np.flip(input_tensor, axis=3)
            ort_inputs_flip = {self.session.get_inputs()[0].name: input_tensor_flip}
            ort_outs_flip = self.session.run(None, ort_inputs_flip)
            logits_flip = ort_outs_flip[0][0, 0]
            probs_flip = 1.0 / (1.0 + np.exp(-logits_flip))
            probs_flip = np.flip(probs_flip, axis=1)
            probs = 0.5 * (probs + probs_flip)

        return probs

    # ------------------------------------------------------------------
    # Optimized Lane Tracking & Confidence (from Train CNN Main/inference.py)
    # ------------------------------------------------------------------
    def find_lane_center(self, mask: np.ndarray) -> float:
        """Finds the X coordinate of the lane center using row scanning."""
        if mask.ndim == 3:
            mask = mask[..., 0]

        h, w = mask.shape
        image_center = (w - 1) / 2.0

        # Bottom ROI (40% height)
        roi_ratio = 0.4
        y0 = int(h * (1.0 - roi_ratio))
        roi = mask[y0:, :]

        binary = (roi >= self.mask_threshold).astype(np.uint8)

        # Close morphological gaps
        kernel = np.ones((3, 3), dtype=np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=1)

        centers: list[float] = []
        min_lane_width = max(12, int(0.06 * w))
        center_idx = int(round(image_center))

        for row in binary:
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
                    centers.append((left + right) / 2.0)
                    continue

            has_row = row.astype(bool)
            best_seg = None  # (width, -dist_to_center, center_x)

            j = 0
            while j < w:
                if has_row[j]:
                    j += 1
                    continue

                start = j
                while j < w and not has_row[j]:
                    j += 1
                end = j - 1

                # Must be bounded by crops on both sides
                if start == 0 or end == w - 1:
                    continue
                if not (has_row[start - 1] and has_row[end + 1]):
                    continue

                center_x = (start + end) / 2.0
                width = end - start + 1
                dist_to_center = abs(center_x - image_center)
                key = (width, -dist_to_center, center_x)
                if best_seg is None or key > best_seg:
                    best_seg = key

            if best_seg is not None:
                centers.append(best_seg[2])

        if centers:
            return float(np.clip(np.median(centers), 0.0, w - 1.0))

        # Fallback: left/right boundary of all crop pixels in ROI
        col_sum = binary.sum(axis=0)
        cols = np.where(col_sum > 0)[0]
        if len(cols) >= 2:
            left = float(cols[0])
            right = float(cols[-1])
            return (left + right) / 2.0

        return image_center

    def compute_steering_angle(self, mask: np.ndarray, max_angle_deg: float = 3.5) -> float:
        """Translates offset of lane center to steering angle (in degrees)."""
        _, w = mask.shape[:2]
        lane_center = self.find_lane_center(mask)
        image_center = (w - 1) / 2.0

        offset = (lane_center - image_center) / max(image_center, 1.0)
        angle = float(np.clip(offset * max_angle_deg, -max_angle_deg, max_angle_deg))
        return angle

    def compute_row_confidence(self, mask_prob: np.ndarray, roi_ratio: float = 0.25) -> float:
        """Computes a confidence score based on crop row lane detection density."""
        if mask_prob.ndim == 3:
            mask_prob = mask_prob[..., 0]
            
        h, w = mask_prob.shape
        y0 = int(h * (1.0 - roi_ratio))
        roi = mask_prob[y0:, :]
        
        binary = (roi >= self.mask_threshold).astype(np.uint8)
        kernel = np.ones((3, 3), dtype=np.uint8)
        binary_closed = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=1)
        
        valid_rows = 0
        total_rows = binary_closed.shape[0]
        center_idx = int(w / 2)
        min_lane_width = max(12, int(0.06 * w))
        
        for row in binary_closed:
            cols = np.flatnonzero(row > 0)
            if cols.size < 2:
                continue
                
            left_cols = cols[cols < center_idx]
            right_cols = cols[cols > center_idx]
            
            # Check 1: Lane contains the image center
            if left_cols.size > 0 and right_cols.size > 0:
                left = float(left_cols[-1])
                right = float(right_cols[0])
                if (right - left) >= min_lane_width:
                    valid_rows += 1
                    continue
            
            # Check 2: Shifted lane (lane doesn't contain center, but we find a valid crop-bounded empty space)
            has_row = row.astype(bool)
            j = 0
            best_seg_width = 0
            while j < w:
                if has_row[j]:
                    j += 1
                    continue
                start = j
                while j < w and not has_row[j]:
                    j += 1
                end = j - 1
                
                # Must be bounded by crop pixels on both sides
                if start == 0 or end == w - 1:
                    continue
                if not (has_row[start - 1] and has_row[end + 1]):
                    continue
                    
                width = end - start + 1
                if width > best_seg_width:
                    best_seg_width = width
            
            if best_seg_width >= min_lane_width:
                valid_rows += 1
                    
        # Confidence Score: 1.0 if valid lane is detected on >50% of ROI rows
        row_score = valid_rows / (total_rows * 0.5 + 1e-6)
        confidence = float(np.clip(row_score, 0.0, 1.0))
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
            mask_prob = (mask / 255.0).astype(np.float32)
            mask_prob = cv2.resize(mask_prob, (self.input_size[1], self.input_size[0]), interpolation=cv2.INTER_LINEAR)
        else:
            input_tensor, _ = self.preprocess_image(bgr_image)
            mask_prob = self.predict_mask(input_tensor, enable_tta=False)
            
            # Domain Adaptation Check: if CNN confidence is low (due to Gazebo domain gap), fallback to HSV green segmentation
            confidence = self.compute_row_confidence(mask_prob)
            if confidence < 0.30:
                hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
                lower_green = np.array([35, 30, 30])
                upper_green = np.array([85, 255, 255])
                mask_hsv = cv2.inRange(hsv, lower_green, upper_green)
                hsv_prob = (mask_hsv / 255.0).astype(np.float32)
                hsv_prob = cv2.resize(hsv_prob, (self.input_size[1], self.input_size[0]), interpolation=cv2.INTER_LINEAR)
                
                hsv_conf = self.compute_row_confidence(hsv_prob)
                if hsv_conf > confidence:
                    mask_prob = hsv_prob

        confidence = self.compute_row_confidence(mask_prob)
        heading_error = self.compute_steering_angle(mask_prob, max_angle_deg=max_angle_deg)
        lane_center = self.find_lane_center(mask_prob)
        
        _, w = mask_prob.shape[:2]
        image_center = (w - 1) / 2.0
        lane_offset = (lane_center - image_center) / max(image_center, 1.0)
        
        self.latest_mask = mask_prob
        
        return heading_error, lane_offset, lane_center, confidence
