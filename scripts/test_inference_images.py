#!/usr/bin/env python3
"""
Interactive CNN Inference & Drive Simulation Tool.
Simulates 100% of the real robot's perception and control pipeline:
- ONNX 384x384 INT8 Inference (tối ưu Raspberry Pi & Laptop)
- ROI 80% (cắt 20% hậu cảnh trên, tập trung cận cảnh tim luống)
- Lane Center & Confidence Extraction (mask_threshold: 0.35)
- Real Robot Steering (max_steering_angle: 14.0 deg)
- In-Place Turn Adjustment logic (dừng tiến xoay căn chỉnh khi |góc| > 1.2°)
- Sliding Mode Controller (SMC) Steering Calculation
- Skid-Steer Differential Kinematics, ESP32 Serial Protocol ('V rpm_L rpm_R\\n') & BTS7960 PWM duty cycles
- Interactive Multi-Image File Picker (GUI Dialog or Terminal Input)
- Visual Real-time 3-Panel HUD Dashboard (Camera Frame | CNN Mask with 80% ROI | Real-time Guidance HUD)

Usage:
  test-img
  test-img /path/to/folder
  test-img image1.jpg image2.jpg
"""

import os
import sys
import time
import math
import argparse
import numpy as np
import cv2

# Set workspace paths
WS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(WS_DIR, 'src', 'my_robot_controller'))

from my_robot_controller.inference_handler import InferenceHandler
from my_robot_controller.controllers import TrackingControllerSMC

def load_robot_params():
    """Tự động tải 100% cấu hình thực tế từ params_real.yaml để test-img đồng bộ hoàn toàn với xe thật."""
    yaml_path = os.path.join(WS_DIR, 'src', 'my_robot_controller', 'config', 'params_real.yaml')
    params = {
        'model_path': 'models/crop_row_cnn_best_final_int8.onnx',
        'input_height': 384,
        'input_width': 384,
        'roi_ratio': 0.80,
        'mask_threshold': 0.35,
        'linear_speed': 0.075,
        'turn_linear_speed': 0.075,
        'turn_angular_speed': 0.60,
        'turn_in_place_threshold_deg': 1.2,
        'turn_in_place_resume_deg': 0.8,
        'camera_trim_deg': 0.0,
        'lambda_smc': 2.5,
        'k_smc': 4.2,
        'eta_smc': 0.8,
        'phi_smc': 0.4,
        'max_steering_angle_deg': 14.0,
        'ema_alpha': 0.35,
        'low_confidence_threshold': 0.30,
        'high_confidence_threshold': 0.55,
        'wheel_base': 0.58,
        'wheel_d': 0.20,
        'max_linear_speed': 0.18,
        'min_duty_cycle': 22.0,
    }
    if os.path.exists(yaml_path):
        try:
            import yaml
            with open(yaml_path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
                if isinstance(data, dict):
                    cnn_p = data.get('cnn_driver_node', {}).get('ros__parameters', {})
                    for k, v in cnn_p.items():
                        if k in params:
                            params[k] = v
                    bts_p = data.get('bts7960_driver_node', {}).get('ros__parameters', {})
                    for k, v in bts_p.items():
                        if k in params:
                            params[k] = v
        except Exception as e:
            print(f"⚠️ Không đọc được params_real.yaml ({e}), dùng thông số mặc định.")
    return params

def vel_to_duty(velocity_ms: float, max_linear_speed: float = 0.18, min_duty_cycle: float = 22.0) -> float:
    """Chuyển vận tốc (m/s) -> duty cycle có dấu (-100 .. +100) theo chuẩn driver BTS7960."""
    clamped = max(-max_linear_speed, min(max_linear_speed, velocity_ms))
    raw_duty = clamped / max_linear_speed * 100.0
    if abs(raw_duty) > 0.5 and abs(raw_duty) < min_duty_cycle:
        raw_duty = min_duty_cycle if raw_duty > 0 else -min_duty_cycle
    return raw_duty

def pick_images_gui_or_cli(args_paths):
    """Select images via CLI arguments, GUI File Dialog, or Terminal Prompt."""
    image_paths = []
    valid_exts = ('.jpg', '.jpeg', '.png', '.bmp', '.webp')

    # 1. If paths passed via command line
    if args_paths:
        for p in args_paths:
            p = os.path.abspath(os.path.expanduser(p))
            if os.path.isdir(p):
                files = sorted([os.path.join(p, f) for f in os.listdir(p) if f.lower().endswith(valid_exts)])
                image_paths.extend(files)
            elif os.path.isfile(p):
                image_paths.append(p)
        if image_paths:
            return image_paths

    # 2. Try Tkinter GUI File Dialog
    if os.environ.get("DISPLAY"):
        try:
            import tkinter as tk
            from tkinter import filedialog
            root = tk.Tk()
            root.withdraw()
            root.attributes('-topmost', True)

            print("\n" + "=" * 65)
            print("  📂 ĐANG MỞ HỘP THOẠI CHỌN ẢNH TRÊN LAPTOP...")
            print("  (Bạn có thể giữ phím Ctrl hoặc Shift để chọn nhiều ảnh cùng lúc)")
            print("=" * 65)

            selected_files = filedialog.askopenfilenames(
                title="Chọn một hoặc nhiều ảnh thử nghiệm AI CNN",
                filetypes=[
                    ("Hình ảnh (*.jpg, *.png, *.jpeg, *.webp)", "*.jpg *.jpeg *.png *.bmp *.webp"),
                    ("Tất cả tập tin", "*.*")
                ]
            )
            root.destroy()

            if selected_files:
                return list(selected_files)
        except Exception as e:
            print(f"  [Ghi chú] Không thể mở hộp thoại GUI: {e}")

    # 3. Fallback to Terminal Prompt
    sample_candidates = [
        os.path.join(WS_DIR, 'scripts', 'test_images', 'field_carton_d435.jpg'),
        os.path.join(WS_DIR, 'src', 'my_robot_controller', 'models', 'sample_carton_field.jpg'),
    ]
    default_sample = next((s for s in sample_candidates if os.path.exists(s)), "")

    print("\n" + "=" * 65)
    print("  ⌨️  NHẬP ĐƯỜNG DẪN ẢNH HOẶC THƯ MỤC TRÊN LAPTOP")
    if default_sample:
        print(f"  [Nhấn Enter để dùng ảnh thực địa: {os.path.basename(default_sample)}]")
    print("=" * 65)
    try:
        user_input = input("👉 Đường dẫn: ").strip().strip('"').strip("'")
    except (EOFError, KeyboardInterrupt):
        user_input = ""

    if not user_input:
        if default_sample and os.path.exists(default_sample):
            return [default_sample]
        else:
            print("❌ Không tìm thấy ảnh mặc định.")
            return []

    p = os.path.abspath(os.path.expanduser(user_input))
    if os.path.isdir(p):
        return sorted([os.path.join(p, f) for f in os.listdir(p) if f.lower().endswith(valid_exts)])
    elif os.path.isfile(p):
        return [p]

    print(f"❌ Đường dẫn không tồn tại: {p}")
    return []

def draw_hud(bgr_orig, mask_prob, lane_center, conf, heading_err, lane_off, 
             v_lin, w_ang, rpm_l, rpm_r, duty_l, duty_r, esp_cmd, inference_ms,
             state_name, state_color, robot_params, img_idx, total_imgs, filename):
    """
    Renders 3-panel vehicle HUD:
    [Camera Gốc Thực Tế] | [Mặt Nạ CNN 384x384 (ROI 80%)] | [Dashboard Điều Khiển Xe Thật 100%]
    """
    h_orig, w_orig = bgr_orig.shape[:2]
    vis_h, vis_w = 480, 560
    roi_ratio = float(robot_params.get('roi_ratio', 0.80))
    mask_thresh = float(robot_params.get('mask_threshold', 0.35))
    max_steer = float(robot_params.get('max_steering_angle_deg', 14.0))

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
    cv2.drawContours(p2, contours, -1, (255, 255, 255), 2)

    # Vạch giới hạn ROI 80% trên Panel 2:
    # y0 = (1.0 - roi_ratio) * vis_h (ví dụ 20% trên đỉnh ảnh = cắt bỏ)
    y_roi = int(round(vis_h * (1.0 - roi_ratio)))
    # Tô tối vùng 20% trên đỉnh để biểu thị vùng bị loại bỏ khỏi nhận diện
    p2_top_shade = p2[0:y_roi, :].copy()
    p2[0:y_roi, :] = cv2.addWeighted(p2_top_shade, 0.4, np.zeros_like(p2_top_shade), 0.6, 0)
    for x in range(0, vis_w, 16):
        cv2.line(p2, (x, y_roi), (min(x + 8, vis_w), y_roi), (0, 180, 255), 2)
    cv2.putText(p2, "CUT TOP 20% (BO QUA HAU CANH)", (15, max(y_roi - 8, 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 160, 255), 1, cv2.LINE_AA)
    cv2.putText(p2, "VUNG ROI 80% DUNG SUY LUAN", (15, y_roi + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 200), 1, cv2.LINE_AA)

    # Panel 3: Live Vehicle HUD (100% Robot Driving Simulation)
    p3 = p1.copy()
    overlay = p1.copy()
    overlay[bin_mask > 0] = [255, 200, 0] # Phủ màu hàng thùng
    p3 = cv2.addWeighted(p1, 0.65, overlay, 0.35, 0)

    # Vạch ROI trên Panel 3
    for x in range(0, vis_w, 16):
        cv2.line(p3, (x, y_roi), (min(x + 8, vis_w), y_roi), (0, 215, 255), 1)

    # Guide Lines
    img_center_x = int((vis_w - 1) * 0.5)
    # Tỉ lệ scale tọa độ lane_center từ mask_prob sang vis_w
    w_mask = mask_prob.shape[1]
    target_center_x = int(np.clip(lane_center * (vis_w / float(w_mask)), 0, vis_w - 1))

    # 1. Image Center (Mũi xe / Tâm trục robot) - Nét đứt màu xanh lá
    for y in range(0, vis_h, 20):
        cv2.line(p3, (img_center_x, y), (img_center_x, min(y + 10, vis_h)), (0, 255, 0), 2)

    # 2. Detected Row Center (Tim luống do AI phát hiện) - Đường nét liền vàng/xanh ngọc
    cv2.line(p3, (target_center_x, vis_h - 1), (target_center_x, y_roi), (0, 215, 255), 3)

    # 3. Steering Target Vector (Mũi tên bẻ lái từ tâm xe đến tim luống)
    arrow_y = int(vis_h * 0.74)
    cv2.arrowedLine(p3, (img_center_x, arrow_y), (target_center_x, arrow_y), (0, 0, 255), 3, tipLength=0.22)

    # 4. Dashboard Bar trên cùng Panel 3
    dash_h = 108
    dash_overlay = p3[0:dash_h, :].copy()
    dash_bg = np.zeros_like(dash_overlay)
    p3[0:dash_h, :] = cv2.addWeighted(dash_overlay, 0.20, dash_bg, 0.80, 0)

    # Line 1: State Badge
    cv2.putText(p3, f"STATUS: {state_name}", (12, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.52, state_color, 2, cv2.LINE_AA)
    
    # Line 2: Vision Metrics
    cv2.putText(p3, f"Conf: {conf*100:4.1f}% | Offset: {lane_off:+.3f} | Lai CNN: {heading_err:+.2f} deg (Max +/-{max_steer:.0f} deg)", 
                (12, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (255, 255, 255), 1, cv2.LINE_AA)
    
    # Line 3: Kinematics & Motor RPM
    cv2.putText(p3, f"Speed: v={v_lin:.3f} m/s | w={w_ang:+.3f} rad/s | L={rpm_l:+.1f} RPM | R={rpm_r:+.1f} RPM", 
                (12, 67), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (200, 255, 200), 1, cv2.LINE_AA)
    
    # Line 4: BTS7960 PWM & ESP32 Protocol & FPS
    fps_val = 1000.0 / max(1.0, inference_ms)
    cv2.putText(p3, f"PWM: L={duty_l:+.1f}% R={duty_r:+.1f}% | ESP32: {repr(esp_cmd).strip()} | {inference_ms:.1f}ms ({fps_val:.1f} FPS)", 
                (12, 89), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 220, 255), 1, cv2.LINE_AA)

    # Add labels to top of panels
    cv2.putText(p1, f"[1] CAMERA GOC ({w_orig}x{h_orig} -> D435)", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
    cv2.putText(p2, f"[2] CNN MASK 384x384 (ROI: {int(roi_ratio*100)}%)", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
    cv2.putText(p3, f"[3] DIEU KHIEN XE THAT ({img_idx+1}/{total_imgs})", (12, dash_h + 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

    # Combine 3 panels horizontally
    combined = np.hstack((p1, p2, p3))
    return combined

def launch_matplotlib_gallery(results_data, out_dir):
    """Launches an interactive Matplotlib viewer with next/prev buttons and hotkeys."""
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Button

    current_idx = [0]
    total_imgs = len(results_data)

    fig, ax = plt.subplots(figsize=(16, 5.5))
    fig.canvas.manager.set_window_title(f"Mô Phỏng Tự Hành AI CNN Bám Luống Thùng Carton - robot_ws ({total_imgs} ảnh)")
    plt.subplots_adjust(bottom=0.12, top=0.96, left=0.01, right=0.99)
    ax.axis('off')

    # Convert first BGR to RGB
    first_rgb = cv2.cvtColor(results_data[0]["canvas"], cv2.COLOR_BGR2RGB)
    im_display = ax.imshow(first_rgb)

    def update_view():
        idx = current_idx[0]
        data = results_data[idx]
        rgb = cv2.cvtColor(data["canvas"], cv2.COLOR_BGR2RGB)
        im_display.set_data(rgb)
        ax.set_title(f"[{idx+1}/{total_imgs}] {data['filename']}  |  {data['state_name']}  |  Góc lái: {data['heading']:+.2f}°  |  Lệnh ESP32: {repr(data['esp_cmd']).strip()}", fontsize=10.5, fontweight='bold', color='#113355')
        fig.canvas.draw_idle()

    def on_next(event=None):
        if current_idx[0] < total_imgs - 1:
            current_idx[0] += 1
            update_view()

    def on_prev(event=None):
        if current_idx[0] > 0:
            current_idx[0] -= 1
            update_view()

    def on_key(event):
        if event.key in ['right', 'down', ' ', 'enter', 'n']:
            on_next()
        elif event.key in ['left', 'up', 'backspace', 'p']:
            on_prev()
        elif event.key in ['q', 'escape']:
            plt.close(fig)

    fig.canvas.mpl_connect('key_press_event', on_key)

    # Add navigation buttons
    ax_prev = plt.axes([0.36, 0.02, 0.13, 0.06])
    ax_next = plt.axes([0.51, 0.02, 0.13, 0.06])
    btn_prev = Button(ax_prev, '◀ Ảnh Trước (P)', color='#e0e0e0', hovercolor='#b0d0ff')
    btn_next = Button(ax_next, 'Ảnh Sau (N) ▶', color='#e0e0e0', hovercolor='#b0d0ff')
    btn_prev.on_clicked(on_prev)
    btn_next.on_clicked(on_next)

    update_view()
    print("\n  🖥️ ĐANG MỞ CỬA SỔ XEM TRỰC TIẾP TRÊN MÀN HÌNH LAPTOP...")
    print("     • Dùng phím Mũi Tên [Trái/Phải], [Space], hoặc click nút bấm để chuyển ảnh.")
    print("     • Nhấn [Q] hoặc đóng cửa sổ để hoàn tất.")
    plt.show()

def main():
    parser = argparse.ArgumentParser(description="Mô phỏng 100% luồng AI CNN & Điều khiển xe thật trên ảnh tĩnh")
    parser.add_argument('paths', nargs='*', help="Đường dẫn đến 1 hoặc nhiều ảnh, hoặc thư mục chứa ảnh")
    parser.add_argument('--model', default='', help="Đường dẫn file model ONNX (mặc định crop_row_cnn_best_final_int8.onnx)")
    parser.add_argument('--out-dir', default='inference_results', help="Thư mục lưu ảnh kết quả")
    args = parser.parse_args()

    # 1. Load Real Robot Parameters from params_real.yaml
    p = load_robot_params()
    
    input_h = int(p.get('input_height', 384))
    input_w = int(p.get('input_width', 384))
    roi_ratio = float(p.get('roi_ratio', 0.80))
    mask_thresh = float(p.get('mask_threshold', 0.35))
    lin_speed = float(p.get('linear_speed', 0.075))
    turn_ang_speed = float(p.get('turn_angular_speed', 0.60))
    turn_in_place_thresh = float(p.get('turn_in_place_threshold_deg', 1.2))
    turn_in_place_resume = float(p.get('turn_in_place_resume_deg', 0.8))
    camera_trim = float(p.get('camera_trim_deg', 0.0))
    max_steer = float(p.get('max_steering_angle_deg', 14.0))
    low_conf_thresh = float(p.get('low_confidence_threshold', 0.30))
    
    wheel_d = float(p.get('wheel_d', 0.20))
    wheel_base = float(p.get('wheel_base', 0.58))
    wheel_circ = math.pi * wheel_d
    max_lin_speed = float(p.get('max_linear_speed', 0.18))
    min_duty = float(p.get('min_duty_cycle', 22.0))

    # Resolve Model Path
    model_path = args.model
    if not model_path or not os.path.exists(model_path):
        candidate_names = [
            'crop_row_cnn_best_final_int8.onnx',
            'crop_row_cnn_best_final.onnx'
        ]
        for name in candidate_names:
            c = os.path.join(WS_DIR, 'src', 'my_robot_controller', 'models', name)
            if os.path.exists(c):
                model_path = c
                break

    if not model_path or not os.path.exists(model_path):
        print(f"❌ Không tìm thấy model ONNX tại: {model_path}")
        sys.exit(1)

    print("\n" + "=" * 75)
    print("  🚀 MÔ PHỎNG 100% SUY LUẬN AI CNN & ĐIỀU KHIỂN XE THẬT (ROBOT_WS)")
    print("=" * 75)
    print(f"  🧠 Model ONNX sử dụng      : {os.path.relpath(model_path, WS_DIR)}")
    print(f"  📐 Kích thước Input CNN    : {input_w}x{input_h} px (Chuẩn suy luận ~2 FPS)")
    print(f"  🎯 Vùng quan sát (ROI)     : {int(roi_ratio * 100)}% (Cắt 20% hậu cảnh trên đỉnh)")
    print(f"  ⚡ Ngưỡng phân tách Mask   : {mask_thresh}")
    print(f"  🔄 Trần góc bẻ lái         : +/-{max_steer:.1f} deg")
    print(f"  🛑 Ngưỡng dừng xoay tại chỗ: > {turn_in_place_thresh:.1f} deg (Xoay {turn_ang_speed:.2f} rad/s)")
    print(f"  🏎️ Vận tốc tiến bò thẳng   : {lin_speed:.3f} m/s (Lực kéo khỏe trên cỏ)")
    print("=" * 75)

    # 2. Select Images
    image_paths = pick_images_gui_or_cli(args.paths)
    if not image_paths:
        print("❌ Không có ảnh nào được chọn. Đang thoát...")
        sys.exit(0)

    print(f"\n  📸 Đã nạp thành công {len(image_paths)} hình ảnh để chạy suy luận.")
    os.makedirs(args.out_dir, exist_ok=True)

    # 3. Initialize Controller & Inference
    handler = InferenceHandler(
        model_path=model_path,
        input_size=(input_h, input_w),
        mask_threshold=mask_thresh,
        roi_ratio=roi_ratio,
        use_hsv_mask=False
    )
    
    controller = TrackingControllerSMC()
    controller.initialize(
        lambda_smc=float(p.get('lambda_smc', 2.5)),
        k_smc=float(p.get('k_smc', 4.2)),
        eta_smc=float(p.get('eta_smc', 0.8)),
        phi_smc=float(p.get('phi_smc', 0.4)),
        linear_speed=lin_speed,
        turn_angular_speed=turn_ang_speed
    )

    results_data = []

    print("\n" + "-" * 75)
    print("  TIẾN HÀNH SUY LUẬN & TÍNH TOÁN ĐỘNG HỌC CHO TỪNG ẢNH:")
    print("-" * 75)

    for idx, img_path in enumerate(image_paths):
        bgr = cv2.imread(img_path)
        if bgr is None:
            print(f"⚠️ Bỏ qua file lỗi không đọc được: {img_path}")
            continue

        # Exact Real-World Perception Pipeline
        t0 = time.time()
        heading_error, lane_offset, lane_center, confidence = handler.process_image(bgr, max_angle_deg=max_steer)
        inference_ms = (time.time() - t0) * 1000.0
        heading_error = float(np.clip(heading_error + camera_trim, -max_steer, max_steer))
        mask_prob = handler.latest_mask

        # Decision & Control Logic (100% matching cnn_driver.py)
        if confidence < low_conf_thresh:
            state_name = "LOST / EOR (MAT DAU / HET HANG)"
            state_color = (0, 0, 255) # Red
            v_lin = 0.0
            w_ang = 0.0
        elif abs(heading_error) > turn_in_place_thresh:
            state_name = f"DUNG TIEN - XOAY TAI CHO (|goc|={abs(heading_error):.1f}° > {turn_in_place_thresh:.1f}°)"
            state_color = (0, 215, 255) # Gold / Orange
            v_lin = 0.0
            turn_dir = -1.0 if heading_error > 0 else 1.0
            w_ang = turn_dir * turn_ang_speed
        else:
            state_name = f"TIEN BAM LUONG SMC (|goc|={abs(heading_error):.1f}° <= {turn_in_place_thresh:.1f}°)"
            state_color = (0, 255, 0) # Green
            v_lin = lin_speed
            controller.reset()
            cmd = controller.compute_command(heading_error, dt_actual=0.067)
            w_ang = float(cmd["angular_velocity"])

        # Differential Drive Kinematics
        v_left = v_lin - (w_ang * wheel_base / 2.0)
        v_right = v_lin + (w_ang * wheel_base / 2.0)
        rpm_left = (v_left / wheel_circ) * 60.0
        rpm_right = (v_right / wheel_circ) * 60.0
        esp_cmd = f"V {rpm_left:.1f} {rpm_right:.1f}\n"

        # BTS7960 Motor PWM Duty Cycle
        v_l_bts = v_left
        v_r_bts = v_right
        if v_lin > 0.03:
            min_fwd = 0.035
            min_v = min(v_l_bts, v_r_bts)
            if min_v < min_fwd:
                shift = min_fwd - min_v
                v_l_bts += shift
                v_r_bts += shift
        duty_l = vel_to_duty(v_l_bts, max_lin_speed, min_duty)
        duty_r = vel_to_duty(v_r_bts, max_lin_speed, min_duty)

        filename = os.path.basename(img_path)
        print(f"[{idx+1:02d}/{len(image_paths):02d}] {filename:<25} | Conf: {confidence*100:5.1f}% | Tam: {lane_center:5.1f}px | Lai: {heading_error:+5.2f}° | ESP: {repr(esp_cmd).strip():<15} | PWM: L={duty_l:+.0f}% R={duty_r:+.0f}% ({inference_ms:.1f}ms)")

        # Render HUD
        hud_canvas = draw_hud(
            bgr, mask_prob, lane_center, confidence, heading_error, lane_offset,
            v_lin, w_ang, rpm_left, rpm_right, duty_l, duty_r, esp_cmd, inference_ms,
            state_name, state_color, p, idx, len(image_paths), filename
        )

        # Automatically save result
        save_path = os.path.join(args.out_dir, f"hud_{os.path.splitext(filename)[0]}.png")
        cv2.imwrite(save_path, hud_canvas)

        results_data.append({
            "filename": filename,
            "canvas": hud_canvas,
            "conf": confidence,
            "heading": heading_error,
            "esp_cmd": esp_cmd,
            "state_name": state_name
        })

    print("-" * 75)
    print(f"  💾 Toàn bộ {len(results_data)} ảnh HUD kết quả đã được lưu tại: {os.path.abspath(args.out_dir)}/")

    # 4. Display Matplotlib Interactive Gallery if DISPLAY is available
    if os.environ.get("DISPLAY") and results_data:
        try:
            launch_matplotlib_gallery(results_data, args.out_dir)
        except Exception as e:
            print(f"  [Ghi chú] Giao diện hiển thị: {e}")

    print("\n" + "=" * 75)
    print(f"  ✅ ĐÃ HOÀN TẤT MÔ PHỎNG SUY LUẬN AI CNN (100% ĐỒNG BỘ XE THẬT)!")
    print("=" * 75 + "\n")

if __name__ == '__main__':
    main()
