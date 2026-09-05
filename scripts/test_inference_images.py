#!/usr/bin/env python3
"""
Interactive CNN Inference & Drive Simulation Tool.
Simulates 100% of the real robot's perception and control pipeline:
- ONNX 512x512 Inference
- Lane Center & Confidence Extraction
- Sliding Mode Controller (SMC) Steering Calculation
- Skid-Steer Differential Kinematics & ESP32 Serial Protocol ('V rpm_L rpm_R\n')
- Interactive Multi-Image File Picker (GUI Dialog or Terminal Input)
- Visual Real-time HUD Dashboard (Matplotlib TkAgg interactive window)

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

def pick_images_gui_or_cli(args_paths):
    """Select images via CLI arguments, GUI File Dialog, or Terminal Prompt."""
    image_paths = []

    # 1. If paths passed via command line
    if args_paths:
        for p in args_paths:
            p = os.path.abspath(os.path.expanduser(p))
            if os.path.isdir(p):
                valid_exts = ('.jpg', '.jpeg', '.png', '.bmp', '.webp')
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
    default_sample = os.path.join(WS_DIR, 'src', 'my_robot_controller', 'models', 'sample_carton_field.jpg')
    print("\n" + "=" * 65)
    print("  ⌨️  NHẬP ĐƯỜNG DẪN ẢNH HOẶC THƯ MỤC TRÊN LAPTOP")
    print(f"  [Nhấn Enter để dùng ảnh mẫu bãi cỏ thùng carton]:")
    print("=" * 65)
    try:
        user_input = input("👉 Đường dẫn: ").strip().strip('"').strip("'")
    except (EOFError, KeyboardInterrupt):
        user_input = ""

    if not user_input:
        if os.path.exists(default_sample):
            return [default_sample]
        else:
            print("❌ Không tìm thấy ảnh mặc định.")
            return []

    p = os.path.abspath(os.path.expanduser(user_input))
    if os.path.isdir(p):
        valid_exts = ('.jpg', '.jpeg', '.png', '.bmp', '.webp')
        return sorted([os.path.join(p, f) for f in os.listdir(p) if f.lower().endswith(valid_exts)])
    elif os.path.isfile(p):
        return [p]

    print(f"❌ Đường dẫn không tồn tại: {p}")
    return []

def draw_hud(bgr_orig, mask_prob, lane_center, conf, heading_err, lane_off, 
             v_lin, w_ang, rpm_l, rpm_r, esp_cmd, inference_ms, img_idx, total_imgs, filename):
    """
    Renders 3-panel vehicle HUD:
    [Original Frame] | [CNN Segmentation Mask] | [AI Augmented Guidance HUD]
    """
    h_orig, w_orig = bgr_orig.shape[:2]
    vis_h, vis_w = 512, 512

    # Panel 1: Original resized
    p1 = cv2.resize(bgr_orig, (vis_w, vis_h))

    # Panel 2: Colored Segmentation Mask
    mask_512 = cv2.resize(mask_prob, (vis_w, vis_h))
    bin_mask = (mask_512 >= 0.35).astype(np.uint8)
    
    p2 = np.zeros((vis_h, vis_w, 3), dtype=np.uint8)
    p2[:] = [25, 45, 25] # Nền cỏ xanh sẫm
    p2[bin_mask > 0] = [230, 180, 0] # Hàng thùng màu vàng/cyan
    
    # Boundary contour overlay
    contours, _ = cv2.findContours(bin_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(p2, contours, -1, (255, 255, 255), 2)

    # Panel 3: Live Vehicle HUD
    p3 = p1.copy()
    overlay = p1.copy()
    overlay[bin_mask > 0] = [255, 200, 0] # Phủ màu hàng thùng
    p3 = cv2.addWeighted(p1, 0.65, overlay, 0.35, 0)

    # Guide Lines
    img_center_x = int((vis_w - 1) * 0.5)
    target_center_x = int(np.clip(lane_center, 0, vis_w - 1))

    # 1. Image Center (Mũi xe / Tâm trục robot) - Nét đứt màu xanh lá
    for y in range(0, vis_h, 20):
        cv2.line(p3, (img_center_x, y), (img_center_x, min(y + 10, vis_h)), (0, 255, 0), 2)

    # 2. Detected Row Center (Tim luống do AI phát hiện) - Đường nét liền vàng/xanh ngọc
    cv2.line(p3, (target_center_x, vis_h - 1), (target_center_x, int(vis_h * 0.40)), (0, 215, 255), 3)

    # 3. Steering Target Vector (Mũi tên bẻ lái từ tâm xe đến tim luống)
    arrow_y = int(vis_h * 0.75)
    cv2.arrowedLine(p3, (img_center_x, arrow_y), (target_center_x, arrow_y), (0, 0, 255), 3, tipLength=0.25)

    # 4. Dashboard Bar trên cùng
    dash_h = 95
    dash_overlay = p3[0:dash_h, :].copy()
    dash_bg = np.zeros_like(dash_overlay)
    p3[0:dash_h, :] = cv2.addWeighted(dash_overlay, 0.25, dash_bg, 0.75, 0)

    is_tracking = (conf >= 0.30)
    status_text = "TRACKING (BÁM LUỐNG TỰ HÀNH)" if is_tracking else "LOST / EOR (HẾT HÀNG)"
    status_color = (0, 255, 0) if is_tracking else (0, 0, 255)

    cv2.putText(p3, f"STATUS: {status_text}", (15, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.62, status_color, 2, cv2.LINE_AA)
    cv2.putText(p3, f"Confidence: {conf*100:.1f}% | Offset: {lane_off:+.3f} | Heading: {heading_err:+.2f} deg", 
                (15, 47), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(p3, f"Speed: {v_lin:.2f} m/s | w: {w_ang:+.3f} rad/s | L: {rpm_l:.1f} RPM | R: {rpm_r:.1f} RPM", 
                (15, 68), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (200, 255, 200), 1, cv2.LINE_AA)
    cv2.putText(p3, f"Serial ESP32: {repr(esp_cmd).strip()} | Latency: {inference_ms:.1f} ms ({1000/max(1, inference_ms):.1f} FPS)", 
                (15, 87), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 215, 255), 1, cv2.LINE_AA)

    # Add labels to panels
    cv2.putText(p1, f"[1] CAMERA GOC ({w_orig}x{h_orig})", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    cv2.putText(p2, f"[2] CNN SEGMENTATION MASK (512x512)", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    cv2.putText(p3, f"[3] REAL-TIME VEHICLE HUD ({img_idx+1}/{total_imgs})", (15, dash_h + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    # Combine 3 panels horizontally
    combined = np.hstack((p1, p2, p3))
    return combined

def launch_matplotlib_gallery(results_data, out_dir):
    """Launches an interactive Matplotlib viewer with next/prev buttons and hotkeys."""
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Button

    current_idx = [0]
    total_imgs = len(results_data)

    fig, ax = plt.subplots(figsize=(15, 6))
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
        ax.set_title(f"[{idx+1}/{total_imgs}] {data['filename']}  |  Confidence: {data['conf']*100:.1f}%  |  Góc lái: {data['heading']:+.2f}°  |  Lệnh ESP32: {repr(data['esp_cmd']).strip()}", fontsize=11, fontweight='bold', color='#113355')
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
    ax_prev = plt.axes([0.35, 0.02, 0.13, 0.06])
    ax_next = plt.axes([0.52, 0.02, 0.13, 0.06])
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
    parser.add_argument('--model', default='', help="Đường dẫn file model ONNX (mặc định crop_row_cnn_best_final.onnx)")
    parser.add_argument('--out-dir', default='inference_results', help="Thư mục lưu ảnh kết quả")
    args = parser.parse_args()

    # 1. Resolve Model Path
    model_path = args.model
    if not model_path or not os.path.exists(model_path):
        model_path = os.path.join(WS_DIR, 'src', 'my_robot_controller', 'models', 'crop_row_cnn_best_final.onnx')
    if not os.path.exists(model_path):
        print(f"❌ Không tìm thấy model ONNX tại: {model_path}")
        sys.exit(1)

    print("\n" + "=" * 70)
    print("  🚀 MÔ PHỎNG 100% SUY LUẬN AI CNN & ĐIỀU KHIỂN XE THẬT (ROBOT_WS)")
    print("=" * 70)
    print(f"  🧠 Model ONNX sử dụng : {os.path.relpath(model_path, WS_DIR)}")

    # 2. Select Images
    image_paths = pick_images_gui_or_cli(args.paths)
    if not image_paths:
        print("❌ Không có ảnh nào được chọn. Đang thoát...")
        sys.exit(0)

    print(f"  📸 Đã nạp thành công {len(image_paths)} hình ảnh để chạy suy luận.")
    os.makedirs(args.out_dir, exist_ok=True)

    # 3. Initialize Controller & Inference
    handler = InferenceHandler(
        model_path=model_path,
        input_size=(512, 512),
        mask_threshold=0.35,
        use_hsv_mask=False
    )
    
    controller = TrackingControllerSMC()
    controller.initialize(
        lambda_smc=2.0,
        k_smc=3.5,
        eta_smc=0.6,
        phi_smc=0.5,
        linear_speed=0.30,       # Chuẩn xe thật: 0.30 m/s
        turn_angular_speed=0.60  # Giới hạn góc bẻ lái: 0.60 rad/s
    )

    wheel_d = 0.20
    wheel_base = 0.58
    wheel_circ = math.pi * wheel_d

    results_data = []

    print("\n" + "-" * 70)
    print("  TIẾN HÀNH SUY LUẬN & TÍNH TOÁN ĐỘNG HỌC CHO TỪNG ẢNH:")
    print("-" * 70)

    for idx, img_path in enumerate(image_paths):
        bgr = cv2.imread(img_path)
        if bgr is None:
            print(f"⚠️ Bỏ qua file lỗi không đọc được: {img_path}")
            continue

        # Exact Real-World Pipeline
        t0 = time.time()
        input_tensor, _ = handler.preprocess_image(bgr)
        mask_prob = handler.predict_mask(input_tensor, enable_tta=False)
        confidence = handler.compute_row_confidence(mask_prob)
        lane_center = handler.find_lane_center(mask_prob)
        _, w_mask = mask_prob.shape[:2]
        img_center = (w_mask - 1) * 0.5
        lane_offset = (lane_center - img_center) / max(img_center, 1.0)
        heading_error = float(np.clip(lane_offset * 7.0, -7.0, 7.0))
        inference_ms = (time.time() - t0) * 1000

        # SMC Tracking Calculation (Đánh giá đáp ứng bẻ lái chuẩn xác theo từng ảnh tĩnh)
        controller.reset()
        cmd = controller.compute_command(heading_error, dt_actual=0.067)
        v_lin = cmd["linear_velocity"]
        w_ang = cmd["angular_velocity"]

        # Differential Drive Kinematics
        v_left = v_lin - (w_ang * wheel_base / 2.0)
        v_right = v_lin + (w_ang * wheel_base / 2.0)
        rpm_left = (v_left / wheel_circ) * 60.0
        rpm_right = (v_right / wheel_circ) * 60.0
        esp_cmd = f"V {rpm_left:.1f} {rpm_right:.1f}\n"

        filename = os.path.basename(img_path)
        print(f"[{idx+1:02d}/{len(image_paths):02d}] {filename:<25} | Conf: {confidence*100:5.1f}% | Tâm: {lane_center:5.1f}px | Lái: {heading_error:+5.2f}° | Lệnh ESP32: {repr(esp_cmd).strip():<16} ({inference_ms:.1f}ms)")

        # Render HUD
        hud_canvas = draw_hud(
            bgr, mask_prob, lane_center, confidence, heading_error, lane_offset,
            v_lin, w_ang, rpm_left, rpm_right, esp_cmd, inference_ms, idx, len(image_paths), filename
        )

        # Automatically save result
        save_path = os.path.join(args.out_dir, f"hud_{os.path.splitext(filename)[0]}.png")
        cv2.imwrite(save_path, hud_canvas)

        results_data.append({
            "filename": filename,
            "canvas": hud_canvas,
            "conf": confidence,
            "heading": heading_error,
            "esp_cmd": esp_cmd
        })

    print("-" * 70)
    print(f"  💾 Toàn bộ {len(results_data)} ảnh HUD kết quả đã được lưu tại: {os.path.abspath(args.out_dir)}/")

    # 4. Display Matplotlib Interactive Gallery if DISPLAY is available
    if os.environ.get("DISPLAY") and results_data:
        try:
            launch_matplotlib_gallery(results_data, args.out_dir)
        except Exception as e:
            print(f"  [Ghi chú] Giao diện hiển thị: {e}")

    print("\n" + "=" * 70)
    print(f"  ✅ ĐÃ HOÀN TẤT MÔ PHỎNG SUY LUẬN AI CNN!")
    print("=" * 70 + "\n")

if __name__ == '__main__':
    main()
