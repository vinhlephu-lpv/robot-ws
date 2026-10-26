#!/usr/bin/env python3
"""
Tool Thu Thập Dữ Liệu Ảnh Huấn Luyện YOLO (Biển Báo Mũi Tên Quay Đầu)
-------------------------------------------------------------------
Cách dùng:
  python3 scripts/collect_arrow_dataset.py

Phím tắt:
  [SPACE] : Chụp 1 tấm ảnh
  [C]     : Bật/Tắt chế độ chụp liên tục tự động (mỗi 0.5s chụp 1 tấm)
  [Q]/ESC : Lưu và Thoát
"""

import os
import sys
import time
import cv2

SAVE_DIR = os.path.expanduser('~/robot_ws/dataset/arrow_marker/images')
os.makedirs(SAVE_DIR, exist_ok=True)


def open_camera():
    # Thử RealSense D435 trước
    try:
        import pyrealsense2 as rs
        ctx = rs.context()
        if len(ctx.query_devices()) > 0:
            print("📷 Phát hiện Intel RealSense D435, đang mở camera...")
            pipe = rs.pipeline()
            cfg = rs.config()
            cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            pipe.start(cfg)
            return ('realsense', pipe)
    except Exception:
        pass

    # Thử V4L2 USB camera
    candidates = ['/dev/v4l/by-id/usb-Intel_R__RealSense_TM__Depth_Camera_435_Intel_R__RealSense_TM__Depth_Camera_435_825513023675-video-index0', 4, 6, 0, 1]
    for dev in candidates:
        try:
            cap = cv2.VideoCapture(dev)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret and frame is not None:
                    print(f"📷 Đã mở Camera: {dev}")
                    return ('opencv', cap)
                cap.release()
        except Exception:
            pass

    print("❌ Không mở được camera nào! Hãy cắm Intel RealSense D435 hoặc Webcam.")
    return (None, None)


def main():
    print("=" * 70)
    print("🎯 TOOL THU THẬP ẢNH HUẤN LUYỆN YOLO (MŨI TÊN QUAY ĐẦU)")
    print(f"📁 Thư mục lưu ảnh: {SAVE_DIR}")
    print("👉 Hướng camera vào biển mũi tên ở các khoảng cách từ 0.5m đến 3.0m")
    print("   [SPACE] : Chụp 1 tấm ảnh")
    print("   [C]     : Bật/Tắt chụp liên tục tự động")
    print("   [Q]     : Thoát")
    print("=" * 70)

    cam_type, cam_obj = open_camera()
    if cam_type is None:
        sys.exit(1)

    # Đếm số ảnh hiện có trong thư mục
    existing_files = [f for f in os.listdir(SAVE_DIR) if f.endswith(('.jpg', '.png'))]
    count = len(existing_files)
    auto_mode = False
    last_auto_capture = 0.0

    window_name = "Thu Thap Anh YOLO - Nhan [SPACE] De Chup | [C] Tu Dong | [Q] Thoat"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 960, 720)

    try:
        while True:
            frame = None
            if cam_type == 'realsense':
                frames = cam_obj.wait_for_frames(1000)
                color = frames.get_color_frame()
                if color:
                    import numpy as np
                    frame = np.asanyarray(color.get_data())
            else:
                ret, frame = cam_obj.read()

            if frame is None:
                time.sleep(0.02)
                continue

            disp = frame.copy()
            h, w = disp.shape[:2]

            # Xử lý chụp tự động
            now = time.time()
            if auto_mode and (now - last_auto_capture >= 0.5):
                last_auto_capture = now
                count += 1
                fname = f"arrow_{count:04d}.jpg"
                cv2.imwrite(os.path.join(SAVE_DIR, fname), frame)
                print(f"📸 [Tự động] Đã lưu: {fname} (Tổng: {count} ảnh)")

            # Vẽ thanh thông tin trực quan trên ảnh
            cv2.rectangle(disp, (0, 0), (w, 50), (20, 20, 20), -1)
            target_str = " (Đạt mục tiêu 100 ảnh!)" if count >= 100 else f" (Cần thêm {100 - count} ảnh)"
            status_color = (0, 255, 0) if count >= 100 else (0, 215, 255)
            cv2.putText(disp, f"DA CHUP: {count} ANH {target_str}", (15, 32),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, status_color, 2, cv2.LINE_AA)

            mode_str = "AUTO (0.5s/anh)" if auto_mode else "MANUAL [SPACE]"
            mode_color = (0, 0, 255) if auto_mode else (255, 255, 255)
            cv2.putText(disp, f"Che do: {mode_str}", (w - 300, 32),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, mode_color, 2, cv2.LINE_AA)

            # Vẽ hồng tâm canh giữa
            cv2.circle(disp, (w // 2, h // 2), 6, (0, 255, 255), -1)
            cv2.drawMarker(disp, (w // 2, h // 2), (0, 255, 255), cv2.MARKER_CROSS, 40, 1)

            cv2.imshow(window_name, disp)
            key = cv2.waitKey(1) & 0xFF

            if key == 32:  # Phím SPACE -> Chụp 1 ảnh
                count += 1
                fname = f"arrow_{count:04d}.jpg"
                cv2.imwrite(os.path.join(SAVE_DIR, fname), frame)
                print(f"📸 [Chụp tay] Đã lưu: {fname} (Tổng: {count} ảnh)")
                # Hiệu ứng chớp trắng
                flash = np.ones_like(disp) * 255
                cv2.imshow(window_name, flash)
                cv2.waitKey(40)

            elif key in (ord('c'), ord('C')):
                auto_mode = not auto_mode
                state = "BẬT" if auto_mode else "TẮT"
                print(f"⚙️ Chế độ chụp tự động: {state}")

            elif key in (ord('q'), ord('Q'), 27):
                break

    finally:
        if cam_type == 'realsense':
            try:
                cam_obj.stop()
            except Exception:
                pass
        elif cam_type == 'opencv':
            cam_obj.release()
        cv2.destroyAllWindows()

    print("=" * 70)
    print(f"🎉 Hoàn tất! Đã thu thập tổng cộng {count} tấm ảnh trong:")
    print(f"👉 {SAVE_DIR}")
    print("Bước tiếp theo: Tải thư mục này lên Roboflow.com để gán nhãn khung bao 'arrow'!")
    print("=" * 70)


if __name__ == '__main__':
    main()
