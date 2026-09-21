#!/usr/bin/env python3
"""
Kiểm tra chẩn đoán toàn diện kết nối phần cứng Camera Intel RealSense D435.
Chạy trực tiếp trên Raspberry Pi hoặc Laptop:
  python3 scripts/test_realsense.py
  (hoặc dùng lệnh tắt: test-d435)
"""

import os
import sys
import glob
import re
import subprocess

def run_cmd(cmd):
    try:
        return subprocess.check_output(cmd, shell=True, stderr=subprocess.STDOUT).decode('utf-8').strip()
    except Exception as e:
        return ""

def main():
    print("\n" + "=" * 70)
    print("  🔍 KIỂM TRA CHẨN ĐOÁN CAMERA INTEL REALSENSE D435")
    print("=" * 70)

    # 1. Kiểm tra USB Bus
    print("\n[BƯỚC 1/4] Kiểm tra cổng USB Bus:")
    lsusb_out = run_cmd("lsusb")
    d435_found = False
    for line in lsusb_out.splitlines():
        if any(kw in line.lower() for kw in ['8086:0b07', '8086:0b3a', 'realsense', 'd435']):
            print(f"  ✅ Tìm thấy RealSense trên cổng USB: {line}")
            d435_found = True
            break
    if not d435_found:
        print("  ❌ KHÔNG TÌM THẤY Intel RealSense trên lệnh lsusb!")
        print("     👉 Vui lòng cắm lại cáp USB Type-C, ưu tiên cắm vào cổng USB 3.0 (màu xanh dương).")
    else:
        # Kiểm tra tốc độ USB (USB 2.1 hay USB 3.2)
        usb_speeds = run_cmd("lsusb -t")
        if "5000M" in usb_speeds or "10000M" in usb_speeds:
            print("  ⚡ Cổng USB đang hoạt động ở tốc độ USB 3.0 SuperSpeed (Rất tốt!).")
        else:
            print("  ℹ️ Cổng USB đang hoạt động ở tốc độ USB 2.0/2.1 (Vẫn chạy tốt chế độ màu RGB 640x480).")

    # 2. Kiểm tra V4L2 video nodes
    print("\n[BƯỚC 2/4] Kiểm tra các cổng thiết bị Video /dev/video*:")
    video_nodes = sorted(glob.glob("/dev/video*"))
    if not video_nodes:
        print("  ❌ Không tìm thấy cổng /dev/video nào trên hệ thống!")
    rs_nodes = []
    for vn in video_nodes:
        name = "Unknown"
        sys_name = f"/sys/class/video4linux/{os.path.basename(vn)}/name"
        if os.path.exists(sys_name):
            try:
                with open(sys_name) as f:
                    name = f.read().strip()
            except Exception:
                pass
        is_rs = any(kw in name.lower() for kw in ['realsense', 'd435'])
        mark = "👉 [RealSense]" if is_rs else "  "
        print(f"  {mark} {vn}: {name}")
        if is_rs:
            rs_nodes.append(vn)

    # 3. Kiểm tra udev rules & quyền truy cập video
    print("\n[BƯỚC 3/4] Kiểm tra quyền truy cập người dùng (Permissions):")
    current_user = os.environ.get("USER", "pi")
    groups_out = run_cmd("groups")
    in_video = "video" in groups_out
    in_plugdev = "plugdev" in groups_out
    print(f"  • User hiện tại   : {current_user}")
    print(f"  • Nhóm 'video'    : {'✅ ĐÃ CÓ' if in_video else '❌ THIẾU (Chạy: sudo usermod -aG video $USER)'}")
    print(f"  • Nhóm 'plugdev'  : {'✅ ĐÃ CÓ' if in_plugdev else '⚠️ THIẾU (Chạy: sudo usermod -aG plugdev $USER)'}")
    
    udev_file = "/etc/udev/rules.d/99-realsense-libusb.rules"
    if os.path.exists(udev_file):
        print(f"  • RealSense udev  : ✅ Đã cài đặt ({udev_file})")
    else:
        print(f"  • RealSense udev  : ⚠️ Chưa có udev rules (Nên chạy scripts/setup_realsense.sh)")

    # 4. Kiểm tra đọc khung hình bằng OpenCV
    print("\n[BƯỚC 4/4] Thử mở và đọc khung hình trực tiếp bằng OpenCV:")
    try:
        import cv2
        import numpy as np
    except ImportError:
        print("  ❌ Chưa cài đặt OpenCV (python3-opencv). Chạy: sudo apt install python3-opencv")
        return

    # Ưu tiên tìm symlink index0 trong /dev/v4l/by-id
    by_id = "/dev/v4l/by-id"
    test_targets = []
    if os.path.exists(by_id):
        for fn in sorted(os.listdir(by_id)):
            if any(kw in fn for kw in ['RealSense', 'realsense', 'D435', 'd435']):
                full = os.path.join(by_id, fn)
                real = os.path.realpath(full)
                m = re.search(r'video(\d+)', real)
                idx = int(m.group(1)) if m else real
                if 'index0' in fn:
                    test_targets.insert(0, (fn, idx, real))
                else:
                    test_targets.append((fn, idx, real))

    if not test_targets:
        for vn in rs_nodes:
            m = re.search(r'video(\d+)', vn)
            idx = int(m.group(1)) if m else vn
            test_targets.append((vn, idx, vn))

    success_color_node = None
    for label, target_idx, real_path in test_targets:
        print(f"\n  Testing: {label} -> {real_path} (OpenCV index {target_idx})...")
        cap = None
        for api in [cv2.CAP_V4L2, cv2.CAP_ANY]:
            try:
                cap = cv2.VideoCapture(target_idx, api)
                if cap.isOpened():
                    break
            except Exception:
                pass
        if cap and cap.isOpened():
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_FPS, 15)
            ret, frame = cap.read()
            if ret and frame is not None:
                h, w = frame.shape[:2]
                diff = 0.0
                if frame.ndim == 3 and frame.shape[2] == 3:
                    diff = float(np.max(np.abs(frame[:, :, 0].astype(np.int16) - frame[:, :, 1].astype(np.int16))))
                is_color = diff > 5.0
                stream_type = "🎨 KÊNH MÀU RGB (DÙNG CHO BÁM LUỐNG)" if is_color else "⚪ Kênh hồng ngoại IR / Độ sâu Depth"
                print(f"    ✅ ĐỌC THÀNH CÔNG! Kích thước: {w}x{h} | {stream_type}")
                if is_color and success_color_node is None:
                    success_color_node = real_path
            else:
                print(f"    ⚠️ Mở được nhưng cap.read() trả về False.")
            cap.release()
        else:
            print(f"    ❌ Không thể mở cổng {target_idx} qua OpenCV.")

    print("\n" + "=" * 70)
    print("  📋 KẾT LUẬN CHẨN ĐOÁN:")
    print("=" * 70)
    if success_color_node:
        print(f"  🎉 CAMERA ĐÃ HOẠT ĐỘNG HOÀN HẢO!")
        print(f"  • Cổng màu RGB chính : {success_color_node}")
        print(f"  • Lệnh chạy kiểm tra : test-d435")
    elif d435_found:
        print("  ⚠️ Camera được nhận diện trên USB nhưng chưa mở được luồng Video bằng OpenCV.")
        print("  👉 Khắc phục nhanh: Chạy lệnh cài đặt quyền udev:")
        print("     bash scripts/setup_realsense.sh")
    else:
        print("  ❌ Camera chưa được cắm hoặc cáp USB bị lỏng.")
        print("  👉 Khắc phục: Cắm chặt lại cáp Type-C vào cổng USB 3.0 màu xanh trên Laptop/PC.")
    print("=" * 70 + "\n")

if __name__ == '__main__':
    main()
