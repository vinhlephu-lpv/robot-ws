#!/usr/bin/env python3
"""
Tự Động Huấn Luyện YOLOv8 Mũi Tên và Xuất Sang ONNX Tối Ưu Cho Laptop
-------------------------------------------------------------------
Cách dùng:
  python3 scripts/train_arrow_yolo.py [duong_dan_data.yaml]

Ví dụ:
  python3 scripts/train_arrow_yolo.py ~/Downloads/turn-arrow-1/data.yaml
"""

import os
import sys
import shutil
import subprocess

try:
    from ultralytics import YOLO
except ImportError:
    print("📦 Đang cài đặt thư viện ultralytics...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "--break-system-packages", "ultralytics"])
    from ultralytics import YOLO


def main():
    print("=" * 70)
    print("🚀 TỰ ĐỘNG HUẤN LUYỆN YOLOv8 MŨI TÊN (TURN ARROW DETECTION)")
    print("=" * 70)

    # 1. Tìm file data.yaml
    yaml_path = None
    if len(sys.argv) > 1 and os.path.exists(sys.argv[1]):
        yaml_path = os.path.abspath(sys.argv[1])
    else:
        # Tự động tìm trong dataset/
        candidates = [
            os.path.expanduser('~/robot_ws/dataset/arrow_marker/data.yaml'),
            os.path.expanduser('~/robot_ws/dataset/data.yaml'),
            os.path.expanduser('~/Downloads/data.yaml')
        ]
        for c in candidates:
            if os.path.exists(c):
                yaml_path = c
                break

    if not yaml_path or not os.path.exists(yaml_path):
        print("❌ Chưa tìm thấy file data.yaml do Roboflow xuất ra!")
        print("👉 Hãy truyền đường dẫn trực tiếp:")
        print("   python3 scripts/train_arrow_yolo.py /duong/dan/data.yaml")
        sys.exit(1)

    print(f"📄 Tìm thấy cấu hình Dataset: {yaml_path}")

    # 2. Huấn luyện model YOLOv8 Nano siêu nhẹ
    import torch
    device_to_use = 0 if torch.cuda.is_available() else 'cpu'
    dev_name = torch.cuda.get_device_name(0) if torch.cuda.is_available() else "CPU"
    print(f"🧠 Bắt đầu huấn luyện mô hình YOLOv8n (50 epochs) trên: {dev_name} ({device_to_use})...")
    model = YOLO("yolov8n.pt")
    results = model.train(
        data=yaml_path,
        epochs=50,
        imgsz=640,
        batch=16,
        device=device_to_use,
        project=os.path.expanduser('~/robot_ws/runs/train_arrow'),
        name='yolov8n_arrow',
        exist_ok=True
    )

    best_pt = os.path.expanduser('~/robot_ws/runs/train_arrow/yolov8n_arrow/weights/best.pt')
    if not os.path.exists(best_pt):
        print(f"❌ Không tìm thấy file weights tại: {best_pt}")
        sys.exit(1)

    print(f"✅ Huấn luyện hoàn tất! Trọng số tốt nhất: {best_pt}")

    # 3. Xuất sang định dạng ONNX
    print("⚡ Đang xuất sang định dạng ONNX siêu tốc để tích hợp vào ROS 2...")
    best_model = YOLO(best_pt)
    exported_path = best_model.export(format="onnx", imgsz=640, half=False)

    # 4. Copy vào thư mục models/ của dự án
    target_dir1 = os.path.expanduser('~/robot_ws/models')
    target_dir2 = os.path.expanduser('~/robot_ws/src/my_robot_controller/models')
    os.makedirs(target_dir1, exist_ok=True)
    os.makedirs(target_dir2, exist_ok=True)

    dest1 = os.path.join(target_dir1, 'turn_arrow_yolo.onnx')
    dest2 = os.path.join(target_dir2, 'turn_arrow_yolo.onnx')

    shutil.copyfile(exported_path, dest1)
    shutil.copyfile(exported_path, dest2)

    print("=" * 70)
    print("🎉 CHÚC MỪNG BẠN! MÔ HÌNH ĐÃ ĐƯỢC HUẤN LUYỆN VÀ SẴN SÀNG!")
    print(f"📁 Đã lưu mô hình ONNX tại:")
    print(f"   1. {dest1}")
    print(f"   2. {dest2}")
    print("👉 Bây giờ chỉ cần khởi động pc-cnn & pi-cnn: hệ thống sẽ tự động nạp mô hình này!")
    print("=" * 70)


if __name__ == '__main__':
    main()
