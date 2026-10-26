#!/usr/bin/env python3
"""
Split CNN Dataset into Train (80%) and Val (20%)
Sử dụng phương pháp lấy mẫu phân tầng đều đặn (mỗi 5 ảnh thì 1 ảnh vào val, 4 ảnh vào train).
Giúp đảm bảo toàn bộ các video (9->13), các góc cua, đoạn thẳng và ảnh trống đều được chia đều vào cả 2 tập.
"""

import os
import shutil
from pathlib import Path

def split_dataset():
    dataset_dir = Path("/home/acer/robot_ws/dataset/CNN_Dataset")
    train_dir = dataset_dir / "train"
    val_dir = dataset_dir / "val"

    train_dir.mkdir(parents=True, exist_ok=True)
    val_dir.mkdir(parents=True, exist_ok=True)

    # Lấy toàn bộ file ảnh .jpg ở thư mục gốc CNN_Dataset
    img_files = sorted([f for f in dataset_dir.glob("*.jpg") if f.is_file()])
    total = len(img_files)

    if total == 0:
        print("⚠️ Không tìm thấy file ảnh .jpg nào ở thư mục gốc CNN_Dataset!")
        return

    print("=" * 65)
    print("📂 TIẾN HÀNH PHÂN CHIA TẬP DỮ LIỆU CNN (TRAIN / VAL)")
    print(f"📁 Thư mục gốc: {dataset_dir}")
    print(f"📊 Tổng số ảnh : {total}")
    print("=" * 65)

    train_count = 0
    val_count = 0

    for idx, img_path in enumerate(img_files, 1):
        if idx % 5 == 0:
            # 20% vào tập Validation
            target_path = val_dir / img_path.name
            shutil.move(str(img_path), str(target_path))
            val_count += 1
        else:
            # 80% vào tập Train
            target_path = train_dir / img_path.name
            shutil.move(str(img_path), str(target_path))
            train_count += 1

    print(f"✅ Hoàn tất phân chia!")
    print(f"   🔹 Tập Train (Huấn luyện): {train_count} ảnh (~{train_count/total*100:.1f}%) -> {train_dir}")
    print(f"   🔹 Tập Val   (Kiểm tra)  : {val_count} ảnh (~{val_count/total*100:.1f}%) -> {val_dir}")
    print("=" * 65)

if __name__ == "__main__":
    split_dataset()
