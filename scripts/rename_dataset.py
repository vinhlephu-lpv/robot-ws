#!/usr/bin/env python3
"""
Dataset Renamer - Đổi tên ảnh dataset theo thứ tự liên tục (ví dụ: 0001.jpg, 0002.jpg, ...).

Đặc điểm:
- Giữ nguyên toàn bộ dữ liệu ảnh đã lọc, không làm mất hay thay đổi nội dung ảnh.
- Đổi tên an toàn qua 2 bước (two-pass) để tránh xung đột tên tệp.
- Tự động cập nhật file labels.csv (nếu có) tương ứng với tên ảnh mới và sao lưu labels.csv.bak.
- Hỗ trợ tham số dòng lệnh tùy chỉnh thư mục, số chữ số (mặc định 4: 0001.jpg), tiền tố, xem trước (--dry-run).

Cách dùng:
    python3 rename_dataset.py
    python3 rename_dataset.py "/home/vinh/Màn hình nền/robot_ws/dataset/imgs/dataset_2"
    python3 rename_dataset.py --dry-run
    python3 rename_dataset.py --digits 4 --start 1
"""

import os
import sys
import re
import csv
import shutil
import argparse
from pathlib import Path


def natural_sort_key(s):
    """Tách số và chữ để sắp xếp tự nhiên (ví dụ frame_9 trước frame_10)."""
    return [int(text) if text.isdigit() else text.lower() for text in re.split(r'(\d+)', str(s))]


def rename_dataset_images(target_dir, digits=4, start_index=1, prefix="", dry_run=False, update_csv=True):
    target_path = Path(target_dir).expanduser().resolve()
    
    if not target_path.exists() or not target_path.is_dir():
        print(f"❌ Lỗi: Thư mục không tồn tại: {target_path}")
        return False

    # Lấy danh sách ảnh hợp lệ
    valid_extensions = ('.jpg', '.jpeg', '.png', '.bmp')
    all_files = [f for f in target_path.iterdir() if f.is_file()]
    image_files = [f for f in all_files if f.suffix.lower() in valid_extensions and not f.name.startswith('__temp_')]
    
    if not image_files:
        print(f"⚠️ Không tìm thấy file ảnh nào trong: {target_path}")
        return False

    # Sắp xếp ảnh theo thứ tự tự nhiên
    image_files.sort(key=lambda x: natural_sort_key(x.name))
    total_imgs = len(image_files)

    print("=" * 65)
    print("🔄 BỘ ĐỔI TÊN ẢNH DATASET")
    print(f"📂 Thư mục: {target_path}")
    print(f"📊 Tổng số ảnh hiện tại: {total_imgs}")
    print(f"🔢 Định dạng tên: {prefix}{'0' * (digits - 1)}1{image_files[0].suffix.lower()} -> {prefix}{total_imgs + start_index - 1:0{digits}d}{image_files[-1].suffix.lower()}")
    if dry_run:
        print("🔍 Chế độ: DRY RUN (Chỉ xem trước, không thay đổi file)")
    print("=" * 65)

    # Tạo mapping từ tên cũ sang tên mới
    rename_mapping = {}
    for idx, old_file in enumerate(image_files, start=start_index):
        ext = old_file.suffix.lower()
        new_name = f"{prefix}{idx:0{digits}d}{ext}"
        new_file = target_path / new_name
        rename_mapping[old_file.name] = (old_file, new_file, new_name)

    # Hiển thị vài file mẫu
    print("📋 Xem trước 5 file đầu tiên:")
    for old_name, (_, _, new_name) in list(rename_mapping.items())[:5]:
        print(f"   {old_name}  -->  {new_name}")
    if total_imgs > 5:
        print("   ...")
        print("📋 Xem trước 3 file cuối cùng:")
        for old_name, (_, _, new_name) in list(rename_mapping.items())[-3:]:
            print(f"   {old_name}  -->  {new_name}")
    print("-" * 65)

    if dry_run:
        print("✅ Kiểm tra hoàn tất (DRY RUN). Không có file nào bị thay đổi.")
        return True

    # BƯỚC 1: Đổi sang tên tạm thời để tránh xung đột
    temp_mapping = {}
    print("⏳ Bước 1/2: Đổi tên sang tệp tạm thời...")
    for idx, (old_name, (old_file, new_file, new_name)) in enumerate(rename_mapping.items()):
        temp_name = f"__temp_rename_{idx:06d}{old_file.suffix.lower()}"
        temp_file = target_path / temp_name
        old_file.rename(temp_file)
        temp_mapping[temp_name] = (temp_file, new_file, old_name, new_name)

    # BƯỚC 2: Đổi từ tên tạm thời sang tên đích cuối cùng
    print("⏳ Bước 2/2: Đổi tên sang định dạng đích...")
    for temp_name, (temp_file, new_file, old_name, new_name) in temp_mapping.items():
        temp_file.rename(new_file)

    print(f"✅ Đã đổi tên thành công {total_imgs} file ảnh ({start_index:0{digits}d} đến {total_imgs + start_index - 1:0{digits}d})!")

    # XỬ LÝ LABELS.CSV (nếu có)
    csv_path = target_path / "labels.csv"
    if update_csv and csv_path.exists():
        update_dataset_csv(csv_path, rename_mapping)

    return True


def update_dataset_csv(csv_path, rename_mapping):
    """Cập nhật lại file labels.csv theo tên ảnh mới, loại bỏ các dòng ảnh đã bị xóa."""
    bak_path = csv_path.with_suffix('.csv.bak')
    print("-" * 65)
    print(f"📄 Tìm thấy file nhãn: {csv_path.name}")
    
    try:
        with open(csv_path, mode='r', newline='', encoding='utf-8') as f:
            reader = csv.reader(f)
            header = next(reader, None)
            if not header:
                print("⚠️ File CSV rỗng, bỏ qua cập nhật CSV.")
                return
            
            rows = list(reader)

        # Sao lưu file CSV gốc
        shutil.copy2(csv_path, bak_path)
        print(f"💾 Đã sao lưu bản gốc vào: {bak_path.name}")

        # Lọc và cập nhật tên frame
        updated_rows = []
        old_name_to_new = {k: v[2] for k, v in rename_mapping.items()}
        
        # Tìm cột frame_file (thường là cột đầu tiên)
        frame_col_idx = 0
        for i, col in enumerate(header):
            if 'frame' in col.lower() or 'img' in col.lower() or 'file' in col.lower():
                frame_col_idx = i
                break

        kept_count = 0
        for row in rows:
            if not row:
                continue
            old_frame_name = row[frame_col_idx].strip()
            if old_frame_name in old_name_to_new:
                new_row = list(row)
                new_row[frame_col_idx] = old_name_to_new[old_frame_name]
                updated_rows.append(new_row)
                kept_count += 1

        # Ghi lại file CSV mới
        with open(csv_path, mode='w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(updated_rows)

        print(f"✅ Đã cập nhật {csv_path.name}: {kept_count}/{len(rows)} hàng (khớp {kept_count} ảnh đã lọc).")
    except Exception as e:
        print(f"⚠️ Lỗi khi cập nhật file CSV: {e}")


def main():
    default_dir = "/home/vinh/Màn hình nền/robot_ws/dataset/imgs/dataset_2"
    if not os.path.exists(default_dir):
        # Fallback tới thư mục tương đối nếu chạy từ thư mục khác
        script_dir = Path(__file__).resolve().parent
        ws_dir = script_dir.parent
        alt_dir = ws_dir / "dataset" / "imgs" / "dataset_2"
        if alt_dir.exists():
            default_dir = str(alt_dir)

    parser = argparse.ArgumentParser(description="Đổi tên ảnh dataset theo thứ tự liên tục (0001.jpg, 0002.jpg, ...)")
    parser.add_argument("path", nargs="?", default=default_dir, help="Đường dẫn thư mục chứa ảnh cần đổi tên")
    parser.add_argument("-d", "--digits", type=int, default=4, help="Số chữ số đánh số (mặc định 4: 0001.jpg)")
    parser.add_argument("-s", "--start", type=int, default=1, help="Số bắt đầu (mặc định 1)")
    parser.add_argument("-p", "--prefix", type=str, default="", help="Tiền tố tên ảnh (mặc định rỗng: 0001.jpg; nếu cần frame_ thì truyền --prefix frame_)")
    parser.add_argument("--dry-run", action="store_true", help="Chạy thử để xem trước, không thay đổi file thực tế")
    parser.add_argument("--no-csv", action="store_true", help="Không cập nhật file labels.csv")

    args = parser.parse_args()

    rename_dataset_images(
        target_dir=args.path,
        digits=args.digits,
        start_index=args.start,
        prefix=args.prefix,
        dry_run=args.dry_run,
        update_csv=not args.no_csv
    )


if __name__ == "__main__":
    main()
