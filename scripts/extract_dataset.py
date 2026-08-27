#!/usr/bin/env python3
"""
Dataset Extractor - Trích xuất từng ảnh từ file video để gán nhãn train CNN.

- Đọc file video MP4 quay từ xe thật.
- Trích xuất ảnh theo chu kỳ (mặc định mỗi 0.3s hoặc mỗi N frame) để tránh trùng lặp khi xe dừng/chạy chậm.
- Tự động đánh số thứ tự: frame_00001.jpg, frame_00002.jpg,...
- Lưu sẵn vào thư mục dataset/images/ chuẩn bị gán nhãn (YOLO / CNN).

Sử dụng:
    python3 extract_dataset.py <duong_dan_video.mp4>
    python3 extract_dataset.py <duong_dan_video.mp4> --interval 0.3
    python3 extract_dataset.py <duong_dan_video.mp4> --output ~/dataset_crop_row
"""

import os
import sys
import argparse
import cv2


def extract_frames(video_path, output_dir, time_interval=0.3, prefix="crop_row"):
    if not os.path.exists(video_path):
        print(f"❌ Lỗi: Không tìm thấy file video: {video_path}")
        return

    os.makedirs(output_dir, exist_ok=True)
    cap = cv2.VideoCapture(video_path)

    if not cap.isOpened():
        print(f"❌ Không thể mở file video: {video_path}")
        return

    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    duration_s = total_frames / fps if fps > 0 else 0

    frame_step = max(1, int(fps * time_interval))

    print("=" * 60)
    print("🎞️ BỘ TRÍCH XUẤT DATASET TỪ VIDEO")
    print(f"📁 Video nguồn: {os.path.basename(video_path)}")
    print(f"📐 Độ phân giải: {width}x{height} | {fps:.1f} FPS | Thời lượng: {duration_s:.1f}s")
    print(f"⏱️ Khoảng cách lấy mẫu: Mỗi {time_interval:.2f}s (cách {frame_step} frames)")
    print(f"📂 Thư mục xuất ảnh: {output_dir}")
    print("=" * 60)

    saved_count = 0
    current_frame = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        if current_frame % frame_step == 0:
            out_name = f"{prefix}_{saved_count+1:05d}.jpg"
            out_path = os.path.join(output_dir, out_name)
            cv2.imwrite(out_path, frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
            saved_count += 1

            progress = (current_frame / total_frames * 100) if total_frames > 0 else 0
            print(f"\r⏳ Đang trích xuất: {progress:5.1f}% | Đã lưu: {saved_count} ảnh...", end="", flush=True)

        current_frame += 1

    cap.release()
    print(f"\n\n🎉 HOÀN TẤT! Đã trích xuất thành công {saved_count} ảnh vào:\n👉 {output_dir}\n")


def main():
    parser = argparse.ArgumentParser(description="Trích xuất dataset ảnh từ video camera xe thật")
    parser.add_argument('video', help="Đường dẫn file video .mp4")
    parser.add_argument('--interval', type=float, default=0.3, help="Khoảng thời gian giữa các ảnh (giây), mặc định 0.3s")
    parser.add_argument('--output', default=None, help="Thư mục lưu ảnh trích xuất")
    parser.add_argument('--prefix', default="crop_row", help="Tiền tố tên ảnh (ví dụ: crop_row_00001.jpg)")
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    ws_dir = os.path.abspath(os.path.join(script_dir, '..'))
    out_dir = args.output or os.path.join(ws_dir, 'dataset', 'images')

    extract_frames(args.video, out_dir, args.interval, args.prefix)


if __name__ == '__main__':
    main()
