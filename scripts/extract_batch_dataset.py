#!/usr/bin/env python3
"""
Batch Dataset Extractor
Trích xuất video sang ảnh với tần suất chính xác 3 ảnh/giây (3 FPS).
- Nhóm YOLO: video4.mp4, video7.mp4 -> dataset/YoLo_Dataset
- Nhóm CNN: video9.mp4, video10.mp4, video11.mp4, video12.mp4, video13.mp4 -> dataset/CNN_Dataset
"""

import os
import sys
import time
import cv2

def extract_video_frames(video_path, output_dir, prefix, rate_fps=3.0, quality=98):
    if not os.path.exists(video_path):
        print(f"❌ Không tìm thấy file: {video_path}")
        return 0

    os.makedirs(output_dir, exist_ok=True)
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"❌ Không thể mở video: {video_path}")
        return 0

    video_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    duration_s = total_frames / video_fps if video_fps > 0 else 0.0

    target_count = int(total_frames * rate_fps / video_fps)
    target_indices = set(int(round(i * video_fps / rate_fps)) for i in range(target_count))

    print(f"\n🎬 Xử lý: {os.path.basename(video_path)}")
    print(f"   📐 {width}x{height} | {video_fps:.1f} FPS | Thời lượng: {duration_s:.1f}s | Tổng: {total_frames} frames")
    print(f"   🎯 Tốc độ trích xuất: {rate_fps:.1f} ảnh/giây -> Dự kiến: ~{len(target_indices)} ảnh")

    saved_count = 0
    idx = 0
    t0 = time.time()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        if idx in target_indices:
            saved_count += 1
            out_filename = f"{prefix}_frame_{saved_count:04d}.jpg"
            out_path = os.path.join(output_dir, out_filename)
            cv2.imwrite(out_path, frame, [cv2.IMWRITE_JPEG_QUALITY, quality])

            pct = (idx + 1) / total_frames * 100
            print(f"\r   ⏳ Đang trích xuất: {pct:5.1f}% [{saved_count}/{len(target_indices)} ảnh]...", end="", flush=True)

        idx += 1

    cap.release()
    t_elapsed = time.time() - t0
    print(f"\r   ✅ Hoàn tất! Đã lưu {saved_count} ảnh chất lượng cao vào: {output_dir} ({t_elapsed:.1f}s)")
    return saved_count

def main():
    base_dir = "/home/acer/robot_ws"
    rec_dir = os.path.join(base_dir, "recordings")
    dataset_dir = os.path.join(base_dir, "dataset")

    yolo_dir = os.path.join(dataset_dir, "YoLo_Dataset")
    cnn_dir = os.path.join(dataset_dir, "CNN_Dataset")

    os.makedirs(yolo_dir, exist_ok=True)
    os.makedirs(cnn_dir, exist_ok=True)

    # 1. Trích xuất nhóm YOLO (video 4 & 7)
    print("=" * 70)
    print("🚀 BẮT ĐẦU TRÍCH XUẤT NHÓM: YoLo Dataset (Video 4 & 7)")
    print(f"📁 Thư mục đích: {yolo_dir}")
    print("=" * 70)

    yolo_videos = [
        ("video4.mp4", "video4"),
        ("video7.mp4", "video7"),
    ]
    total_yolo = 0
    for vname, pfx in yolo_videos:
        vpath = os.path.join(rec_dir, vname)
        total_yolo += extract_video_frames(vpath, yolo_dir, pfx, rate_fps=3.0)

    # 2. Trích xuất nhóm CNN (video 9, 10, 11, 12, 13)
    print("\n" + "=" * 70)
    print("🚀 BẮT ĐẦU TRÍCH XUẤT NHÓM: CNN Dataset (Video 9, 10, 11, 12, 13)")
    print(f"📁 Thư mục đích: {cnn_dir}")
    print("=" * 70)

    cnn_videos = [
        ("video9.mp4", "video9"),
        ("video10.mp4", "video10"),
        ("video11.mp4", "video11"),
        ("video12.mp4", "video12"),
        ("video13.mp4", "video13"),
    ]
    total_cnn = 0
    for vname, pfx in cnn_videos:
        vpath = os.path.join(rec_dir, vname)
        total_cnn += extract_video_frames(vpath, cnn_dir, pfx, rate_fps=3.0)

    # 3. Tạo symlinks tiện lợi
    # Thư mục dataset/
    for name, target in [("YoLo Dataset", yolo_dir), ("CNN dataset", cnn_dir)]:
        link_path = os.path.join(dataset_dir, name)
        if not os.path.exists(link_path):
            os.symlink(target, link_path)
    
    # Thư mục robot_ws/
    for name, target in [("YoLo Dataset", yolo_dir), ("CNN dataset", cnn_dir)]:
        link_path = os.path.join(base_dir, name)
        if not os.path.exists(link_path):
            os.symlink(target, link_path)

    print("\n" + "=" * 70)
    print("🎉 TỔNG KẾT HOÀN TẤT TRÍCH XUẤT 3 ẢNH/GIÂY")
    print(f"📦 [YoLo Dataset]: {total_yolo} ảnh tại -> {yolo_dir}")
    print(f"📦 [CNN Dataset ]: {total_cnn} ảnh tại -> {cnn_dir}")
    print("=" * 70)

if __name__ == "__main__":
    main()
