#!/usr/bin/env python3
"""
Batch Dataset Extractor 2 (Cập nhật đúng video 03.mp4, 04.mp4, 05.mp4)
1. Trích xuất 03.mp4 & 04.mp4 -> dataset/dataset_cnn2 (3 ảnh/giây, tên 001.jpg -> ...)
2. Trích xuất 05.mp4          -> dataset/dataset_y2   (3 ảnh/giây, tên 001.jpg -> ...)
3. Nén thành 2 file zip:
   - dataset_cnn2.zip
   - dataset_y2.zip
"""

import os
import sys
import time
import shutil
import zipfile
import cv2

def extract_frames_sequential(video_list, out_dir, rate_fps=3.0, quality=98, start_num=1):
    # Dọn dẹp thư mục cũ để tạo mới hoàn toàn
    if os.path.exists(out_dir):
        shutil.rmtree(out_dir)
    os.makedirs(out_dir, exist_ok=True)

    saved_count = start_num - 1
    total_saved_for_group = 0
    info_map = []

    for video_path in video_list:
        if not os.path.exists(video_path):
            print(f"❌ Không tìm thấy file: {video_path}")
            continue

        cap = cv2.VideoCapture(video_path)
        if not cap.isOpened():
            print(f"❌ Không thể mở video: {video_path}")
            continue

        video_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
        total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        duration_s = total_frames / video_fps if video_fps > 0 else 0.0

        target_count = int(total_frames * rate_fps / video_fps)
        target_indices = set(int(round(i * video_fps / rate_fps)) for i in range(target_count))

        print(f"\n🎬 Xử lý: {os.path.basename(video_path)}")
        print(f"   📐 {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))} | {video_fps:.1f} FPS | Thời lượng: {duration_s:.1f}s")
        print(f"   🎯 Tốc độ: {rate_fps:.1f} ảnh/s -> Dự kiến: ~{len(target_indices)} ảnh")

        video_saved = 0
        idx = 0
        t0 = time.time()
        start_for_this_video = saved_count + 1

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            if idx in target_indices:
                saved_count += 1
                video_saved += 1
                fname = f"{saved_count:03d}.jpg"
                fpath = os.path.join(out_dir, fname)
                cv2.imwrite(fpath, frame, [cv2.IMWRITE_JPEG_QUALITY, quality])

                pct = (idx + 1) / total_frames * 100
                print(f"\r   ⏳ Đang trích xuất: {pct:5.1f}% [{video_saved}/{len(target_indices)} ảnh] -> {fname}...", end="", flush=True)

            idx += 1

        cap.release()
        t_elapsed = time.time() - t0
        end_for_this_video = saved_count
        info_map.append((os.path.basename(video_path), start_for_this_video, end_for_this_video, video_saved))
        total_saved_for_group += video_saved
        print(f"\r   ✅ Đã lưu {video_saved} ảnh ({start_for_this_video:03d}.jpg -> {end_for_this_video:03d}.jpg) trong {t_elapsed:.1f}s")

    return total_saved_for_group, info_map


def compress_to_zip(src_dir, zip_path):
    print(f"\n📦 Đang nén {src_dir} -> {os.path.basename(zip_path)}...")
    if os.path.exists(zip_path):
        os.remove(zip_path)
    t0 = time.time()
    with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED) as zipf:
        for root, dirs, files in os.walk(src_dir):
            for file in sorted(files):
                full_path = os.path.join(root, file)
                rel_path = os.path.relpath(full_path, os.path.dirname(src_dir))
                zipf.write(full_path, rel_path)
    sz_mb = os.path.getsize(zip_path) / (1024 * 1024)
    print(f"   ✅ Nén xong {os.path.basename(zip_path)}: {sz_mb:.1f} MB ({time.time() - t0:.1f}s)")
    return sz_mb


def main():
    base_dir = "/home/acer/robot_ws"
    rec_dir = os.path.join(base_dir, "recordings")
    dataset_dir = os.path.join(base_dir, "dataset")

    cnn2_dir = os.path.join(dataset_dir, "dataset_cnn2")
    y2_dir = os.path.join(dataset_dir, "dataset_y2")

    cnn2_zip = os.path.join(dataset_dir, "dataset_cnn2.zip")
    y2_zip = os.path.join(dataset_dir, "dataset_y2.zip")

    # 1. CNN Dataset 2 (03.mp4 & 04.mp4)
    print("=" * 70)
    print("🚀 BẮT ĐẦU TRÍCH XUẤT NHÓM: dataset_cnn2 (Video 03.mp4 & 04.mp4)")
    print(f"📁 Thư mục đích: {cnn2_dir}")
    print("=" * 70)
    cnn_videos = [
        os.path.join(rec_dir, "03.mp4"),
        os.path.join(rec_dir, "04.mp4"),
    ]
    total_cnn2, cnn_info = extract_frames_sequential(cnn_videos, cnn2_dir, rate_fps=3.0)

    # 2. YOLO Dataset 2 (05.mp4)
    print("\n" + "=" * 70)
    print("🚀 BẮT ĐẦU TRÍCH XUẤT NHÓM: dataset_y2 (Video 05.mp4)")
    print(f"📁 Thư mục đích: {y2_dir}")
    print("=" * 70)
    y_videos = [
        os.path.join(rec_dir, "05.mp4"),
    ]
    total_y2, y_info = extract_frames_sequential(y_videos, y2_dir, rate_fps=3.0)

    # 3. Nén file zip
    print("\n" + "=" * 70)
    print("🗜️ TIẾN HÀNH NÉN ZIP")
    print("=" * 70)
    sz_cnn = compress_to_zip(cnn2_dir, cnn2_zip)
    sz_y2 = compress_to_zip(y2_dir, y2_zip)

    # Tạo symlink tiện lợi tại ~/robot_ws
    for zname in ["dataset_cnn2.zip", "dataset_y2.zip"]:
        src_link = os.path.join(dataset_dir, zname)
        dst_link = os.path.join(base_dir, zname)
        if os.path.exists(dst_link) or os.path.islink(dst_link):
            os.remove(dst_link)
        os.symlink(src_link, dst_link)

    print("\n" + "=" * 70)
    print("🎉 TỔNG KẾT HOÀN TẤT")
    print(f"📁 [dataset_cnn2]: {total_cnn2} ảnh tại {cnn2_dir}")
    for vname, s, e, count in cnn_info:
        print(f"   • {vname}: {count} ảnh ({s:03d}.jpg -> {e:03d}.jpg)")
    print(f"📦 [dataset_cnn2.zip]: {sz_cnn:.1f} MB tại {cnn2_zip}")

    print(f"\n📁 [dataset_y2]: {total_y2} ảnh tại {y2_dir}")
    for vname, s, e, count in y_info:
        print(f"   • {vname}: {count} ảnh ({s:03d}.jpg -> {e:03d}.jpg)")
    print(f"📦 [dataset_y2.zip]: {sz_y2:.1f} MB tại {y2_zip}")
    print("=" * 70)


if __name__ == "__main__":
    main()
