#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LET-VIDEO: Công cụ quay video trực tiếp từ Camera ưu tiên để thu thập Dataset.

Nhiệm vụ duy nhất: Ghi video chất lượng cao trực tiếp từ Camera ra file MP4.
Không chạy ROS 2 cồng kềnh, không tách frame, tối ưu tốc độ ghi và chống nghẽn đĩa.

Thứ tự ưu tiên Camera tự động:
  1. Intel RealSense D435 (pyrealsense2 SDK hoặc V4L2 RGB stream)
  2. Camera iPhone DroidCam (172.20.10.1:4747 qua Wi-Fi / Cáp USB)
  3. USB Webcam vật lý (/dev/video* chuẩn MJPG 30-60 FPS)

Cách dùng:
  let-video
  let-video [tên_video]
  let-video luong_bap_1
  let-video luong_1 --fps 30 --width 640 --height 480
  let-video luong_1 --no-view
  let-video luong_1 --device /dev/video0
"""

import os
import sys
import time
import glob
import signal
import socket
import argparse
import threading
import queue
from datetime import datetime
from urllib.parse import urlparse

# Tắt buffer đầu ra để in log thời gian thực
if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(line_buffering=True)

try:
    import cv2
    import numpy as np
except ImportError:
    print("❌ Lỗi: Thiếu thư viện OpenCV! Vui lòng chạy: sudo apt install python3-opencv")
    sys.exit(1)


def is_url_accessible(url: str, timeout: float = 0.5) -> bool:
    """Kiểm tra nhanh xem IP/port của stream có phản hồi không (tránh block 30s)."""
    try:
        parsed = urlparse(url)
        host = parsed.hostname
        port = parsed.port or (80 if parsed.scheme == 'http' else 443)
        if not host:
            return False
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except Exception:
        return False


class RealSenseSDKCapture:
    """Wrapper cho Intel RealSense D435 qua pyrealsense2 pipeline."""
    def __init__(self, pipeline, width: int = 640, height: int = 480, fps: float = 30.0):
        self.pipeline = pipeline
        self.width = width
        self.height = height
        self.fps = fps
        self._opened = True

    def isOpened(self) -> bool:
        return self._opened

    def read(self):
        if not self._opened:
            return False, None
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            color_frame = frames.get_color_frame()
            if not color_frame:
                return False, None
            frame = np.asanyarray(color_frame.get_data())
            return True, frame
        except Exception:
            return False, None

    def release(self):
        self._opened = False
        try:
            self.pipeline.stop()
        except Exception:
            pass

    def get(self, prop):
        if prop == cv2.CAP_PROP_FRAME_WIDTH:
            return self.width
        if prop == cv2.CAP_PROP_FRAME_HEIGHT:
            return self.height
        if prop == cv2.CAP_PROP_FPS:
            return self.fps
        return 0


def detect_and_open_camera(req_device=None, width=1920, height=1080, fps=30.0):
    """
    Tự động dò tìm và mở Camera theo đúng thứ tự ưu tiên chuẩn của hệ thống:
    1. Intel RealSense D435 (Ưu tiên 1080p @ 30 FPS)
    2. Camera iPhone DroidCam (1080p @ 30 FPS)
    3. USB Webcam vật lý (/dev/video* chuẩn 1080p MJPG @ 30 FPS)
    """
    # Nếu người dùng chỉ định thiết bị cụ thể
    if req_device and req_device != 'auto':
        print(f"👉 Đang kết nối thiết bị được chỉ định: {req_device}")
        if req_device in ('realsense', 'd435', 'D435'):
            cap, name = _open_realsense(width, height, fps)
            if cap:
                return cap, name
        elif req_device.startswith('http'):
            cap = _open_url(req_device)
            if cap:
                return cap, f"iPhone Stream ({req_device})"
        else:
            cap = _open_v4l2(req_device, width, height, fps)
            if cap:
                return cap, f"V4L2 Camera ({req_device})"
        print(f"⚠️ Không thể mở thiết bị {req_device}, chuyển sang tự động dò tìm...")

    # ── ƯU TIÊN 1: Intel RealSense D435 ──────────────────────────────────────
    print("🔍 [1/3] Đang tìm kiếm Intel RealSense D435 (Mục tiêu: 1080p @ 30 FPS)...")
    cap, name = _open_realsense(width, height, fps)
    if cap:
        return cap, name

    # ── ƯU TIÊN 2: Camera iPhone DroidCam (172.20.10.1:4747) ────────────────────────
    print("🔍 [2/3] Đang tìm kiếm Camera iPhone DroidCam (172.20.10.1: 1080p @ 30 FPS)...")
    iphone_urls = [
        "http://172.20.10.1:4747/mjpegfeed?1920x1080",
        "http://172.20.10.1:4747/video",
    ]
    for url in iphone_urls:
        if is_url_accessible(url, timeout=0.4):
            cap = _open_url(url)
            if cap:
                return cap, f"Camera iPhone DroidCam ({url})"

    # ── ƯU TIÊN 3: USB Webcam vật lý (/dev/video*) ───────────────────────────
    print("🔍 [3/3] Đang tìm kiếm USB Webcam vật lý (Mục tiêu: 1080p MJPG @ 30 FPS)...")
    candidates = sorted(glob.glob('/dev/video*'))
    for dev in candidates:
        sname_file = f"/sys/class/video4linux/{os.path.basename(dev)}/name"
        cname = ""
        if os.path.exists(sname_file):
            try:
                with open(sname_file) as f:
                    cname = f.read().strip()
            except Exception:
                pass
        if any(kw in cname.lower() for kw in ['ir', 'depth', 'metadata']):
            continue

        cap = _open_v4l2(dev, width, height, fps)
        if cap:
            dev_label = cname if cname else dev
            return cap, f"USB Webcam: {dev_label} ({dev})"

    return None, None


def _open_realsense(width, height, fps):
    """Mở RealSense qua pyrealsense2 hoặc V4L2 RGB node (Ưu tiên 1080p @ 30 FPS)."""
    try:
        import pyrealsense2 as rs
        ctx = rs.context()
        if len(ctx.query_devices()) > 0:
            pipeline = rs.pipeline()
            config = rs.config()
            rs_w = width if width > 0 else 1920
            rs_h = height if height > 0 else 1080
            rs_fps = int(min(30.0, fps))
            try:
                config.enable_stream(rs.stream.color, rs_w, rs_h, rs.format.bgr8, rs_fps)
                pipeline.start(config)
                frames = pipeline.wait_for_frames(timeout_ms=1500)
                if frames.get_color_frame():
                    return RealSenseSDKCapture(pipeline, rs_w, rs_h, rs_fps), f"Intel RealSense D435 ({rs_w}x{rs_h} @ {rs_fps} FPS)"
                pipeline.stop()
            except Exception:
                # Fallback nếu USB 2.0 nghẽn băng thông
                try:
                    config = rs.config()
                    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, rs_fps)
                    pipeline.start(config)
                    frames = pipeline.wait_for_frames(timeout_ms=1500)
                    if frames.get_color_frame():
                        return RealSenseSDKCapture(pipeline, 1280, 720, rs_fps), "Intel RealSense D435 (1280x720 @ 30 FPS)"
                    pipeline.stop()
                except Exception:
                    pass
    except Exception:
        pass

    by_id = '/dev/v4l/by-id'
    if os.path.exists(by_id):
        for fn in sorted(os.listdir(by_id)):
            if any(kw in fn.lower() for kw in ['realsense', 'd435']) and 'index0' in fn:
                full_path = os.path.join(by_id, fn)
                cap = _open_v4l2(full_path, width, height, fps)
                if cap:
                    return cap, f"Intel RealSense D435 (V4L2: {fn})"

    for dev in sorted(glob.glob('/dev/video*')):
        sname_file = f"/sys/class/video4linux/{os.path.basename(dev)}/name"
        if os.path.exists(sname_file):
            try:
                with open(sname_file) as f:
                    name = f.read().strip()
                if any(kw in name.lower() for kw in ['realsense', 'd435']) and 'rgb' in name.lower():
                    cap = _open_v4l2(dev, width, height, fps)
                    if cap:
                        return cap, f"Intel RealSense D435 RGB ({dev})"
            except Exception:
                pass
    return None, None


def _open_url(url):
    """Mở stream video mạng."""
    try:
        cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
        if not cap.isOpened():
            cap = cv2.VideoCapture(url)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            ret, frame = cap.read()
            if ret and frame is not None and frame.size > 0:
                return cap
            cap.release()
    except Exception:
        pass
    return None


def _open_v4l2(dev, width, height, fps):
    """Mở cổng V4L2 bằng OpenCV MJPG tối ưu."""
    try:
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap = cv2.VideoCapture(dev)
        if not cap.isOpened():
            return None

        try:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        except Exception:
            pass
        if width > 0 and height > 0:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if fps > 0:
            cap.set(cv2.CAP_PROP_FPS, fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        ret, frame = cap.read()
        if ret and frame is not None and frame.size > 0:
            return cap
        cap.release()
    except Exception:
        pass
    return None


class VideoRecordingEngine:
    """
    Bộ máy ghi hình đa luồng tốc độ cao:
    - Luồng đọc Capture độc lập, không trễ
    - Luồng ghi VideoWriter ra đĩa MP4
    - Hiển thị GUI preview trực quan hoặc CLI nếu chạy qua SSH headless
    """
    def __init__(self, cap, cam_name, output_path, fps=30.0, show_view=True):
        self.cap = cap
        self.cam_name = cam_name
        self.output_path = output_path
        self.target_fps = fps
        self.show_view = show_view

        # Đọc 1 frame thăm dò kích thước thực tế
        ret, test_frame = self.cap.read()
        if ret and test_frame is not None:
            self.height, self.width = test_frame.shape[:2]
            self.first_frame = test_frame
        else:
            self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or 1920
            self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or 1080
            self.first_frame = None

        cam_fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.record_fps = float(cam_fps) if cam_fps and cam_fps > 5.0 else self.target_fps

        self.frame_queue = queue.Queue(maxsize=150)
        self.is_running = True
        self.total_frames = 0
        self.start_time = None
        self.end_time = None

        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.writer = cv2.VideoWriter(self.output_path, fourcc, self.record_fps, (self.width, self.height))
        if not self.writer.isOpened():
            fourcc = cv2.VideoWriter_fourcc(*'avc1')
            self.writer = cv2.VideoWriter(self.output_path, fourcc, self.record_fps, (self.width, self.height))

        if not self.writer.isOpened():
            raise RuntimeError(f"❌ Không thể khởi tạo VideoWriter cho file: {self.output_path}")

        # Đẩy frame đầu tiên vào queue nếu có
        if self.first_frame is not None:
            self.frame_queue.put(self.first_frame)
            self.total_frames += 1

        self.writer_thread = threading.Thread(target=self._writer_worker, daemon=True)
        self.writer_thread.start()

    def _writer_worker(self):
        """Luồng chuyên ghi frame từ queue ra ổ cứng."""
        while self.is_running or not self.frame_queue.empty():
            try:
                frame = self.frame_queue.get(timeout=0.1)
                self.writer.write(frame)
                self.frame_queue.task_done()
            except queue.Empty:
                continue
            except Exception:
                break

    def run(self):
        """Vòng lặp chính đọc camera và hiển thị."""
        has_display = bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))
        gui_enabled = self.show_view and has_display

        window_name = "REC: LET-VIDEO Dataset Recorder"
        if gui_enabled:
            try:
                cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(window_name, min(1024, self.width), min(768, self.height))
            except Exception:
                gui_enabled = False

        self.start_time = time.time()
        last_stat_time = self.start_time
        fps_frame_count = 0
        current_fps = self.record_fps

        print("\n" + "=" * 80)
        print(f"🔴 [LET-VIDEO] ĐANG QUAY VIDEO DATASET TRỰC TIẾP...")
        print(f"   📹 Nguồn Camera : {self.cam_name}")
        print(f"   📐 Độ phân giải : {self.width} x {self.height} @ {self.record_fps:.1f} FPS")
        print(f"   💾 Tệp lưu trữ   : {self.output_path}")
        if gui_enabled:
            print(f"   🖥️ Màn hình xem  : Đang mở cửa sổ trực quan (Nhấn 'q' hoặc 'ESC' để dừng)")
        else:
            print(f"   🖥️ Chế độ dòng lệnh: Chạy nền Terminal (Nhấn 'Ctrl + C' để dừng và lưu)")
        print("=" * 80 + "\n")

        try:
            while self.is_running:
                ret, frame = self.cap.read()
                if not ret or frame is None:
                    time.sleep(0.01)
                    continue

                self.total_frames += 1
                fps_frame_count += 1
                now = time.time()

                if not self.frame_queue.full():
                    self.frame_queue.put(frame)

                elapsed_stat = now - last_stat_time
                if elapsed_stat >= 1.0:
                    current_fps = fps_frame_count / elapsed_stat
                    fps_frame_count = 0
                    last_stat_time = now

                elapsed_total = now - self.start_time
                mins = int(elapsed_total // 60)
                secs = int(elapsed_total % 60)

                file_mb = 0.0
                if os.path.exists(self.output_path):
                    try:
                        file_mb = os.path.getsize(self.output_path) / (1024 * 1024)
                    except Exception:
                        pass

                status_text = (
                    f"\r🔴 [REC {mins:02d}:{secs:02d}] "
                    f"Frames: {self.total_frames:<5d} | "
                    f"FPS: {current_fps:4.1f} | "
                    f"Dung lượng: {file_mb:5.1f} MB | "
                    f"Queue: {self.frame_queue.qsize():<2d} "
                )
                sys.stdout.write(status_text)
                sys.stdout.flush()

                if gui_enabled:
                    view_frame = frame.copy()
                    
                    cv2.rectangle(view_frame, (0, 0), (self.width, 42), (20, 20, 20), -1)
                    if int(now * 2) % 2 == 0:
                        cv2.circle(view_frame, (22, 21), 8, (0, 0, 255), -1)
                    else:
                        cv2.circle(view_frame, (22, 21), 8, (50, 50, 200), 2)
                    
                    rec_label = f"REC {mins:02d}:{secs:02d} | {self.total_frames} frames | {current_fps:.1f} FPS"
                    cv2.putText(view_frame, rec_label, (40, 27),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
                    
                    cv2.rectangle(view_frame, (0, self.height - 28), (self.width, self.height), (20, 20, 20), -1)
                    guide_txt = f"{self.cam_name[:35]} | Nhan phim [Q] hoac [ESC] de dung & luu"
                    cv2.putText(view_frame, guide_txt, (10, self.height - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 200), 1, cv2.LINE_AA)

                    cv2.imshow(window_name, view_frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord('q'), ord('Q'), 27):
                        print("\n\n⏹️ Người dùng nhấn phím dừng quay...")
                        break

        except KeyboardInterrupt:
            print("\n\n🛑 Nhận tín hiệu ngắt (Ctrl+C). Đang hoàn tất video...")
        finally:
            self.stop()
            if gui_enabled:
                try:
                    cv2.destroyAllWindows()
                except Exception:
                    pass

    def stop(self):
        """Dừng quay và đóng file an toàn."""
        self.is_running = False
        self.end_time = time.time()
        print("\n⏳ Đang lưu nốt các khung hình trong bộ đệm ra file...")
        self.writer_thread.join(timeout=3.0)

        try:
            self.cap.release()
        except Exception:
            pass

        try:
            self.writer.release()
        except Exception:
            pass

        total_time = max(0.1, (self.end_time or time.time()) - (self.start_time or time.time()))
        avg_fps = self.total_frames / total_time
        file_size_mb = 0.0
        if os.path.exists(self.output_path):
            file_size_mb = os.path.getsize(self.output_path) / (1024 * 1024)

        mins = int(total_time // 60)
        secs = int(total_time % 60)

        print("\n" + "=" * 80)
        print("✅ [LET-VIDEO] ĐÃ QUAY XONG VÀ LƯU VIDEO DATASET THÀNH CÔNG!")
        print(f"📁 Tệp video    : {self.output_path}")
        print(f"⏱️ Thời lượng   : {mins:02d}:{secs:02d} ({total_time:.1f} giây)")
        print(f"🎞️ Tổng số frame: {self.total_frames:,} khung hình")
        print(f"⚡ FPS trung bình: {avg_fps:.1f} FPS")
        print(f"💾 Dung lượng   : {file_size_mb:.2f} MB")
        print(f"📷 Nguồn camera : {self.cam_name}")
        print("=" * 80)
        print("👉 Gợi ý: Video sẵn sàng để gán nhãn hoặc cắt dataset (extract-dataset)!")


def main():
    parser = argparse.ArgumentParser(
        description="LET-VIDEO: Công cụ quay video trực tiếp từ Camera để lấy Dataset",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Ví dụ:\n  let-video\n  let-video luong_1\n  let-video luong_2 --no-view\n"
    )
    parser.add_argument('name', nargs='?', default='', help='Tên phiên quay (ví dụ: luong_1, bap_ngay)')
    parser.add_argument('--device', default='auto', help='Cổng camera (realsense, auto, /dev/video0, URL)')
    parser.add_argument('--width', type=int, default=1920, help='Chiều rộng khung hình (mặc định: 1920)')
    parser.add_argument('--height', type=int, default=1080, help='Chiều cao khung hình (mặc định: 1080)')
    parser.add_argument('--fps', type=float, default=30.0, help='Tốc độ khung hình (mặc định: 30.0)')
    parser.add_argument('--dir', default='', help='Thư mục lưu video (mặc định: ~/robot-ws/recordings)')
    parser.add_argument('--no-view', action='store_true', help='Tắt cửa sổ hiển thị (chạy headless nhẹ máy)')

    args = parser.parse_args()

    ws_dir = os.environ.get('WS_DIR', '')
    if not ws_dir:
        home = os.path.expanduser('~')
        candidates = [
            os.path.join(home, 'Màn hình nền', 'robot_ws'),
            os.path.join(home, 'robot-ws'),
            os.path.join(home, 'robot_ws'),
        ]
        for c in candidates:
            if os.path.isdir(c):
                ws_dir = c
                break
        if not ws_dir:
            ws_dir = os.getcwd()

    if args.dir:
        output_dir = os.path.abspath(args.dir)
    else:
        output_dir = os.path.join(ws_dir, 'recordings')

    os.makedirs(output_dir, exist_ok=True)

    if args.name:
        fname = args.name.strip()
        if not fname.endswith('.mp4'):
            fname += '.mp4'
    else:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        fname = f"dataset_{timestamp}.mp4"

    output_path = os.path.join(output_dir, fname)

    cap, cam_name = detect_and_open_camera(
        req_device=args.device,
        width=args.width,
        height=args.height,
        fps=args.fps
    )

    if cap is None:
        print("\n❌ [LỖI] Không tìm thấy hoặc không thể mở bất kỳ Camera nào!")
        print("   👉 Vui lòng kiểm tra cáp cắm Camera (RealSense / Webcam / iPhone DroidCam).")
        print("   👉 Kiểm tra nhanh danh sách camera: ls /dev/video*")
        sys.exit(1)

    engine = VideoRecordingEngine(
        cap=cap,
        cam_name=cam_name,
        output_path=output_path,
        fps=args.fps,
        show_view=not args.no_view
    )

    def handle_sig(sig, frame):
        engine.is_running = False

    signal.signal(signal.SIGINT, handle_sig)
    signal.signal(signal.SIGTERM, handle_sig)

    engine.run()


if __name__ == '__main__':
    main()
