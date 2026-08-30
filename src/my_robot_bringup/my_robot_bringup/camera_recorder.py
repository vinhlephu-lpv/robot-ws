#!/usr/bin/env python3
"""
High-Performance Camera Recorder & Frame Extractor Node for RViz.
Sử dụng MultiThreadedExecutor + Queue đệm RAM để ghi video và tách ảnh không độ trễ,
đảm bảo RViz xem mượt mà 30 FPS không bao giờ bị đơ hay lag.
"""

import os
import sys
import time
import csv
import glob
import queue
import threading
import subprocess
from datetime import datetime
import numpy as np

try:
    import cv2
except ImportError:
    print("❌ Thiếu thư viện opencv-python! Chạy: pip3 install opencv-python")

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class CameraRecorderNode(Node):
    def __init__(self):
        super().__init__('camera_recorder_node')

        self.callback_group = ReentrantCallbackGroup()

        # Thư mục gốc dataset chuẩn (luôn ưu tiên thư mục trong Workspace)
        ws_candidates = [
            os.path.join(os.path.expanduser('~'), 'Màn hình nền', 'robot_ws', 'dataset'),
            os.path.join(os.path.expanduser('~'), 'robot_ws', 'dataset'),
            os.path.join(os.path.expanduser('~'), 'robot-ws', 'dataset'),
            os.path.join(os.getcwd(), 'dataset'),
        ]
        default_dataset_dir = ws_candidates[0]
        for cand in ws_candidates:
            if os.path.exists(os.path.dirname(cand)):
                default_dataset_dir = cand
                break

        self.declare_parameter('mode', 'auto')          # 'auto', 'topic', hoặc 'v4l2'
        self.declare_parameter('device', 'auto')        # Cổng /dev/video* (nếu dùng V4L2)
        self.declare_parameter('width', 1920)           # 1920 (1080p Full HD), 1280 (720p), 640 (VGA)
        self.declare_parameter('height', 1080)
        self.declare_parameter('fps', 60.0)             # 60.0 FPS hoặc 30.0 FPS
        self.declare_parameter('topic', '/camera/color/image_raw')
        self.declare_parameter('record_name', '')
        self.declare_parameter('extract_interval', 0.333)  # Mỗi 0.333s lưu 1 ảnh (3 ảnh/giây)
        self.declare_parameter('output_dir', default_dataset_dir)

        self.mode_param = self.get_parameter('mode').value
        self.device_param = self.get_parameter('device').value
        self.target_width = int(self.get_parameter('width').value)
        self.target_height = int(self.get_parameter('height').value)
        self.target_fps = float(self.get_parameter('fps').value)
        self.topic_name = self.get_parameter('topic').value
        self.custom_name = self.get_parameter('record_name').value
        self.extract_interval = float(self.get_parameter('extract_interval').value)
        self.dataset_root = self.get_parameter('output_dir').value

        # Tên phiên thu thập dữ liệu
        if self.custom_name:
            self.session_name = self.custom_name.replace('.mp4', '').replace(' ', '_')
        else:
            self.session_name = f"dataset_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

        # 1. Thư mục lưu video riêng biệt: dataset/videos/<session_name>.mp4
        self.video_dir = os.path.join(self.dataset_root, 'videos')
        os.makedirs(self.video_dir, exist_ok=True)
        self.video_path = os.path.join(self.video_dir, f"{self.session_name}.mp4")

        # 2. Thư mục lưu ảnh tách frame riêng biệt: dataset/imgs/<session_name>/frame_00001.jpg
        self.imgs_dir = os.path.join(self.dataset_root, 'imgs', self.session_name)
        os.makedirs(self.imgs_dir, exist_ok=True)

        # File nhãn CSV lưu góc lái và tốc độ
        self.csv_path = os.path.join(self.imgs_dir, 'labels.csv')
        self.csv_file = open(self.csv_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['frame_file', 'timestamp', 'linear_x', 'angular_z'])

        # Subscribe /cmd_vel
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_callback, 10,
            callback_group=self.callback_group)

        # Hàng đợi RAM FIFO không khóa (tối đa 90 frame ~ 3 giây)
        self.frame_queue = queue.Queue(maxsize=90)

        self.writer = None
        self.total_video_frames = 0
        self.total_extracted_images = 0
        self.last_extract_time = 0.0
        self.start_time = None
        self.last_log_time = 0.0
        self.is_running = True
        self.cap = None

        # Xác định chế độ hoạt động
        if self.mode_param == 'auto':
            has_astra = False
            try:
                out = subprocess.check_output(['lsusb'], text=True, stderr=subprocess.DEVNULL)
                if '2bc5:' in out:
                    has_astra = True
            except Exception:
                pass

            self.active_mode = 'topic' if has_astra else 'v4l2'
        else:
            self.active_mode = self.mode_param

        # QoS tối ưu chống lag: Best Effort + Keep Last 2 frames
        cam_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=2
        )

        if self.active_mode == 'topic':
            self.image_sub = self.create_subscription(
                Image, self.topic_name, self._topic_image_callback, cam_qos,
                callback_group=self.callback_group)
            self.get_logger().info(f"📷 [Chế độ Topic] Đang nhận luồng ảnh Camera USB qua: {self.topic_name}")
        else:
            self.image_pub = self.create_publisher(Image, self.topic_name, 10)
            self.alt_image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
            self.cap = self._open_v4l2_camera()
            if self.cap is None or not self.cap.isOpened():
                self.get_logger().warn("⚠️ Không tìm thấy Camera USB ngoài qua V4L2. Chuyển sang chờ topic ROS...")
                self.active_mode = 'topic'
                self.image_sub = self.create_subscription(
                    Image, self.topic_name, self._topic_image_callback, cam_qos,
                    callback_group=self.callback_group)
            else:
                self.v4l2_thread = threading.Thread(target=self._v4l2_capture_loop, daemon=True)
                self.v4l2_thread.start()

        # Luồng ngầm ghi đĩa độc lập (KHÔNG BAO GIỜ làm đơ ROS 2 hay RViz)
        self.writer_thread = threading.Thread(target=self._writer_worker, daemon=True)
        self.writer_thread.start()

        self.get_logger().info("=" * 65)
        self.get_logger().info("🚀 [RViz Camera Recorder] KHỞI ĐỘNG THÀNH CÔNG (ASYNC ENCODING)!")
        self.get_logger().info(f"📹 Nguồn hình ảnh:          {self.active_mode.upper()} -> {self.topic_name}")
        self.get_logger().info(f"🎞️ Đường dẫn Video riêng:   {self.video_path}")
        self.get_logger().info(f"🖼️ Thư mục Ảnh riêng:       {self.imgs_dir}/")
        self.get_logger().info(f"⏱️ Chu kỳ tách ảnh:         Mỗi {self.extract_interval:.3f}s/ảnh (3 ảnh/giây)")
        self.get_logger().info("=" * 65)

    def _open_v4l2_camera(self):
        """Mở webcam USB ngoài (ưu tiên webcam USB như DVD20, bỏ qua webcam tích hợp laptop)."""
        candidates = []
        if self.device_param != 'auto' and os.path.exists(self.device_param):
            candidates = [self.device_param]
        else:
            all_devs = sorted(glob.glob('/dev/video*'))
            usb_external = []
            builtin = []
            others = []
            for dev in all_devs:
                try:
                    out = subprocess.check_output(['udevadm', 'info', '-q', 'property', '-n', dev], text=True, stderr=subprocess.DEVNULL)
                    props = dict(line.split('=', 1) for line in out.strip().split('\n') if '=' in line)
                    model = props.get('ID_MODEL', '').lower()
                    vendor = props.get('ID_VENDOR', '').lower()
                    is_usb = (props.get('ID_BUS', '') == 'usb')
                    if 'user_facing' in model or 'integrated' in model or 'internal' in model:
                        builtin.append(dev)
                    elif is_usb:
                        usb_external.append(dev)
                    else:
                        others.append(dev)
                except Exception:
                    others.append(dev)
            # Ưu tiên webcam ngoài cắm qua USB
            candidates = usb_external + others + builtin

        for dev in candidates:
            try:
                dev_idx = int(dev.replace('/dev/video', '')) if dev.startswith('/dev/video') and dev.replace('/dev/video', '').isdigit() else dev
                cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
                if not cap.isOpened():
                    cap = cv2.VideoCapture(dev_idx)

                if cap.isOpened():
                    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.target_width)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.target_height)
                    cap.set(cv2.CAP_PROP_FPS, self.target_fps)
                    
                    # Tắt thông báo rác libjpeg từ C
                    null_fd = None
                    old_err = None
                    try:
                        null_fd = os.open(os.devnull, os.O_WRONLY)
                        old_err = os.dup(2)
                        os.dup2(null_fd, 2)
                    except Exception:
                        pass

                    ret, test_frame = cap.read()

                    if old_err is not None:
                        try:
                            os.dup2(old_err, 2)
                            os.close(old_err)
                        except Exception:
                            pass
                    if null_fd is not None:
                        try:
                            os.close(null_fd)
                        except Exception:
                            pass

                    if ret and test_frame is not None:
                        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                        self.get_logger().info(f"📷 Đã kết nối Webcam USB DVD20 tại {dev}: {w}x{h} @ {self.target_fps} FPS")
                        return cap
                    cap.release()
            except Exception:
                pass
        return None

    def _cmd_callback(self, msg: Twist):
        self.current_linear_x = msg.linear.x
        self.current_angular_z = msg.angular.z

    def _topic_image_callback(self, msg: Image):
        """Callback siêu nhẹ (<0.01ms): Chỉ đẩy con trỏ data vào RAM queue rồi thoát ngay!"""
        now = time.time()
        # Nếu queue bị đầy do đĩa ghi chậm, bỏ frame cũ nhất để luôn đồng bộ thời gian thực
        if self.frame_queue.full():
            try:
                self.frame_queue.get_nowait()
            except queue.Empty:
                pass

        try:
            self.frame_queue.put_nowait((msg.data, msg.width, msg.height, msg.encoding, now))
        except queue.Full:
            pass

    def _v4l2_capture_loop(self):
        """Vòng lặp đọc V4L2 và phát lên RViz (đã lọc sạch log rác Corrupt JPEG data từ libjpeg)."""
        null_fd = None
        try:
            null_fd = os.open(os.devnull, os.O_WRONLY)
        except Exception:
            pass

        try:
            while self.is_running and self.cap and self.cap.isOpened():
                old_err = None
                if null_fd is not None:
                    try:
                        old_err = os.dup(2)
                        os.dup2(null_fd, 2)
                    except Exception:
                        old_err = None

                ret, frame = self.cap.read()

                if old_err is not None:
                    try:
                        os.dup2(old_err, 2)
                        os.close(old_err)
                    except Exception:
                        pass

                if not ret or frame is None:
                    time.sleep(0.005)
                    continue

                now = time.time()
                h, w, c = frame.shape

                # 1. Đưa frame gốc 1080p Full HD vào Queue ngầm để ghi Video MP4 và tách Dataset ảnh
                if self.frame_queue.full():
                    try:
                        self.frame_queue.get_nowait()
                    except queue.Empty:
                        pass
                try:
                    self.frame_queue.put_nowait((frame.tobytes(), w, h, 'bgr8', now))
                except queue.Full:
                    pass

                # 2. Phát luồng ảnh preview 640x360 lên ROS 2 cho RViz (giúp RViz hiển thị tức thì, không bị nghẽn 370MB/s)
                try:
                    preview = cv2.resize(frame, (640, 360), interpolation=cv2.INTER_LINEAR)
                    ph, pw, pc = preview.shape
                    msg = Image()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'camera_link'
                    msg.height = ph
                    msg.width = pw
                    msg.encoding = 'bgr8'
                    msg.step = pw * pc
                    msg.data = preview.tobytes()
                    self.image_pub.publish(msg)
                    if hasattr(self, 'alt_image_pub'):
                        self.alt_image_pub.publish(msg)
                except Exception:
                    pass
        finally:
            if null_fd is not None:
                try:
                    os.close(null_fd)
                except Exception:
                    pass

    def _writer_worker(self):
        """Luồng công nhân ngầm: Chuyên xử lý chuyển đổi màu, ghi MP4 và lưu ảnh vào ổ cứng."""
        while self.is_running or not self.frame_queue.empty():
            try:
                item = self.frame_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            raw_data, w, h, encoding, now = item

            if self.start_time is None:
                self.start_time = now
                self.last_log_time = now

            try:
                # Giải mã định dạng ảnh
                if encoding == 'bgr8':
                    frame = np.frombuffer(raw_data, dtype=np.uint8).reshape((h, w, 3))
                elif encoding == 'rgb8':
                    rgb = np.frombuffer(raw_data, dtype=np.uint8).reshape((h, w, 3))
                    frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                elif encoding == 'mono8':
                    gray = np.frombuffer(raw_data, dtype=np.uint8).reshape((h, w))
                    frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                else:
                    frame = np.frombuffer(raw_data, dtype=np.uint8).reshape((h, w, -1))
                    if frame.shape[2] == 3:
                        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

                # Điều chỉnh độ phân giải đầu ra theo đúng target (hỗ trợ 2K 2560x1440, 1080p, v.v.)
                if (w != self.target_width or h != self.target_height) and self.target_width > 0 and self.target_height > 0:
                    frame = cv2.resize(frame, (self.target_width, self.target_height), interpolation=cv2.INTER_CUBIC)
                    h, w = self.target_height, self.target_width

                # 1. Ghi frame vào file Video MP4 (Đồng bộ 1:1 chuẩn thời gian thực, chống tua nhanh)
                if self.writer is None:
                    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                    self.writer = cv2.VideoWriter(self.video_path, fourcc, self.target_fps, (w, h))

                elapsed = max(0.0, now - self.start_time)
                target_video_frames = int(round(elapsed * self.target_fps))
                repeats = max(1, target_video_frames - self.total_video_frames)
                repeats = min(repeats, 15)

                for _ in range(repeats):
                    self.writer.write(frame)
                    self.total_video_frames += 1

                # 2. Tách ảnh theo chu kỳ
                if (now - self.last_extract_time) >= self.extract_interval:
                    self.total_extracted_images += 1
                    img_filename = f"frame_{self.total_extracted_images:05d}.jpg"
                    img_save_path = os.path.join(self.imgs_dir, img_filename)
                    cv2.imwrite(img_save_path, frame, [cv2.IMWRITE_JPEG_QUALITY, 100])

                    # Ghi nhãn CSV
                    self.csv_writer.writerow([
                        img_filename,
                        f"{now:.3f}",
                        f"{self.current_linear_x:.3f}",
                        f"{self.current_angular_z:.3f}"
                    ])
                    self.csv_file.flush()
                    self.last_extract_time = now

                # In log mỗi 2.5 giây
                if now - self.last_log_time >= 2.5:
                    elapsed = now - self.start_time
                    fps = self.total_video_frames / elapsed if elapsed > 0 else 0
                    mb = os.path.getsize(self.video_path) / (1024 * 1024) if os.path.exists(self.video_path) else 0
                    mins = int(elapsed // 60)
                    secs = int(elapsed % 60)
                    q_len = self.frame_queue.qsize()
                    self.get_logger().info(
                        f"🔴 [Quay Camera USB]: {mins:02d}:{secs:02d} | Video: {self.total_video_frames} frames ({fps:.1f} FPS, {mb:.1f}MB) | "
                        f"Đã tách: {self.total_extracted_images} ảnh -> dataset/imgs/{self.session_name}/ | Queue: {q_len}"
                    )
                    self.last_log_time = now

            except Exception as e:
                pass

    def destroy_node(self):
        self.is_running = False
        if hasattr(self, 'writer_thread') and self.writer_thread.is_alive():
            self.writer_thread.join(timeout=2.0)

        if self.writer is not None:
            self.writer.release()
            self.writer = None

        if self.cap is not None:
            self.cap.release()
            self.cap = None

        if hasattr(self, 'csv_file') and self.csv_file and not self.csv_file.closed:
            self.csv_file.close()

        elapsed = time.time() - self.start_time if self.start_time else 0.0
        final_fps = self.total_video_frames / elapsed if elapsed > 0 else 0.0
        mb = os.path.getsize(self.video_path) / (1024 * 1024) if os.path.exists(self.video_path) else 0.0

        mins = int(elapsed // 60)
        secs = int(elapsed % 60)

        self.get_logger().info("=" * 65)
        self.get_logger().info("✅ ĐÃ HOÀN TẤT THU THẬP DỮ LIỆU TỪ CAMERA USB!")
        self.get_logger().info(f"⏱️ Thời lượng:       {mins:02d}:{secs:02d} ({self.total_video_frames} frames @ {final_fps:.1f} FPS)")
        self.get_logger().info(f"📹 File Video riêng: {self.video_path} ({mb:.2f} MB)")
        self.get_logger().info(f"🖼️ Thư mục Ảnh riêng:{self.imgs_dir}/ ({self.total_extracted_images} ảnh sẵn sàng train)")
        self.get_logger().info(f"📊 File nhãn CSV:   {self.csv_path}")
        self.get_logger().info("=" * 65)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraRecorderNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
