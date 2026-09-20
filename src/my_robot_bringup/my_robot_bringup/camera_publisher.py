#!/usr/bin/env python3
"""
Camera Publisher Node — Thu hình 1080p@60fps từ USB Webcam bằng OpenCV MJPG.

Thay thế hoàn toàn v4l2_camera_node (không hỗ trợ giải mã MJPG trên Jazzy).
OpenCV giải mã MJPG chuẩn xác trên mọi nền tảng (Pi 4/5, x86 PC).

Chức năng:
  - Mở camera USB (DV20) bằng cv2.VideoCapture + CAP_V4L2 + MJPG fourcc
  - Publish ảnh gốc 1920×1080 lên /camera/color/image_raw (encoding rgb8)
  - Background thread capture riêng để không block ROS 2 callbacks

Sử dụng:
  ros2 run my_robot_bringup camera_publisher
  ros2 run my_robot_bringup camera_publisher --ros-args -p width:=1920 -p height:=1080 -p fps:=60.0
"""

import os
import time
import threading
import socket
from urllib.parse import urlparse

# Set OpenCV FFMPEG network timeout (1.5s) to avoid hanging on dead/unreachable network streams
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "timeout;1500000|stimeout;1500000"

import numpy as np

try:
    import cv2
except ImportError:
    print("❌ Thiếu thư viện opencv-python! Chạy: sudo apt install python3-opencv")
    cv2 = None

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image


def _is_url_accessible(url: str, timeout: float = 0.8) -> bool:
    """Kiểm tra nhanh xem IP/port của stream có phản hồi không (tránh cv2.VideoCapture bị block 30s)."""
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


class RealSenseCapture:
    """
    Bộ bọc (Wrapper) cho Intel RealSense D435 qua pyrealsense2 pipeline.
    Cung cấp giao diện tương thích 100% với cv2.VideoCapture (isOpened, read, release, get).
    """
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
        if cv2 is not None:
            if prop == cv2.CAP_PROP_FRAME_WIDTH:
                return self.width
            if prop == cv2.CAP_PROP_FRAME_HEIGHT:
                return self.height
            if prop == cv2.CAP_PROP_FPS:
                return self.fps
        return 0


class CameraPublisher(Node):
    """Publish ảnh từ Intel RealSense D435, iPhone DroidCam hoặc USB Webcam."""

    def __init__(self):
        super().__init__('camera_publisher')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('video_device', 'realsense')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 15.0)
        self.declare_parameter('camera_frame_id', 'camera_link')

        self.device = self.get_parameter('video_device').value
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = float(self.get_parameter('fps').value)
        self.frame_id = self.get_parameter('camera_frame_id').value

        # ── Publisher (RELIABLE để tương thích CNN driver + wifi_cam_bridge) ─
        cam_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.image_pub = self.create_publisher(
            Image, '/camera/color/image_raw', cam_qos)
        # Fallback topic cho compatibility
        self.image_alt_pub = self.create_publisher(
            Image, '/camera/image_raw', cam_qos)

        # ── State ─────────────────────────────────────────────────────
        self.is_running = True
        self.cap = None
        self._frame_count = 0
        self._last_log_time = time.time()

        # ── Start capture & auto-reconnect thread ─────────────────────
        self._capture_thread = threading.Thread(
            target=self._capture_loop, daemon=True)
        self._capture_thread.start()

    def _try_open_urls(self, urls):
        """Thử mở luồng mạng (iPhone/IP camera). Trả về VideoCapture nếu thành công."""
        for url in urls:
            if not _is_url_accessible(url, timeout=0.8):
                continue
            for api in [cv2.CAP_FFMPEG, cv2.CAP_ANY]:
                cap = None
                try:
                    cap = cv2.VideoCapture(url, api)
                    if cap.isOpened():
                        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                        ret, test_frame = cap.read()
                        if ret and test_frame is not None:
                            actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                            actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                            self.get_logger().info(
                                f"📱 Camera iPhone / IP Stream đã kết nối thành công: {url} "
                                f"({actual_w}x{actual_h})")
                            return cap
                        cap.release()
                except Exception:
                    if cap is not None:
                        try:
                            cap.release()
                        except Exception:
                            pass
        return None

    def _is_realsense_device(self, dev: str) -> bool:
        """Kiểm tra xem thiết bị video có phải là RealSense không (tránh mở nhầm IR/Depth làm Webcam)."""
        try:
            real_path = os.path.realpath(dev) if os.path.exists(dev) else dev
            bname = os.path.basename(real_path)
            npath = f"/sys/class/video4linux/{bname}/name"
            if os.path.exists(npath):
                with open(npath, 'r') as f:
                    cname = f.read().strip()
                if any(kw in cname for kw in ['RealSense', 'realsense', 'D435', 'd435']):
                    return True
        except Exception:
            pass
        return False

    def _try_open_realsense(self):
        """
        ƯU TIÊN 1: Dò tìm và kết nối Camera Intel RealSense D435.
        Cách 1: Thử qua pyrealsense2 pipeline (nếu thư viện có sẵn trên máy).
        Cách 2: Quét V4L2 device node của RealSense (/dev/v4l/by-id/*RealSense* hoặc sysfs) mở bằng OpenCV.
        """
        # ── Cách 1: pyrealsense2 SDK ──
        try:
            import pyrealsense2 as rs
            ctx = rs.context()
            devices = ctx.query_devices()
            if len(devices) > 0:
                dev = devices[0]
                dev_name = dev.get_info(rs.camera_info.name)
                self.get_logger().info(f"🔍 [RealSense] Phát hiện thiết bị phần cứng: {dev_name}")
                pipeline = rs.pipeline()
                config = rs.config()
                rs_w = 640 if self.width <= 640 else 1280
                rs_h = 480 if self.height <= 480 else 720
                rs_fps = int(min(30.0, self.fps))
                config.enable_stream(rs.stream.color, rs_w, rs_h, rs.format.bgr8, rs_fps)
                pipeline.start(config)

                # Kiểm tra frame thực tế
                frames = pipeline.wait_for_frames(timeout_ms=2000)
                color_frame = frames.get_color_frame()
                if color_frame:
                    self.get_logger().info(
                        f"📷 [ƯU TIÊN 1] Intel RealSense D435 đã kết nối thành công qua pyrealsense2 SDK "
                        f"({rs_w}x{rs_h} @ {rs_fps} FPS, BGR8)!")
                    return RealSenseCapture(pipeline, rs_w, rs_h, rs_fps)
                pipeline.stop()
        except Exception:
            pass

        # ── Cách 2: V4L2 UVC Driver qua OpenCV ──
        rs_devs = []
        by_id = '/dev/v4l/by-id'
        if os.path.exists(by_id):
            for fn in sorted(os.listdir(by_id)):
                if any(kw in fn for kw in ['RealSense', 'realsense', 'D435', 'd435', 'Intel_R__RealSense']):
                    full = os.path.join(by_id, fn)
                    if 'index0' in fn:
                        rs_devs.insert(0, full)
                    else:
                        rs_devs.append(full)

        sys_v4l = '/sys/class/video4linux'
        if os.path.exists(sys_v4l):
            for vn in sorted(os.listdir(sys_v4l)):
                nfile = os.path.join(sys_v4l, vn, 'name')
                if os.path.exists(nfile):
                    try:
                        with open(nfile, 'r') as f:
                            cname = f.read().strip()
                        if any(kw in cname for kw in ['RealSense', 'realsense', 'D435', 'd435']):
                            dev_node = f'/dev/{vn}'
                            if os.path.exists(dev_node):
                                if 'RGB' in cname:
                                    rs_devs.insert(0, dev_node)
                                elif dev_node not in rs_devs:
                                    rs_devs.append(dev_node)
                    except Exception:
                        pass

        for dev in rs_devs:
            cap = None
            try:
                cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
                if not cap.isOpened():
                    cap = cv2.VideoCapture(dev)
                if cap.isOpened():
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                    cap.set(cv2.CAP_PROP_FPS, 30)
                    ret, test_frame = cap.read()
                    if ret and test_frame is not None:
                        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                        actual_fps = cap.get(cv2.CAP_PROP_FPS)
                        self.get_logger().info(
                            f"📷 [ƯU TIÊN 1] Intel RealSense D435 đã kết nối thành công qua V4L2: {dev} "
                            f"({actual_w}x{actual_h} @ {actual_fps:.0f} FPS)!")
                        return cap
                    cap.release()
            except Exception:
                if cap is not None:
                    try:
                        cap.release()
                    except Exception:
                        pass
        return None

    def _try_open_usb_devices(self):
        """Thử mở USB Webcam (/dev/video*). Bỏ qua RealSense để tránh xung đột IR/Depth."""
        candidates = []
        if isinstance(self.device, str) and self.device.startswith('/dev/video') and os.path.exists(self.device):
            if not self._is_realsense_device(self.device):
                candidates.append(self.device)
        for i in range(8):
            dev = f'/dev/video{i}'
            if dev not in candidates and os.path.exists(dev) and not self._is_realsense_device(dev):
                candidates.append(dev)

        resolutions_to_try = [(self.width, self.height, self.fps)]
        if (self.width, self.height, self.fps) != (640, 480, 30.0):
            resolutions_to_try.append((640, 480, 30.0))

        for dev in candidates:
            if dev.endswith('1') and '/dev/video0' in candidates and dev != self.device:
                continue

            if dev.startswith('/dev/video') and dev.replace('/dev/video', '').isdigit():
                dev_id = int(dev.replace('/dev/video', ''))
            else:
                dev_id = dev

            for w, h, fps in resolutions_to_try:
                cap = None
                try:
                    cap = cv2.VideoCapture(dev_id, cv2.CAP_V4L2)
                    if not cap.isOpened():
                        cap = cv2.VideoCapture(dev_id)

                    if cap.isOpened():
                        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
                        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
                        cap.set(cv2.CAP_PROP_FPS, fps)

                        ret, test_frame = cap.read()
                        if ret and test_frame is not None:
                            actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                            actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                            actual_fps = cap.get(cv2.CAP_PROP_FPS)
                            self.get_logger().info(
                                f"📷 [ƯU TIÊN 3] Camera USB Webcam đã kết nối thành công: {dev} "
                                f"({actual_w}x{actual_h} @ {actual_fps:.0f} FPS, MJPG)")
                            return cap
                        cap.release()
                except Exception:
                    if cap is not None:
                        try:
                            cap.release()
                        except Exception:
                            pass
        return None

    def _open_camera(self):
        """
        Mở camera: tự động tìm kiếm theo thứ tự ưu tiên nghiêm ngặt:
          1. Intel RealSense D435
          2. Camera iPhone (DroidCam qua USB / Wi-Fi Hotspot)
          3. USB Webcam (/dev/video*)
        """
        target = str(self.device).strip() if self.device else 'realsense'
        is_explicit_http = target.startswith(('http://', 'https://', 'rtsp://'))
        is_explicit_v4l = target.startswith('/dev/video') and os.path.exists(target) and not self._is_realsense_device(target)

        # ── 1. ƯU TIÊN 1: INTEL REALSENSE D435 ──
        if target in ('realsense', 'auto', 'd435', 'd435i', '') or (not is_explicit_http and not is_explicit_v4l):
            cap = self._try_open_realsense()
            if cap is not None:
                return cap

        if target in ('realsense', 'd435', 'd435i'):
            self.get_logger().info(
                "⚠️ Không tìm thấy Intel RealSense D435, chuyển sang [ƯU TIÊN 2] iPhone DroidCam...",
                throttle_duration_sec=10.0)

        # ── 2. ƯU TIÊN 2: CAMERA IPHONE DROIDCAM ──
        urls_to_try = []
        if is_explicit_http:
            urls_to_try.append(target)
            if target.endswith('/video'):
                urls_to_try.append(target.replace('/video', '/mjpegfeed'))
            elif target.endswith('/mjpegfeed'):
                urls_to_try.append(target.replace('/mjpegfeed', '/video'))

        default_iphone = 'http://172.20.10.1:4747/video'
        if default_iphone not in urls_to_try:
            urls_to_try.append(default_iphone)
            urls_to_try.append('http://172.20.10.1:4747/mjpegfeed')

        cap = self._try_open_urls(urls_to_try)
        if cap is not None:
            return cap

        # ── 3. ƯU TIÊN 3: USB WEBCAM VẬT LÝ ──
        self.get_logger().info(
            "⚠️ Không tìm thấy iPhone DroidCam, chuyển sang [ƯU TIÊN 3] USB Webcam (/dev/video*)...",
            throttle_duration_sec=10.0)
        cap = self._try_open_usb_devices()
        if cap is not None:
            return cap

        # Fallback cho explicit video device nếu chưa nhận diện được
        if is_explicit_v4l:
            try:
                cap = cv2.VideoCapture(target, cv2.CAP_V4L2)
                if cap.isOpened():
                    ret, test_frame = cap.read()
                    if ret and test_frame is not None:
                        return cap
                    cap.release()
            except Exception:
                pass

        return None

    def _capture_loop(self):
        """Background thread: đọc frame liên tục và tự động kết nối lại nếu camera bị ngắt."""
        null_fd = None
        try:
            null_fd = os.open(os.devnull, os.O_WRONLY)
        except Exception:
            pass

        consecutive_failures = 0
        try:
            while self.is_running:
                if self.cap is None or not self.cap.isOpened():
                    self.cap = self._open_camera()
                    if self.cap is None or not self.cap.isOpened():
                        self.get_logger().warn(
                            f"⏳ Đang dò tìm & kết nối Camera (Ưu tiên: 1. Intel D435 -> 2. iPhone -> 3. Webcam)... "
                            f"(Cắm cáp RealSense D435, bật DroidCam iPhone hoặc cắm USB Webcam)",
                            throttle_duration_sec=4.0)
                        time.sleep(1.5)
                        continue
                    consecutive_failures = 0
                # Tạm redirect stderr để suppress libjpeg warnings
                old_err = None
                if null_fd is not None:
                    try:
                        old_err = os.dup(2)
                        os.dup2(null_fd, 2)
                    except Exception:
                        old_err = None

                ret, frame = self.cap.read()

                # Khôi phục stderr
                if old_err is not None:
                    try:
                        os.dup2(old_err, 2)
                        os.close(old_err)
                    except Exception:
                        pass

                if not ret or frame is None:
                    consecutive_failures += 1
                    # Cho phép chịu lỗi liên tiếp trong ~3 giây (150 lần x 0.02s) trước khi hủy kết nối
                    if consecutive_failures >= 150:
                        self.get_logger().warn("Mất tín hiệu camera quá 3 giây, đang tự động kết nối lại...", throttle_duration_sec=5.0)
                        try:
                            self.cap.release()
                        except Exception:
                            pass
                        self.cap = None
                    time.sleep(0.01)
                    continue
                consecutive_failures = 0

                # Resize 1080p → 640x480 trước khi publish
                # (CNN driver resize lại 512x512, wifi_cam_bridge resize 320x240)
                # Tiết kiệm từ 6.2MB → 920KB mỗi frame
                pub_frame = cv2.resize(frame, (640, 480),
                                       interpolation=cv2.INTER_LINEAR)

                ph, pw, pc = pub_frame.shape

                # Tạo ROS 2 Image message (giữ bgr8 — không cần chuyển đổi)
                msg = Image()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.frame_id
                msg.height = ph
                msg.width = pw
                msg.encoding = 'bgr8'
                msg.is_bigendian = False
                msg.step = pw * pc
                msg.data = pub_frame.tobytes()

                if not self.is_running or not rclpy.ok():
                    break
                try:
                    self.image_pub.publish(msg)
                    self.image_alt_pub.publish(msg)
                except Exception:
                    break

                # Log thống kê mỗi 30 giây (giảm tải màn hình terminal)
                self._frame_count += 1
                now = time.time()
                if now - self._last_log_time >= 30.0:
                    elapsed = now - self._last_log_time if self._last_log_time > 0 else 30.0
                    fps_actual = self._frame_count / elapsed if elapsed > 0 else 0
                    self.get_logger().info(
                        f"📷 Camera: {pw}x{ph} | {fps_actual:.1f} FPS | "
                        f"Đang cấp dữ liệu trực tiếp cho AI CNN ({self._frame_count} frames)")
                    self._frame_count = 0
                    self._last_log_time = now

        finally:
            if null_fd is not None:
                try:
                    os.close(null_fd)
                except Exception:
                    pass

    def destroy_node(self):
        self.is_running = False
        if self.cap and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
