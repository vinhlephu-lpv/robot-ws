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


class CameraPublisher(Node):
    """Publish ảnh USB Webcam 1080p@60fps qua OpenCV MJPG backend."""

    def __init__(self):
        super().__init__('camera_publisher')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('video_device', '/dev/video0')
        self.declare_parameter('width', 1920)
        self.declare_parameter('height', 1080)
        self.declare_parameter('fps', 60.0)
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
        self._last_log_time = 0.0

        # ── Open camera & start capture thread ────────────────────────
        self.cap = self._open_camera()
        if self.cap is not None and self.cap.isOpened():
            self._capture_thread = threading.Thread(
                target=self._capture_loop, daemon=True)
            self._capture_thread.start()
        else:
            self.get_logger().error(
                f"❌ Không thể mở camera tại {self.device}! "
                "Kiểm tra kết nối USB và /dev/video*.")

    def _open_camera(self):
        """Mở camera USB bằng OpenCV V4L2 backend + MJPG fourcc."""
        # Thử mở theo device path trước
        candidates = []
        if os.path.exists(self.device):
            candidates.append(self.device)
        # Fallback: thử /dev/video0..5
        for i in range(6):
            dev = f'/dev/video{i}'
            if dev not in candidates and os.path.exists(dev):
                candidates.append(dev)

        for dev in candidates:
            try:
                # Dùng index số nếu có thể, hoặc path string
                if dev.startswith('/dev/video') and dev.replace('/dev/video', '').isdigit():
                    dev_id = int(dev.replace('/dev/video', ''))
                else:
                    dev_id = dev

                cap = cv2.VideoCapture(dev_id, cv2.CAP_V4L2)
                if not cap.isOpened():
                    cap = cv2.VideoCapture(dev_id)

                if cap.isOpened():
                    # Đặt MJPG fourcc TRƯỚC khi set resolution
                    cap.set(cv2.CAP_PROP_FOURCC,
                            cv2.VideoWriter_fourcc(*'MJPG'))
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                    cap.set(cv2.CAP_PROP_FPS, self.fps)

                    # Đọc thử 1 frame
                    ret, test_frame = cap.read()
                    if ret and test_frame is not None:
                        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                        actual_fps = cap.get(cv2.CAP_PROP_FPS)
                        self.get_logger().info(
                            f"📷 Camera USB đã kết nối: {dev} "
                            f"({actual_w}x{actual_h} @ {actual_fps:.0f} FPS, MJPG)")
                        return cap
                    cap.release()
            except Exception as e:
                self.get_logger().warn(f"Không mở được {dev}: {e}")
        return None

    def _capture_loop(self):
        """Background thread: đọc frame liên tục và publish lên ROS 2."""
        # Tắt warning libjpeg rác ra stderr
        null_fd = None
        try:
            null_fd = os.open(os.devnull, os.O_WRONLY)
        except Exception:
            pass

        try:
            while self.is_running and self.cap and self.cap.isOpened():
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
                    time.sleep(0.002)
                    continue

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

                self.image_pub.publish(msg)
                self.image_alt_pub.publish(msg)

                # Log thống kê mỗi 10 giây
                self._frame_count += 1
                now = time.time()
                if now - self._last_log_time >= 10.0:
                    elapsed = now - self._last_log_time if self._last_log_time > 0 else 10.0
                    fps_actual = self._frame_count / elapsed if elapsed > 0 else 0
                    self.get_logger().info(
                        f"📷 Camera: {w}x{h} | {fps_actual:.1f} FPS | "
                        f"Đã gửi {self._frame_count} frames")
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
