#!/usr/bin/env python3
"""
WiFi Camera Bridge - Nén ảnh JPEG trước khi gửi qua Wi-Fi.

Chạy trên Raspberry Pi. Subscribe ảnh raw (local) → resize 320×240 →
nén JPEG quality 50% → publish CompressedImage qua Wi-Fi (Best Effort).

Giảm bandwidth từ 27 MB/s xuống ~200 KB/s, giải phóng Wi-Fi cho LiDAR.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
import numpy as np

# Lazy import cv2 - sẽ báo lỗi rõ ràng nếu chưa cài
_cv2 = None


def _get_cv2():
    global _cv2
    if _cv2 is None:
        try:
            import cv2
            _cv2 = cv2
        except ImportError:
            raise RuntimeError(
                'python3-opencv chưa được cài đặt!\n'
                'Chạy: sudo apt install python3-opencv'
            )
    return _cv2


# Lazy import cv_bridge
_bridge = None


def _get_bridge():
    global _bridge
    if _bridge is None:
        try:
            from cv_bridge import CvBridge
            _bridge = CvBridge()
        except ImportError:
            _bridge = False
    return _bridge if _bridge is not False else None


class WifiCamBridge(Node):
    """Nén ảnh camera thành JPEG để gửi qua Wi-Fi."""

    def __init__(self):
        super().__init__('wifi_cam_bridge')

        # Parameters (có thể override từ launch file)
        self.declare_parameter('target_width', 320)
        self.declare_parameter('target_height', 240)
        self.declare_parameter('jpeg_quality', 50)
        self.declare_parameter('skip_frames', 2)  # Gửi mỗi frame thứ 3 (0,1,2 → gửi 0)

        self.target_w = self.get_parameter('target_width').value
        self.target_h = self.get_parameter('target_height').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        self.skip_frames = self.get_parameter('skip_frames').value

        # Subscribe raw image (local - cùng máy Pi, không qua Wi-Fi)
        raw_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.sub = self.create_subscription(
            Image, '/camera/color/image_raw', self._on_image, raw_qos)

        # Publish compressed image (qua Wi-Fi, Best Effort)
        wifi_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pub = self.create_publisher(
            CompressedImage, '/camera/compressed', wifi_qos)

        self._frame_count = 0
        self._sent_count = 0

        self.get_logger().info(
            f'WiFi Camera Bridge khởi động: '
            f'{self.target_w}x{self.target_h} JPEG q={self.jpeg_quality} '
            f'(gửi 1/{self.skip_frames + 1} frame)'
        )

    def _on_image(self, msg: Image):
        """Callback xử lý mỗi frame raw."""
        # Skip frames để giảm FPS
        self._frame_count += 1
        if (self._frame_count % (self.skip_frames + 1)) != 0:
            return

        cv2 = _get_cv2()
        bridge = _get_bridge()

        try:
            img = None
            # 1. Thử giải mã qua cv_bridge nếu có
            if bridge is not None:
                try:
                    img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                except Exception:
                    img = None

            # 2. Giải mã thủ công hỗ trợ toàn diện: RGB, BGR, MONO, YUYV (yuv422_yuy2), UYVY
            if img is None:
                enc = (msg.encoding or '').lower()
                if enc in ('rgb8',):
                    expected = msg.height * msg.width * 3
                    if len(msg.data) >= expected:
                        arr = np.frombuffer(msg.data, dtype=np.uint8)[:expected].reshape(msg.height, msg.width, 3)
                        img = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
                elif enc in ('bgr8',):
                    expected = msg.height * msg.width * 3
                    if len(msg.data) >= expected:
                        img = np.frombuffer(msg.data, dtype=np.uint8)[:expected].reshape(msg.height, msg.width, 3)
                elif enc in ('mono8',):
                    expected = msg.height * msg.width
                    if len(msg.data) >= expected:
                        arr = np.frombuffer(msg.data, dtype=np.uint8)[:expected].reshape(msg.height, msg.width)
                        img = cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
                elif enc in ('yuv422_yuy2', 'yuv422', 'yuyv'):
                    expected = msg.height * msg.width * 2
                    if len(msg.data) >= expected:
                        arr = np.frombuffer(msg.data, dtype=np.uint8)[:expected].reshape(msg.height, msg.width, 2)
                        img = cv2.cvtColor(arr, cv2.COLOR_YUV2BGR_YUYV)
                elif enc in ('uyvy',):
                    expected = msg.height * msg.width * 2
                    if len(msg.data) >= expected:
                        arr = np.frombuffer(msg.data, dtype=np.uint8)[:expected].reshape(msg.height, msg.width, 2)
                        img = cv2.cvtColor(arr, cv2.COLOR_YUV2BGR_UYVY)
                else:
                    # Fallback thông minh dựa trên độ dài dữ liệu
                    total_pixels = msg.height * msg.width
                    if total_pixels > 0:
                        bpp = len(msg.data) / total_pixels
                        if abs(bpp - 2.0) < 0.1:
                            expected = total_pixels * 2
                            arr = np.frombuffer(msg.data, dtype=np.uint8)[:expected].reshape(msg.height, msg.width, 2)
                            img = cv2.cvtColor(arr, cv2.COLOR_YUV2BGR_YUYV)
                        elif abs(bpp - 3.0) < 0.1:
                            expected = total_pixels * 3
                            arr = np.frombuffer(msg.data, dtype=np.uint8)[:expected].reshape(msg.height, msg.width, 3)
                            img = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)

            if img is None:
                self.get_logger().warn(
                    f'Không thể giải mã format ảnh: {msg.encoding} ({msg.width}x{msg.height}, len={len(msg.data)})',
                    throttle_duration_sec=5.0)
                return

            # Resize
            if (img.shape[1] != self.target_w) or (img.shape[0] != self.target_h):
                img = cv2.resize(img, (self.target_w, self.target_h),
                                 interpolation=cv2.INTER_AREA)

            # Nén JPEG
            encode_params = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
            success, jpeg_data = cv2.imencode('.jpg', img, encode_params)
            if not success:
                return

            # Publish CompressedImage
            comp_msg = CompressedImage()
            comp_msg.header = msg.header
            comp_msg.format = 'jpeg'
            comp_msg.data = jpeg_data.tobytes()
            self.pub.publish(comp_msg)

            self._sent_count += 1
            if self._sent_count == 1 or self._sent_count % 100 == 0:
                kb = len(comp_msg.data) / 1024
                self.get_logger().info(
                    f'✅ Đã gửi {self._sent_count} frames qua WiFi, '
                    f'size={kb:.1f} KB/frame ({self.target_w}x{self.target_h})',
                    throttle_duration_sec=15.0)

        except Exception as e:
            self.get_logger().error(
                f'Lỗi nén ảnh: {e}', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = WifiCamBridge()
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
