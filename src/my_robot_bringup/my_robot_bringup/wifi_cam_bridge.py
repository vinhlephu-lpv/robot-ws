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

        try:
            # Chuyển ROS Image → numpy array
            if msg.encoding in ('rgb8', 'RGB8'):
                channels = 3
                cvt_code = cv2.COLOR_RGB2BGR
            elif msg.encoding in ('bgr8', 'BGR8'):
                channels = 3
                cvt_code = None
            elif msg.encoding in ('mono8', 'MONO8'):
                channels = 1
                cvt_code = None
            else:
                # Fallback: giả sử RGB8
                channels = 3
                cvt_code = cv2.COLOR_RGB2BGR

            expected_size = msg.height * msg.width * channels
            if len(msg.data) < expected_size:
                self.get_logger().warn(
                    f'Image data quá nhỏ: {len(msg.data)} < {expected_size}',
                    throttle_duration_sec=5.0)
                return

            img = np.frombuffer(msg.data, dtype=np.uint8)
            img = img[:expected_size].reshape(msg.height, msg.width, channels)

            # Chuyển sang BGR cho cv2
            if cvt_code is not None:
                img = cv2.cvtColor(img, cvt_code)

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
            if self._sent_count % 100 == 0:
                kb = len(comp_msg.data) / 1024
                self.get_logger().info(
                    f'Đã gửi {self._sent_count} frames, '
                    f'size={kb:.1f} KB/frame',
                    throttle_duration_sec=30.0)

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
