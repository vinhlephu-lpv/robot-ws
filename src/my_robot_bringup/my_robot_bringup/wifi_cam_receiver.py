#!/usr/bin/env python3
"""
WiFi Camera Receiver - Giải nén JPEG từ Pi thành raw Image cho RViz.

Chạy trên Laptop. Subscribe CompressedImage từ Wi-Fi (Best Effort) →
giải nén JPEG → publish raw Image lên topic local cho RViz hiển thị.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
import numpy as np

# Lazy import cv2
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


class WifiCamReceiver(Node):
    """Giải nén JPEG từ Pi, publish raw Image cho RViz."""

    def __init__(self):
        super().__init__('wifi_cam_receiver')

        # Subscribe compressed image (từ Pi qua Wi-Fi, Best Effort)
        wifi_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.sub = self.create_subscription(
            CompressedImage, '/camera/compressed', self._on_compressed, wifi_qos)

        # Publish raw image (local cho RViz, reliable vì cùng máy)
        local_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pub = self.create_publisher(
            Image, '/camera/wifi_image', local_qos)
        self.pub_raw = self.create_publisher(
            Image, '/camera/color/image_raw', local_qos)

        self._recv_count = 0
        self.get_logger().info(
            'WiFi Camera Receiver khởi động: '
            '/camera/compressed → /camera/wifi_image & /camera/color/image_raw'
        )

    def _on_compressed(self, msg: CompressedImage):
        """Callback giải nén mỗi frame JPEG."""
        cv2 = _get_cv2()

        try:
            # Giải nén JPEG
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if img_bgr is None:
                self.get_logger().warn(
                    'Không thể giải nén JPEG frame',
                    throttle_duration_sec=5.0)
                return

            # Chuyển BGR → RGB cho ROS
            img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)

            # Tạo ROS Image message
            img_msg = Image()
            img_msg.header = msg.header
            img_msg.height = img_rgb.shape[0]
            img_msg.width = img_rgb.shape[1]
            img_msg.encoding = 'rgb8'
            img_msg.is_bigendian = False
            img_msg.step = img_rgb.shape[1] * 3
            img_msg.data = img_rgb.tobytes()
            self.pub.publish(img_msg)
            self.pub_raw.publish(img_msg)

            self._recv_count += 1
            if self._recv_count % 100 == 0:
                self.get_logger().info(
                    f'Đã giải nén {self._recv_count} frames',
                    throttle_duration_sec=30.0)

        except Exception as e:
            self.get_logger().error(
                f'Lỗi giải nén: {e}', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = WifiCamReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
