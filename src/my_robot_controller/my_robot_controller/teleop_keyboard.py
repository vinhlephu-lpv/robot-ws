#!/usr/bin/env python3
"""
BÀN PHÍM ĐIỀU KHIỂN ROBOT CHUẨN GỐC ROS 2 (CHẠY LIÊN TỤC KHÔNG NGẮT)
- Sử dụng 100% chuẩn phím ROS 2 toàn cầu: i, ,, j, l, k, u, o, m, .
- Tự động phát liên tục 12 Hz nuôi ESP32 Watchdog: Bấm 1 lần là xe chạy liên tục không bao giờ bị khựng!
- Bấm [k] để DỪNG XE.
"""

import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

BANNER = """
================================================================================
          🤖 BÀN PHÍM ĐIỀU KHIỂN ROBOT CHUẨN GỐC ROS 2 (CHẠY LIÊN TỤC)
================================================================================
  [BẢNG PHÍM ĐIỀU HƯỚNG CHUẨN ROS 2]
     [u] : Tiến Rẽ Trái       [i] : TIẾN THẲNG       [o] : Tiến Rẽ Phải
     [j] : XOAY TRÁI          [k] : DỪNG XE          [l] : XOAY PHẢI
     [m] : Lùi Rẽ Trái        [,] : LÙI THẲNG        [.] : Lùi Rẽ Phải

  👉 PHANH DỪNG XE: [k] hoặc [Space]
  👉 ĐIỀU CHỈNH TỐC ĐỘ:
     [q] hoặc [1] : Tăng tốc độ (+10%)
     [z] hoặc [2] : Giảm tốc độ (-10%)
  👉 THOÁT CHƯƠNG TRÌNH: [Ctrl + C]
================================================================================
"""

MOVE_BINDINGS = {
    'i': (1.0, 0.0),
    'I': (1.0, 0.0),
    ',': (-1.0, 0.0),
    '<': (-1.0, 0.0),
    'j': (0.0, 1.0),
    'J': (0.0, 1.0),
    'l': (0.0, -1.0),
    'L': (0.0, -1.0),
    'u': (1.0, 1.0),
    'U': (1.0, 1.0),
    'o': (1.0, -1.0),
    'O': (1.0, -1.0),
    'm': (-1.0, -1.0),
    'M': (-1.0, -1.0),
    '.': (-1.0, 1.0),
    '>': (-1.0, 1.0),
    'k': (0.0, 0.0),
    'K': (0.0, 0.0),
    ' ': (0.0, 0.0),
}

SPEED_BINDINGS = {
    'q': 1.1,
    'Q': 1.1,
    '1': 1.1,
    '+': 1.1,
    '=': 1.1,
    'z': 0.9,
    'Z': 0.9,
    '2': 0.9,
    '-': 0.9,
    '_': 0.9,
}


def get_key(settings, timeout=0.1):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = rclpy.create_node('teleop_keyboard_node')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    linear_speed = 0.35   # m/s
    angular_speed = 0.80  # rad/s

    target_x = 0.0
    target_th = 0.0
    last_action = "DỪNG [k]"

    print(BANNER)
    print(f"👉 CHẾ ĐỘ CHẠY LIÊN TỤC: Bấm 1 lần là xe chạy liên tục, bấm [k] để dừng.")
    print(f"👉 Mức tốc độ: Dài = {linear_speed:.2f} m/s | Góc = {angular_speed:.2f} rad/s\n")

    try:
        while rclpy.ok():
            key = get_key(settings, timeout=0.08)

            if key in MOVE_BINDINGS:
                target_x = MOVE_BINDINGS[key][0]
                target_th = MOVE_BINDINGS[key][1]

                if target_x > 0 and target_th == 0: last_action = "TIẾN ⬆️ [i]"
                elif target_x < 0 and target_th == 0: last_action = "LÙI ⬇️ [,]"
                elif target_th > 0 and target_x == 0: last_action = "XOAY TRÁI ⬅️ [j]"
                elif target_th < 0 and target_x == 0: last_action = "XOAY PHẢI ➡️ [l]"
                elif target_x > 0 and target_th > 0: last_action = "TIẾN TRÁI ↖️ [u]"
                elif target_x > 0 and target_th < 0: last_action = "TIẾN PHẢI ↗️ [o]"
                elif target_x < 0 and target_th < 0: last_action = "LÙI TRÁI ↙️ [m]"
                elif target_x < 0 and target_th > 0: last_action = "LÙI PHẢI ↘️ [.]"
                else: last_action = "DỪNG [k]"

            elif key in SPEED_BINDINGS:
                factor = SPEED_BINDINGS[key]
                linear_speed = round(max(0.15, min(1.20, linear_speed * factor)), 2)
                angular_speed = round(max(0.20, min(2.50, angular_speed * factor)), 2)
                print(f"\n⚡ [TỐC ĐỘ MỚI] Dài: {linear_speed:.2f} m/s | Góc: {angular_speed:.2f} rad/s")

            elif key == '\x03':  # Ctrl+C
                break

            # Phát liên tục 12 Hz duy trì vận tốc đều đặn, nuôi ESP32 Watchdog
            twist = Twist()
            twist.linear.x = target_x * linear_speed
            twist.angular.z = target_th * angular_speed
            pub.publish(twist)

            print(f"\r[ĐIỀU KHIỂN] {last_action:<20} | v = {twist.linear.x:+.2f} m/s (Mức: {linear_speed:.2f} m/s) | w = {twist.angular.z:+.2f} rad/s", end="", flush=True)

    except Exception as e:
        print(f"\nLỗi: {e}")

    finally:
        twist = Twist()
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
        print("\n\nĐã dừng xe và thoát teleop an toàn.")


if __name__ == '__main__':
    main()
