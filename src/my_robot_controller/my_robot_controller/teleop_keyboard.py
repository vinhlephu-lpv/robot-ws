#!/usr/bin/env python3
"""
BÀN PHÍM ĐIỀU KHIỂN ROBOT CHUẨN ROS 2 (CHẾ ĐỘ CHẠY LIÊN TỤC KHÔNG NGẮT)
- Hỗ trợ cả 2 bộ phím: Chuẩn ROS 2 (i, ,, j, l, k) và Chuẩn WASD (w, s, a, d, x/space)
- Tự động phát liên tục 10 Hz nuôi ESP32 Watchdog: Bấm 1 lần là xe chạy liên tục không bao giờ bị khựng!
- Bấm [k], [Space] hoặc [x] để DỪNG XE.
"""

import sys
import time
import threading
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

BANNER = """
================================================================================
          🎮 BÀN PHÍM ĐIỀU KHIỂN ROBOT CHUẨN ROS 2 (CHẠY LIÊN TỤC)
================================================================================
  [PHÍM ĐIỀU HƯỚNG CHUẨN ROS 2]             [PHÍM ĐIỀU HƯỚNG WASD]
     [u] : Tiến Trái   [i] : TIẾN THẲNG   [o] : Tiến Phải      [Q] : Tiến Trái   [W] : TIẾN THẲNG   [E] : Tiến Phải
     [j] : XOAY TRÁI   [k] : DỪNG XE      [l] : XOAY PHẢI      [A] : XOAY TRÁI   [S] : LÙI THẲNG    [D] : XOAY PHẢI
     [m] : Lùi Trái    [,] : LÙI THẲNG    [.] : Lùi Phải       [Z] : Lùi Trái    [X] : DỪNG XE      [C] : Lùi Phải

  👉 PHANH DỪNG XE: [k], [Space] (Phím Cách), hoặc [x]
  👉 ĐIỀU CHỈNH TỐC ĐỘ:
     [1] hoặc [+] : Tăng tốc (+10%)
     [2] hoặc [-] : Giảm tốc (-10%)
  👉 THOÁT CHƯƠNG TRÌNH: [Ctrl + C]
================================================================================
"""

MOVE_BINDINGS = {
    # Chuẩn ROS 2
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

    # Chuẩn WASD
    'w': (1.0, 0.0),
    'W': (1.0, 0.0),
    's': (-1.0, 0.0),
    'S': (-1.0, 0.0),
    'a': (0.0, 1.0),
    'A': (0.0, 1.0),
    'd': (0.0, -1.0),
    'D': (0.0, -1.0),
    'q': (1.0, 1.0),
    'Q': (1.0, 1.0),
    'e': (1.0, -1.0),
    'E': (1.0, -1.0),
    'z': (-1.0, -1.0),
    'Z': (-1.0, -1.0),
    'c': (-1.0, 1.0),
    'C': (-1.0, 1.0),
    'x': (0.0, 0.0),
    'X': (0.0, 0.0),
    ' ': (0.0, 0.0),
}

SPEED_BINDINGS = {
    '+': 1.1,
    '=': 1.1,
    '1': 1.1,
    '-': 0.9,
    '_': 0.9,
    '2': 0.9,
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
    last_action = "DỪNG"

    print(BANNER)
    print(f"👉 CHẾ ĐỘ CHẠY LIÊN TỤC: Bấm 1 lần là xe chạy liên tục, bấm [k] hoặc [Space] để dừng.")
    print(f"👉 Mức tốc độ: Dài = {linear_speed:.2f} m/s | Góc = {angular_speed:.2f} rad/s\n")

    try:
        while rclpy.ok():
            key = get_key(settings, timeout=0.08)

            if key in MOVE_BINDINGS:
                target_x = MOVE_BINDINGS[key][0]
                target_th = MOVE_BINDINGS[key][1]

                if target_x > 0 and target_th == 0: last_action = "TIẾN ⬆️ (i/w)"
                elif target_x < 0 and target_th == 0: last_action = "LÙI ⬇️ (, / s)"
                elif target_th > 0 and target_x == 0: last_action = "XOAY TRÁI ⬅️ (j/a)"
                elif target_th < 0 and target_x == 0: last_action = "XOAY PHẢI ➡️ (l/d)"
                elif target_x > 0 and target_th > 0: last_action = "TIẾN TRÁI ↖️ (u/q)"
                elif target_x > 0 and target_th < 0: last_action = "TIẾN PHẢI ↗️ (o/e)"
                elif target_x < 0 and target_th < 0: last_action = "LÙI TRÁI ↙️ (m/z)"
                elif target_x < 0 and target_th > 0: last_action = "LÙI PHẢI ↘️ (. / c)"
                else: last_action = "DỪNG [k/Space]"

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
