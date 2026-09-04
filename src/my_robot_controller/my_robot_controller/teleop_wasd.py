#!/usr/bin/env python3
"""
WASD Teleop Controller for ROS 2 (Chạy liên tục không ngắt).
Điều khiển robot mượt mà bằng các phím WASD: Bấm 1 lần là xe chạy liên tục!
Bấm [Space], [X] hoặc [K] để DỪNG XE.
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
          🎮 BÀN PHÍM ĐIỀU KHIỂN WASD (CHẠY LIÊN TỤC KHÔNG TỰ DỪNG)
================================================================================
  [ĐIỀU HƯỚNG ROBOT]
      [Q] : Tiến Rẽ Trái       [W] : TIẾN THẲNG       [E] : Tiến Rẽ Phải
      [A] : XOAY TRÁI          [X] / [Space] : DỪNG    [D] : XOAY PHẢI
      [Z] : Lùi Rẽ Trái        [S] : LÙI THẲNG        [C] : Lùi Rẽ Phải

  👉 PHANH DỪNG XE: [Space], [X], [K]
  👉 ĐIỀU CHỈNH TỐC ĐỘ:
     [+] hoặc [1] : Tăng tốc độ (+10%)
     [-] hoặc [2] : Giảm tốc độ (-10%)
  👉 THOÁT CHƯƠNG TRÌNH: [Ctrl + C]
================================================================================
"""

MOVE_BINDINGS = {
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
    'k': (0.0, 0.0),
    'K': (0.0, 0.0),
    ' ': (0.0, 0.0),
    # Hỗ trợ thêm cả các phím tiêu chuẩn i, j, k, l, ,
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
}

SPEED_BINDINGS = {
    '+': 1.1,
    '=': 1.1,
    '1': 1.1,
    '-': 0.9,
    '_': 0.9,
    '2': 0.9,
}


def get_key(settings, timeout=0.05):
    if settings is not None:
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            return sys.stdin.read(1)
        return ''
    else:
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            return sys.stdin.read(1)
        return ''


def main():
    settings = None
    if sys.stdin.isatty():
        try:
            settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        except Exception:
            settings = None

    rclpy.init()
    node = rclpy.create_node('teleop_wasd_node')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    linear_speed = 0.35   # m/s (Khởi đầu êm ái, phù hợp tải động cơ 775)
    angular_speed = 0.80  # rad/s

    print(BANNER)
    print(f"👉 CHẾ ĐỘ CHẠY LIÊN TỤC: Bấm [W] là xe tiến liên tục cho đến khi bấm [Space] hoặc [X] để dừng.")
    print(f"👉 Mức tốc độ: Dài = {linear_speed:.2f} m/s | Góc = {angular_speed:.2f} rad/s\n")

    target_x = 0.0
    target_th = 0.0
    last_action = "DỪNG [Space]"

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            key = get_key(settings, timeout=0.05)

            if key in MOVE_BINDINGS:
                target_x = MOVE_BINDINGS[key][0]
                target_th = MOVE_BINDINGS[key][1]

                if target_x > 0 and target_th == 0: last_action = "TIẾN ⬆️ [W]"
                elif target_x < 0 and target_th == 0: last_action = "LÙI ⬇️ [S]"
                elif target_th > 0 and target_x == 0: last_action = "XOAY TRÁI ⬅️ [A]"
                elif target_th < 0 and target_x == 0: last_action = "XOAY PHẢI ➡️ [D]"
                elif target_x > 0 and target_th > 0: last_action = "TIẾN TRÁI ↖️ [Q]"
                elif target_x > 0 and target_th < 0: last_action = "TIẾN PHẢI ↗️ [E]"
                elif target_x < 0 and target_th < 0: last_action = "LÙI TRÁI ↙️ [Z]"
                elif target_x < 0 and target_th > 0: last_action = "LÙI PHẢI ↘️ [C]"
                else: last_action = "DỪNG [Space]"

            elif key in SPEED_BINDINGS:
                factor = SPEED_BINDINGS[key]
                linear_speed = round(max(0.05, min(1.20, linear_speed * factor)), 2)
                angular_speed = round(max(0.10, min(2.50, angular_speed * factor)), 2)
                print(f"\n⚡ [TỐC ĐỘ MỚI] Dài: {linear_speed:.2f} m/s | Góc: {angular_speed:.2f} rad/s")

            elif key == '\x03':  # Ctrl+C
                break

            # Phát liên tục ~20 Hz duy trì vận tốc đều đặn, nuôi ESP32 Watchdog
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
        if settings is not None:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()
        print("\n\nĐã dừng xe và thoát teleop an toàn.")


if __name__ == '__main__':
    main()
