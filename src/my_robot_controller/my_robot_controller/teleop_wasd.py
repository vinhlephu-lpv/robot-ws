#!/usr/bin/env python3
"""
WASD Teleop Controller for ROS 2.
Control robot using familiar gaming keys: W, A, S, D, Space!
"""

import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

BANNER = """
=====================================================
          🎮 WASD ROBOT TELEOP CONTROLLER
=====================================================
  [Di Chuyển Cơ Bản]
        [W] : Tiến tới
  [A] : Rẽ trái    [D] : Rẽ phải
        [S] : Lùi lại

  [Di Chuyển Góc Cua]
  [Q] : Tiến rẽ trái      [E] : Tiến rẽ phải
  [Z] : Lùi rẽ trái       [C] : Lùi rẽ phải

  [Dừng Xe & Điều Chỉnh Tốc Độ]
  [Space] hoặc [X]        : DỪNG XE KHẨN CẤP
  [+] hoặc [1]            : Tăng tốc độ (+10%)
  [-] hoặc [2]            : Giảm tốc độ (-10%)

  [Ctrl + C]              : Thoát
=====================================================
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
    node = rclpy.create_node('teleop_wasd_node')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    linear_speed = 0.30   # m/s
    angular_speed = 0.80  # rad/s

    print(BANNER)
    print(f"👉 Tốc độ hiện tại: Dài = {linear_speed:.2f} m/s | Góc = {angular_speed:.2f} rad/s\n")

    x = 0.0
    th = 0.0

    try:
        while rclpy.ok():
            key = get_key(settings, timeout=0.1)

            if key in MOVE_BINDINGS:
                x = MOVE_BINDINGS[key][0]
                th = MOVE_BINDINGS[key][1]
                twist = Twist()
                twist.linear.x = x * linear_speed
                twist.angular.z = th * angular_speed
                pub.publish(twist)

                action = "DỪNG"
                if x > 0 and th == 0: action = "TIẾN ⬆️"
                elif x < 0 and th == 0: action = "LÙI ⬇️"
                elif th > 0 and x == 0: action = "RẼ TRÁI ⬅️"
                elif th < 0 and x == 0: action = "RẼ PHẢI ➡️"
                elif x > 0 and th > 0: action = "TIẾN TRÁI ↖️"
                elif x > 0 and th < 0: action = "TIẾN PHẢI ↗️"
                elif x < 0 and th < 0: action = "LÙI TRÁI ↙️"
                elif x < 0 and th > 0: action = "LÙI PHẢI ↘️"

                print(f"\r[ĐIỀU KHIỂN] {action:<14} | v = {twist.linear.x:+.2f} m/s | w = {twist.angular.z:+.2f} rad/s", end="", flush=True)

            elif key in SPEED_BINDINGS:
                factor = SPEED_BINDINGS[key]
                linear_speed = max(0.05, min(1.0, linear_speed * factor))
                angular_speed = max(0.1, min(2.5, angular_speed * factor))
                print(f"\r[TỐC ĐỘ MỚI] Dài = {linear_speed:.2f} m/s | Góc = {angular_speed:.2f} rad/s{' '*20}", end="", flush=True)

            elif key == '\x03':  # Ctrl+C
                break

    except Exception as e:
        print(f"\nLỗi: {e}")

    finally:
        # Publish zero twist on exit
        twist = Twist()
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
        print("\n\nĐã dừng xe và thoát teleop an toàn.")


if __name__ == '__main__':
    main()
