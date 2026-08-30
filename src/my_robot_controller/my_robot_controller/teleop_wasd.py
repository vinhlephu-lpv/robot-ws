#!/usr/bin/env python3
"""
WASD Teleop Controller for ROS 2.
Control robot using familiar gaming keys: W, A, S, D, Space!
"""

import sys
import time
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

    linear_speed = 0.35   # m/s
    angular_speed = 0.80  # rad/s

    print(BANNER)
    print(f"👉 Chế độ Lái Tự Động Ngắt: Nhấn giữ phím để xe chạy, THẢ TAY RA LÀ XE TỰ DỪNG.")
    print(f"👉 Tốc độ hiện tại: Dài = {linear_speed:.2f} m/s | Góc = {angular_speed:.2f} rad/s\n")

    target_x = 0.0
    target_th = 0.0
    last_action = "DỪNG"
    last_key_time = 0.0
    KEY_TIMEOUT = 0.30  # Giây: Quá 0.3s không bấm giữ phím -> tự phanh dừng

    try:
        while rclpy.ok():
            key = get_key(settings, timeout=0.06)

            if key in MOVE_BINDINGS:
                target_x = MOVE_BINDINGS[key][0]
                target_th = MOVE_BINDINGS[key][1]
                last_key_time = time.time()

                if target_x > 0 and target_th == 0: last_action = "TIẾN ⬆️"
                elif target_x < 0 and target_th == 0: last_action = "LÙI ⬇️"
                elif target_th > 0 and target_x == 0: last_action = "RẼ TRÁI ⬅️"
                elif target_th < 0 and target_x == 0: last_action = "RẼ PHẢI ➡️"
                elif target_x > 0 and target_th > 0: last_action = "TIẾN TRÁI ↖️"
                elif target_x > 0 and target_th < 0: last_action = "TIẾN PHẢI ↗️"
                elif target_x < 0 and target_th < 0: last_action = "LÙI TRÁI ↙️"
                elif target_x < 0 and target_th > 0: last_action = "LÙI PHẢI ↘️"
                else: last_action = "DỪNG"

            elif key in SPEED_BINDINGS:
                factor = SPEED_BINDINGS[key]
                linear_speed = round(max(0.15, min(1.20, linear_speed * factor)), 2)
                angular_speed = round(max(0.20, min(2.50, angular_speed * factor)), 2)
                print(f"\n⚡ [TỐC ĐỘ MỚI] Dài: {linear_speed:.2f} m/s | Góc: {angular_speed:.2f} rad/s")

            elif key == '\x03':  # Ctrl+C
                break

            # Tự động phanh dừng khi nhả tay không còn bấm giữ phím
            if (target_x != 0.0 or target_th != 0.0) and (time.time() - last_key_time > KEY_TIMEOUT):
                target_x = 0.0
                target_th = 0.0
                last_action = "DỪNG"

            twist = Twist()
            twist.linear.x = target_x * linear_speed
            twist.angular.z = target_th * angular_speed
            pub.publish(twist)

            print(f"\r[ĐIỀU KHIỂN] {last_action:<14} | v = {twist.linear.x:+.2f} m/s (Mức: {linear_speed:.2f} m/s) | w = {twist.angular.z:+.2f} rad/s", end="", flush=True)

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
