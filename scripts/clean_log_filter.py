#!/usr/bin/env python3
"""
Real-time Terminal Output Cleaner for ROS 2 real-cnn.
Strips boilerplate noise, ROS 2 process IDs, ANSI prefixes, and logger names
so the user gets a sleek, beautiful, distraction-free terminal feed.
"""

import sys
import re

ansi_regex = re.compile(r'\x1b\[[0-9;]*[mK]')

skip_patterns = (
    'All log files can be found below',
    'Default logging verbosity is set to',
    'process started with pid',
    'process has died',
    'SLLidar running on ROS2',
    'Error, unexpected error, code: 80008004',
    'Starting ImuFilter',
    'Using dt computed from message headers',
    'The gravity vector is kept in the IMU message',
    'Imu filter gain set to',
    'Gyro drift bias set to',
    'Magnetometer bias values',
    'Robot initialized',
    'Waiting for robot_description',
    'WiFi Camera Bridge khởi động',
    'Still waiting for data on topic imu/data_raw',
    'Resolving model path for'
)


def clean_line(raw_line: str) -> str:
    # 1. Strip ANSI escape codes first
    clean_text = ansi_regex.sub('', raw_line).strip()
    if not clean_text:
        return ''

    # 2. Skip boilerplate / background noise lines
    for p in skip_patterns:
        if p in clean_text:
            return ''

    # 3. Strip all leading bracketed headers:
    # e.g. [cnn_driver-10] [INFO] [12345.678] [cnn_driver_node]: Message
    m = re.match(r'^(?:\[[^\]]*\]\s*)+(?::\s*)?(.*)', clean_text)
    msg = m.group(1).strip() if m else clean_text

    # 4. Remove unwanted camera topic clutter
    msg = msg.replace(' (/camera/color/image_raw)...', '...')
    msg = msg.replace('(/camera/color/image_raw)...', '...')
    msg = msg.replace('(/camera/color/image_raw)', '')

    return msg


def main():
    try:
        for raw_line in sys.stdin:
            cleaned = clean_line(raw_line)
            if cleaned:
                print(cleaned, flush=True)
    except (KeyboardInterrupt, BrokenPipeError):
        pass


if __name__ == '__main__':
    main()
