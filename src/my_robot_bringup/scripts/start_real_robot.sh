#!/usr/bin/env bash
# ============================================================
# 1-Click: Khởi Động Toàn Bộ Xe Thật (Raspberry Pi)
# Sensors: RPLIDAR C1 + USB Webcam + ESP32 Encoder
# Motors:  BTS7960 + CNN Driver
# ============================================================
set -e

source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
[ -f "/tmp/sllidar_ws/install/setup.bash" ] && source /tmp/sllidar_ws/install/setup.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"
[ -f "$WS_DIR/install/setup.bash" ] && source "$WS_DIR/install/setup.bash"

# Auto-detect LiDAR port
LIDAR_PORT=""
for port in /dev/rplidar /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyACM0; do
    [ -e "$port" ] && LIDAR_PORT="$port" && break
done

if [ -z "$LIDAR_PORT" ]; then
    echo "[ERROR] LiDAR not found! Plug in RPLIDAR C1."
    exit 1
fi

sudo chmod 666 "$LIDAR_PORT" 2>/dev/null || true

echo "============================================================"
echo "  REAL ROBOT BRINGUP"
echo "============================================================"
echo "  LiDAR:  $LIDAR_PORT (RPLIDAR C1, 460800 baud)"
echo "  Camera: USB Webcam (/dev/video0)"
echo "  Motor:  BTS7960 (GPIO)"
echo "  Ctrl+C to stop."
echo "============================================================"

ros2 launch my_robot_bringup real_robot.launch.py serial_port:="$LIDAR_PORT" "$@"
