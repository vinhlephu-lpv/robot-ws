#!/usr/bin/env bash
# ============================================================
# 1-Click: Tự Hành Thật với Nav2 (Cần bản đồ đã quét)
# ============================================================
set -e

source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
[ -f "/tmp/sllidar_ws/install/setup.bash" ] && source /tmp/sllidar_ws/install/setup.bash
[ -f "/tmp/astra_ws/install/setup.bash" ] && source /tmp/astra_ws/install/setup.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"
[ -f "$WS_DIR/install/setup.bash" ] && source "$WS_DIR/install/setup.bash"

LIDAR_PORT=""
for port in /dev/rplidar /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyACM0; do
    [ -e "$port" ] && LIDAR_PORT="$port" && break
done

if [ -z "$LIDAR_PORT" ]; then
    echo "[ERROR] LiDAR not found!"
    exit 1
fi

sudo chmod 666 "$LIDAR_PORT" 2>/dev/null || true

MAP_FILE="${1:-$HOME/maps/my_map.yaml}"

if [ ! -f "$MAP_FILE" ]; then
    echo "[ERROR] Map file not found: $MAP_FILE"
    echo "  Run start_real_slam.sh first to create a map."
    echo "  Usage: $0 /path/to/map.yaml"
    exit 1
fi

echo "============================================================"
echo "  REAL NAV2 AUTONOMOUS NAVIGATION"
echo "============================================================"
echo "  LiDAR: $LIDAR_PORT"
echo "  Map:   $MAP_FILE"
echo "  Click '2D Goal Pose' in RViz to send the robot."
echo "  Ctrl+C to stop."
echo "============================================================"

ros2 launch my_robot_bringup real_nav.launch.py \
    serial_port:="$LIDAR_PORT" \
    map:="$MAP_FILE" \
    "$@"
