#!/usr/bin/env bash
# ============================================================
# 1-Click: Quét Bản Đồ SLAM Thật (RPLIDAR C1 + ESP32 Encoder)
# ============================================================
set -e

source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
[ -f "/tmp/sllidar_ws/install/setup.bash" ] && source /tmp/sllidar_ws/install/setup.bash

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

echo "============================================================"
echo "  REAL SLAM MAPPING"
echo "============================================================"
echo "  LiDAR: $LIDAR_PORT"
echo "  Mode:  SLAM Toolbox (scan matching + loop closing)"
echo "  Walk around to build the map."
echo "  Save:  ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map"
echo "  Ctrl+C to stop."
echo "============================================================"

ros2 launch my_robot_bringup real_slam.launch.py serial_port:="$LIDAR_PORT" "$@"
