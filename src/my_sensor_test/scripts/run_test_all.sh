#!/usr/bin/env bash
# ============================================================
# Script 1-Click: Test CẢ CAMERA VÀ LIDAR trên cùng 1 RViz2
# ============================================================
set -e

# Source ROS 2
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

# Source Workspace
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
fi

echo "============================================================"
echo "    [MY_SENSOR_TEST] ĐANG TEST ĐỒNG THỜI CAMERA & LIDAR"
echo "============================================================"

# Cấp quyền cổng serial nếu có
if [ -e "/dev/ttyUSB0" ]; then
    sudo chmod 666 /dev/ttyUSB0 2>/dev/null || chmod 666 /dev/ttyUSB0 2>/dev/null || true
elif [ -e "/dev/ttyACM0" ]; then
    sudo chmod 666 /dev/ttyACM0 2>/dev/null || chmod 666 /dev/ttyACM0 2>/dev/null || true
fi

echo "• Đang mở RViz2 hiển thị đồng thời Camera Overlay + LiDAR 360°..."
echo "• Bấm Ctrl+C để tắt."
echo "------------------------------------------------------------"

ros2 launch my_sensor_test test_all_sensors.launch.py "$@"
