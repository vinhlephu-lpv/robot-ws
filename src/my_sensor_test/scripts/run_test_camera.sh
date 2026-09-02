#!/usr/bin/env bash
# ============================================================
# Script 1-Click: Test USB Webcam trên Laptop/PC
# Sử dụng v4l2_camera và hiển thị qua rqt_image_view
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

DEV_PATH="${1:-/dev/video0}"
if [ ! -e "$DEV_PATH" ] && [ -e "/dev/video0" ]; then
    DEV_PATH="/dev/video0"
fi

echo "============================================================"
echo "    [MY_SENSOR_TEST] ĐANG TEST USB WEBCAM (V4L2)"
echo "============================================================"
echo "• Thiết bị camera: $DEV_PATH"
echo "• Topic màu (RGB): /camera/color/image_raw"
echo "• Đang mở cửa sổ rqt_image_view..."
echo "• Bấm Ctrl+C trong Terminal này để dừng."
echo "============================================================"

ros2 launch my_sensor_test test_camera.launch.py video_device:="$DEV_PATH"

