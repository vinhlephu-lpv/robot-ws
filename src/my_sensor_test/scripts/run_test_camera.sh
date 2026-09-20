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

DEV_PATH="${1:-realsense}"

# Tự động dò tìm camera nếu không chỉ định cụ thể
if [ "$DEV_PATH" = "realsense" ] || [ "$DEV_PATH" = "auto" ]; then
    # 1. Kiểm tra RealSense
    if ls /dev/v4l/by-id/*RealSense* /dev/v4l/by-id/*realsense* /dev/v4l/by-id/*D435* &>/dev/null || (command -v lsusb &>/dev/null && lsusb 2>/dev/null | grep -Ei "8086:0b07|8086:0b3a|8086:0b5c|8086:0b64|8086:0aa5|RealSense" &>/dev/null); then
        DEV_PATH="realsense"
    # 2. Kiểm tra iPhone
    elif ip route 2>/dev/null | grep -q "172.20.10" || ping -c 1 -W 1 172.20.10.1 &>/dev/null; then
        DEV_PATH="http://172.20.10.1:4747/video"
    # 3. Kiểm tra Webcam
    elif [ -e "/dev/video0" ]; then
        DEV_PATH="/dev/video0"
    else
        DEV_PATH="realsense"
    fi
fi

echo "============================================================"
echo "    [MY_SENSOR_TEST] ĐANG KIỂM TRA CAMERA (AUTO-DISCOVERY)"
echo "============================================================"
echo "• Thiết bị camera: $DEV_PATH"
echo "• Thứ tự ưu tiên : 1. Intel D435 -> 2. iPhone -> 3. Webcam"
echo "• Topic màu (RGB): /camera/color/image_raw"
echo "• Đang mở cửa sổ rqt_image_view..."
echo "• Bấm Ctrl+C trong Terminal này để dừng."
echo "============================================================"

ros2 launch my_sensor_test test_camera.launch.py video_device:="$DEV_PATH"

