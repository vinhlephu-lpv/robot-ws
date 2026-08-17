#!/usr/bin/env bash
# ============================================================
# Script 1-Click: Test Camera trên Laptop/PC
# Tự động nhận diện Orbbec Astra Mini S hoặc USB Webcam
# ============================================================
set -e

# Source ROS 2
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

# Source Astra Driver Workspace
if [ -f "/tmp/astra_ws/install/setup.bash" ]; then
    source /tmp/astra_ws/install/setup.bash
fi

# Source Main Workspace
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
fi

# Check if Astra Camera is plugged in
if lsusb | grep -q "2bc5:"; then
    echo "============================================================"
    echo "  [MY_SENSOR_TEST] ĐÃ KẾT NỐI ORBBEC ASTRA MINI S (3D)"
    echo "============================================================"
    echo "• Topic Màu (RGB): /camera/color/image_raw"
    echo "• Topic Độ Sâu (Depth): /camera/depth/image_raw"
    echo "• Topic PointCloud: /camera/depth/points"
    echo "• Đang mở luồng phát và cửa sổ xem trực tiếp..."
    echo "• Bấm Ctrl+C trong Terminal này để tắt."
    echo "------------------------------------------------------------"

    ros2 launch astra_camera astra.launch.xml "$@" &
    DRIVER_PID=$!

    trap "kill -9 $DRIVER_PID 2>/dev/null || true" EXIT INT TERM

    sleep 3
    ros2 run rqt_image_view rqt_image_view /camera/color/image_raw
else
    echo "============================================================"
    echo "  [MY_SENSOR_TEST] ĐANG KHỞI CHẠY KIỂM THỬ USB WEBCAM"
    echo "============================================================"
    echo "• Thiết bị: /dev/video0"
    echo "• Topic: /camera/image_raw"
    echo "• Bấm Ctrl+C trong Terminal này để tắt."
    echo "------------------------------------------------------------"

    ros2 launch my_sensor_test test_camera.launch.py "$@"
fi
