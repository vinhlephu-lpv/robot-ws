#!/usr/bin/env bash
# ============================================================
# Script 1-Click: Test Camera trên Laptop/PC
# Tự động nhận diện Orbbec Astra Pro / Series hoặc USB Webcam
# Hỗ trợ revert nhanh về Webcam bất kỳ lúc nào:
#   test-cam          -> Mặc định test Orbbec Astra 3D Camera
#   test-cam webcam   -> Revert test USB Webcam V4L2
# ============================================================
set -e

# Source ROS 2
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

# Source Astra Driver Workspace
if [ -f "$HOME/astra_ws/install/setup.bash" ]; then
    source "$HOME/astra_ws/install/setup.bash"
elif [ -f "/tmp/astra_ws/install/setup.bash" ]; then
    source "/tmp/astra_ws/install/setup.bash"
fi

# Source Main Workspace
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
fi

MODE="${1:-auto}"
if [ "$MODE" == "webcam" ] || [ "$MODE" == "v4l2" ]; then
    FORCE_WEBCAM=true
    shift || true
else
    FORCE_WEBCAM=false
fi

# 1. Kiểm tra nếu người dùng yêu cầu ép dùng USB Webcam (Revert)
if [ "$FORCE_WEBCAM" = true ]; then
    DEV_PATH="/dev/video0"
    if [ -e "/dev/video0" ]; then
        DEV_PATH="/dev/video0"
    fi

    echo "============================================================"
    echo "  [MY_SENSOR_TEST] ĐANG TEST USB WEBCAM (V4L2)"
    echo "============================================================"
    echo "• Cổng thiết bị: $DEV_PATH"
    echo "• Topic xem hình: /camera/color/image_raw (hoặc /camera/image_raw)"
    echo "• Đang mở cửa sổ rqt_image_view..."
    echo "• Bấm Ctrl+C trong Terminal này để dừng."
    echo "------------------------------------------------------------"

    ros2 launch my_sensor_test test_camera.launch.py camera_driver:=v4l2 video_device:="$DEV_PATH" "$@"
    exit 0
fi

# 2. Kiểm tra nếu có camera Orbbec Astra cắm vào máy
if lsusb | grep -q "2bc5:"; then
    echo "============================================================"
    echo "  [MY_SENSOR_TEST] ĐÃ KẾT NỐI ORBBEC ASTRA 3D CAMERA"
    echo "============================================================"
    echo "• Topic Màu (RGB):      /camera/color/image_raw"
    echo "• Topic Độ Sâu (Depth):  /camera/depth/image_raw"
    echo "• Topic PointCloud 3D:  /camera/depth/points"
    echo "• Topic IR Hồng ngoại:   /camera/ir/image_raw"
    echo "------------------------------------------------------------"
    echo "💡 MẸO: Để đổi về USB Webcam cũ bất kỳ lúc nào, chạy: test-cam webcam"
    echo "• Đang khởi động driver camera và mở rqt_image_view..."
    echo "• Bấm Ctrl+C trong Terminal này để dừng."
    echo "============================================================"

    # Chọn launch file phù hợp với dòng Astra Pro (PID 0403/0501) hoặc Astra tiêu chuẩn
    if lsusb | grep -q "2bc5:0403" || lsusb | grep -q "2bc5:0501"; then
        LAUNCH_FILE="astra_pro.launch.xml"
    else
        LAUNCH_FILE="astra.launch.xml"
    fi

    ros2 launch astra_camera "$LAUNCH_FILE" "$@" &
    DRIVER_PID=$!

    trap "kill -9 $DRIVER_PID 2>/dev/null || true" EXIT INT TERM

    sleep 3
    ros2 run rqt_image_view rqt_image_view /camera/color/image_raw
else
    # Fallback to USB webcam device nếu không có Astra
    DEV_PATH="/dev/video0"
    if [ -e "/dev/video0" ]; then
        DEV_PATH="/dev/video0"
    fi

    echo "============================================================"
    echo "  [MY_SENSOR_TEST] KHÔNG TÌM THẤY ASTRA -> DÙNG USB WEBCAM"
    echo "============================================================"
    echo "• Cổng thiết bị: $DEV_PATH"
    echo "• Topic xem hình: /camera/color/image_raw"
    echo "• Đang mở cửa sổ rqt_image_view..."
    echo "• Bấm Ctrl+C trong Terminal này để dừng."
    echo "------------------------------------------------------------"

    ros2 launch my_sensor_test test_camera.launch.py camera_driver:=v4l2 video_device:="$DEV_PATH" "$@"
fi
