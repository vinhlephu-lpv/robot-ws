#!/usr/bin/env bash
# ============================================================
# Script 1-Click: Test Quét Bản Đồ SLAM Thực Tế bằng RPLIDAR C1
# ============================================================
set -e

# Source ROS 2
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

# Source SLLidar Workspace
if [ -f "/tmp/sllidar_ws/install/setup.bash" ]; then
    source /tmp/sllidar_ws/install/setup.bash
fi

# Source Main Workspace
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
fi

echo "============================================================"
echo "  [MY_SENSOR_TEST] ĐANG KHỞI CHẠY QUÉT BẢN ĐỒ SLAM (RPLIDAR C1)"
echo "============================================================"

# Tự động tìm cổng serial của LiDAR
LIDAR_PORT=""
for port in /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyACM0 /dev/ttyACM1; do
    if [ -e "$port" ]; then
        LIDAR_PORT="$port"
        break
    fi
done

if [ -z "$LIDAR_PORT" ]; then
    echo "❌ LỖI: Chưa phát hiện cổng USB của LiDAR!"
    echo "👉 Vui lòng CẮM CÁP USB của RPLIDAR C1 vào cổng USB máy tính rồi chạy lại."
    exit 1
fi

sudo chmod 666 "$LIDAR_PORT" 2>/dev/null || chmod 666 "$LIDAR_PORT" 2>/dev/null || true
echo "• Đã kết nối LiDAR tại cổng: $LIDAR_PORT"
echo "• Cảm biến: Slamtec RPLIDAR C1 (Baudrate: 460800)"
echo "• Thuật toán: SLAM Toolbox (Scan Matching)"
echo "• Hiển thị: RViz2 (Bản đồ /map + Tia LiDAR /scan)"
echo "• Bấm Ctrl+C để tắt."
echo "------------------------------------------------------------"

ros2 launch my_sensor_test test_slam.launch.py serial_port:="$LIDAR_PORT" "$@"
