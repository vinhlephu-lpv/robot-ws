#!/usr/bin/env bash
# ============================================================
# Script 1-Click: Test RPLIDAR C1 + Mô Hình Robot 3D trên RViz2
# ============================================================
set -e

# Source ROS 2
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

# Source SLLIDAR Workspace
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
echo "    [MY_SENSOR_TEST] TEST TIA QUÉT RPLIDAR C1 + MÔ HÌNH XE"
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
    echo "👉 Vui lòng CẮM CÁP USB của RPLIDAR C1 vào cổng USB máy tính."
    exit 1
fi

sudo chmod 666 "$LIDAR_PORT" 2>/dev/null || chmod 666 "$LIDAR_PORT" 2>/dev/null || true
echo "• Cổng kết nối: $LIDAR_PORT"
echo "• Cảm biến: Slamtec RPLIDAR C1 (Baudrate: 460800)"
echo "• Hiển thị: Mô hình xe 3D ở tâm + Tia quét đỏ quanh phòng"
echo "• Bấm Ctrl+C để tắt."
echo "------------------------------------------------------------"

ros2 launch my_sensor_test test_lidar.launch.py serial_port:="$LIDAR_PORT" "$@"
