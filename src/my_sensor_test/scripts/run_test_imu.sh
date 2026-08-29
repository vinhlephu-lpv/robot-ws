#!/usr/bin/env bash
# ==============================================================================
# Script kiểm tra nhanh Cảm biến IMU 9 trục ICM-20948 trên Raspberry Pi (I2C)
# ==============================================================================

set -e

# Đổi màu chữ terminal
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${BLUE}================================================================${NC}"
echo -e "${BLUE}    🚀 KIỂM TRA CẢM BIẾN IMU 9 TRỤC ICM-20948 (I2C)            ${NC}"
echo -e "${BLUE}================================================================${NC}"

# 1. Kiểm tra bus I2C trên Linux
if [ ! -e "/dev/i2c-1" ] && [ ! -e "/dev/i2c-0" ]; then
    echo -e "${YELLOW}⚠️ Chưa bật giao tiếp I2C trong Linux/Raspberry Pi!${NC}"
    echo -e "👉 Chạy lệnh: ${GREEN}sudo raspi-config${NC} -> Interface Options -> I2C -> Enable -> Reboot."
fi

# 2. Quét địa chỉ I2C nếu có lệnh i2cdetect
if command -v i2cdetect &> /dev/null; then
    echo -e "🔍 Đang quét bus I2C-1 tìm ICM-20948 (Địa chỉ mặc định: 0x68 hoặc 0x69)..."
    sudo i2cdetect -y 1 2>/dev/null || true
fi

# 3. Nạp môi trường ROS 2
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

# Nạp workspace
SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WS_DIR="$( cd -- "$SCRIPT_DIR/../../.." &> /dev/null && pwd )"
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
fi

# Thiết lập CycloneDDS cấu hình mở (tránh lỗi buffer mạng)
if [ -f "$WS_DIR/cyclonedds.xml" ]; then
    export CYCLONEDDS_URI="file://$WS_DIR/cyclonedds.xml"
    cp -f "$WS_DIR/cyclonedds.xml" "$HOME/.cyclonedds.xml" 2>/dev/null || true
fi
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

echo -e "\n${GREEN}▶ Đang khởi chạy ROS 2 IMU Driver Node (/imu)...${NC}"
echo -e "${YELLOW}💡 Giữ yên xe trong 1-2 giây đầu để cảm biến tự động cân bằng Zero-Bias!${NC}"
echo -e "${YELLOW}💡 Nhấn Ctrl + C để dừng kiểm tra.${NC}\n"

ros2 launch my_sensor_test test_imu.launch.py
