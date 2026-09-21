#!/usr/bin/env bash
# ============================================================
# Cài đặt Driver, Udev Rules và Phân quyền cho Intel RealSense D435
# Chạy trên Raspberry Pi:
#   bash scripts/setup_realsense.sh
# ============================================================
set -e

echo "============================================================"
echo "  🔧 CÀI ĐẶT DRIVER & PHÂN QUYỀN INTEL REALSENSE D435"
echo "============================================================"

# 1. Thêm User vào các nhóm quyền hệ thống
echo "1. Thêm user '$USER' vào nhóm video, plugdev, dialout..."
sudo usermod -aG video,plugdev,dialout "$USER" || true

# 2. Cài đặt udev rules chính thức từ Intel RealSense
echo "2. Thiết lập Udev rules cho Intel RealSense..."
RULES_FILE="/etc/udev/rules.d/99-realsense-libusb.rules"
if [ ! -f "$RULES_FILE" ] || [ ! -s "$RULES_FILE" ]; then
    echo "   Đang tải udev rules chính thức từ Intel..."
    if command -v curl &>/dev/null; then
        sudo curl -sSL https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules -o "$RULES_FILE"
    elif command -v wget &>/dev/null; then
        sudo wget -qO "$RULES_FILE" https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules
    fi
fi

if [ -f "$RULES_FILE" ]; then
    echo "   ✅ Đã cài đặt $RULES_FILE"
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    echo "   ✅ Đã kích hoạt udev rules thành công."
else
    echo "   ⚠️ Không thể tải từ internet. Đang tạo rules cơ bản..."
    sudo bash -c 'cat << "EOF" > /etc/udev/rules.d/99-realsense-libusb.rules
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b07", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b3a", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="8086", MODE="0666", GROUP="video"
EOF'
    sudo udevadm control --reload-rules
    sudo udevadm trigger
fi

# 3. Cài đặt các gói công cụ phụ trợ (nếu chưa có)
echo "3. Kiểm tra các gói bổ trợ (v4l-utils)..."
if ! command -v v4l2-ctl &>/dev/null; then
    echo "   Cài đặt v4l-utils để soi cổng camera..."
    sudo apt-get update -qq && sudo apt-get install -y -qq v4l-utils || true
fi

echo ""
echo "============================================================"
echo "  🎉 CÀI ĐẶT THÀNH CÔNG!"
echo "============================================================"
echo "• Lưu ý: Hãy rút cáp USB RealSense ra và cắm lại vào cổng USB 3.0"
echo "• Chạy lệnh kiểm tra kết nối ngay: python3 scripts/test_realsense.py"
echo "============================================================"
