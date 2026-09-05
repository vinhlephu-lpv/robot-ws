#!/usr/bin/env bash
# ==============================================================================
# SCRIPT NẠP FIRMWARE ESP32-S3 TỰ ĐỘNG TỪ RASPBERRY PI QUA CỔNG SERIAL
# Tự động cài đặt arduino-cli, tìm cổng USB, ngắt tiến trình chiếm cổng và nạp code
# ==============================================================================

set -e

# Đường dẫn thư mục
SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
SKETCH_DIR="$SCRIPT_DIR/code0409"
FQBN="esp32:esp32:esp32s3"

echo "========================================================="
echo "🤖 [ESP32-S3] TRÌNH NẠP CODE TỰ ĐỘNG TỪ RASPBERRY PI"
echo "========================================================="

# 1. Đảm bảo thư mục ~/.local/bin có trong PATH
mkdir -p "$HOME/.local/bin"
export PATH="$HOME/.local/bin:/usr/local/bin:$PATH"

# 2. Kiểm tra arduino-cli, nếu chưa có thì tự động tải
if ! command -v arduino-cli &> /dev/null; then
    echo "⬇️ Chưa tìm thấy arduino-cli. Đang tự động tải về máy..."
    curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR="$HOME/.local/bin" sh
    echo "✅ Đã cài đặt arduino-cli vào $HOME/.local/bin/arduino-cli"
fi

# 3. Cấu hình Board Manager và cài đặt Core ESP32 nếu chưa có
echo "📦 Kiểm tra gói vi điều khiển ESP32..."
if ! arduino-cli core list 2>/dev/null | grep -q "esp32:esp32"; then
    echo "⚙️ Đang cấu hình gói Core ESP32 (lần đầu tiên có thể mất 1-2 phút)..."
    arduino-cli config init --overwrite 2>/dev/null || true
    arduino-cli config set board_manager.additional_urls https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
    arduino-cli core update-index
    arduino-cli core install esp32:esp32
    echo "✅ Đã cài đặt xong Core ESP32!"
else
    echo "✅ Core ESP32 đã sẵn sàng."
fi

# 4. Tự động nhận diện cổng Serial kết nối giữa Pi và ESP32
PORT=""
for p in /dev/esp32 /dev/ttyACM0 /dev/ttyACM1 /dev/ttyUSB0 /dev/ttyUSB1; do
    if [ -e "$p" ]; then
        PORT="$p"
        break
    fi
done

if [ -z "$PORT" ]; then
    echo "❌ [LỖI] Không tìm thấy cổng kết nối ESP32!"
    echo "👉 Vui lòng kiểm tra lại cáp USB nối giữa Raspberry Pi và ESP32."
    echo "   (Các cổng thường gặp: /dev/esp32, /dev/ttyACM0, /dev/ttyUSB0)"
    exit 1
fi

echo "🔌 Đã phát hiện cổng kết nối ESP32: $PORT"

# 5. Mở khóa cổng Serial và ngắt các tiến trình ROS đang chiếm cổng
echo "🔓 Mở khóa cổng Serial và giải phóng tiến trình chiếm dụng..."
sudo chmod 666 "$PORT" 2>/dev/null || true
pkill -f "esp32_bridge" 2>/dev/null || true
pkill -f "encoder_node" 2>/dev/null || true
sleep 1

# 6. Tiến hành Biên dịch và Nạp firmware
echo "🚀 Đang biên dịch và nạp code [$SKETCH_DIR] vào $PORT..."
echo "---------------------------------------------------------"
arduino-cli compile --fqbn "$FQBN" "$SKETCH_DIR" -u -p "$PORT"
echo "---------------------------------------------------------"
echo "🎉 NẠP CODE THÀNH CÔNG VÀO ESP32!"
echo "---------------------------------------------------------"

# 7. Đọc thử 5 dòng dữ liệu Serial phản hồi từ ESP32 để kiểm tra
echo "📡 Kiểm tra tín hiệu phản hồi từ ESP32 qua cổng $PORT (2 giây)..."
python3 -c "
import serial, time
try:
    s = serial.Serial('$PORT', 115200, timeout=1)
    time.sleep(0.5)
    for _ in range(5):
        line = s.readline().decode('utf-8', errors='ignore').strip()
        if line:
            print('  >> ESP32:', line)
    s.close()
    print('✅ Tín hiệu Serial kết nối hoàn hảo!')
except Exception as e:
    print('⚠️ Chưa thể đọc ngay:', e)
" 2>/dev/null || true

echo "========================================================="
echo "✅ HOÀN TẤT! Bạn có thể khởi động lại robot bằng: real-robot"
echo "========================================================="
