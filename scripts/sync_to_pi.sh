#!/usr/bin/env bash
# ==============================================================================
# Script: sync_to_pi.sh
# Đồng bộ mã nguồn độc lập sang thư mục /home/bao/robot-vinh trên Raspberry Pi
# (Hoàn toàn tách biệt với dự án /home/bao/robot_ws của bạn cùng nhóm)
# Cách dùng:
#   ./scripts/sync_to_pi.sh [IP_CUA_PI]
# ==============================================================================

set -e

SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WS_DIR="$(dirname "$SCRIPT_DIR")"

PI_USER="bao"
PI_WS_NAME="robot-vinh"
REMOTE_WS="/home/${PI_USER}/${PI_WS_NAME}"

# Tự động dò IP của Pi thông minh:
TARGET_IP="${1:-}"
if [ -n "$TARGET_IP" ] && ping -c 1 -W 1 "$TARGET_IP" >/dev/null 2>&1; then
    PI_IP="$TARGET_IP"
else
    PI_CANDIDATES=("10.42.0.236" "${PI_IP:-}" "$TARGET_IP" "10.10.178.200" "192.168.10.2" "bao-desktop.local")
    FOUND_IP=""
    for cand in "${PI_CANDIDATES[@]}"; do
        [ -z "$cand" ] && continue
        if ping -c 1 -W 1 "$cand" >/dev/null 2>&1; then
            FOUND_IP="$cand"
            break
        fi
    done
    PI_IP="${FOUND_IP:-10.42.0.236}"
fi

echo "📡 ============================================================"
echo "🚀 ĐỒNG BỘ MÃ NGUỒN PC ➔ RASPBERRY PI (WORKSPACE ĐỘC LẬP: ${PI_WS_NAME})"
echo "   IP Pi          : $PI_IP (User: $PI_USER)"
echo "   Thư mục PC     : $WS_DIR"
echo "   Thư mục Pi     : $REMOTE_WS (Độc lập 100% với robot_ws của bạn cùng nhóm)"
echo "   Chế độ         : Đồng bộ sạch (--delete tự xóa file rác cũ)"
echo "📡 ============================================================"

# 1. Kiểm tra kết nối mạng tới Pi
echo "🔍 Đang kiểm tra kết nối mạng tới $PI_IP..."
if ! ping -c 1 -W 2 "$PI_IP" >/dev/null 2>&1; then
    echo "❌ Không thể ping thấy Raspberry Pi tại địa chỉ: $PI_IP"
    echo "⚠️ Nguyên nhân có thể do:"
    echo "   1. Pi đang tắt nguồn hoặc đang khởi động lại."
    echo "   2. Dây cáp mạng LAN hoặc Wi-Fi chưa kết nối."
    echo "   3. Pi bị đổi IP DHCP."
    echo ""
    echo "💡 Gợi ý: Nếu biết IP mới của Pi, hãy chạy: sync-pi <IP_MOI>"
    exit 1
fi
echo "✅ Kết nối mạng OK!"

# Đảm bảo thư mục gốc /home/bao/robot-vinh tồn tại trên Pi
ssh -o StrictHostKeyChecking=no "${PI_USER}@${PI_IP}" "mkdir -p ${REMOTE_WS}/src ${REMOTE_WS}/scripts ${REMOTE_WS}/logs ${REMOTE_WS}/recordings"

# 2. Cấu hình rsync đồng bộ sạch 100% toàn bộ src/ và scripts/ sang /home/bao/robot-vinh
RSYNC_OPTS=(
  -avz
  --delete
  --progress
  --exclude '__pycache__'
  --exclude '*.pyc'
  --exclude '.git'
  --exclude '.pytest_cache'
  --exclude '*.egg-info'
)

echo "📦 Đang đồng bộ sạch toàn bộ mã nguồn sang ${REMOTE_WS} trên Pi..."

rsync "${RSYNC_OPTS[@]}" \
  "$WS_DIR/src/" \
  "${PI_USER}@${PI_IP}:${REMOTE_WS}/src/"

if [ -d "$WS_DIR/scripts" ]; then
  rsync "${RSYNC_OPTS[@]}" \
    "$WS_DIR/scripts/" \
    "${PI_USER}@${PI_IP}:${REMOTE_WS}/scripts/"
fi

rsync -avz \
  "$WS_DIR/aliases.sh" \
  "$WS_DIR/cyclonedds.xml" \
  "$WS_DIR/.gitignore" \
  "${PI_USER}@${PI_IP}:${REMOTE_WS}/"

echo "✅ Đã chuyển và làm sạch toàn bộ tệp tin sang ${REMOTE_WS} thành công!"

# 3. Đồng bộ thời gian hệ thống
echo "🕒 Đang đồng bộ giờ hệ thống giữa PC và Pi..."
PC_TIME=$(date -u +"%Y-%m-%d %H:%M:%S")
ssh -o StrictHostKeyChecking=no "${PI_USER}@${PI_IP}" "sudo date -u -s '$PC_TIME' >/dev/null 2>&1 || true"
echo "✅ Đã đồng bộ giờ hệ thống!"

# 4. Cấu hình ~/.bashrc trên Pi để hỗ trợ chuyển đổi độc lập & Thực hiện colcon build trong /home/bao/robot-vinh
echo "🔨 Đang cài đặt môi trường độc lập và biên dịch tại ${REMOTE_WS} trên Pi..."
ssh -o StrictHostKeyChecking=no "${PI_USER}@${PI_IP}" "bash -c '
# Gắn phím tắt kích hoạt nhanh dự án robot-vinh vào ~/.bashrc (không xóa cấu hình cũ của bạn cùng nhóm)
if ! grep -q \"# >>> ROBOT-VINH INDEPENDENT WORKSPACE <<<\" ~/.bashrc 2>/dev/null; then
cat >> ~/.bashrc << \"BRC\"

# >>> ROBOT-VINH INDEPENDENT WORKSPACE <<<
alias robot-vinh=\"cd ~/robot-vinh && source ~/robot-vinh/aliases.sh\"
alias vinh=\"cd ~/robot-vinh && source ~/robot-vinh/aliases.sh\"
alias robot-ban=\"cd ~/robot_ws && [ -f ~/robot_ws/install/setup.bash ] && source ~/robot_ws/install/setup.bash\"
if [ -f ~/robot-vinh/aliases.sh ]; then
    source ~/robot-vinh/aliases.sh
fi
# <<< ROBOT-VINH INDEPENDENT WORKSPACE <<<
BRC
fi

# Nạp aliases.sh của robot-vinh (tự động cô lập khỏi ~/robot_ws) và build
cd ${REMOTE_WS}
source ${REMOTE_WS}/aliases.sh
if [ ! -d \"${REMOTE_WS}/install/my_robot_interfaces\" ] || [ ! -d \"${REMOTE_WS}/install/sllidar_ros2\" ]; then
    echo \"📦 [Lần đầu khởi tạo robot-vinh] Đang build toàn bộ các package trong ${REMOTE_WS}...\"
    colcon build --symlink-install
else
    echo \"⚡ Đang build nhanh các package cập nhật trong ${REMOTE_WS}...\"
    colcon build --symlink-install --packages-select my_robot_controller my_robot_bringup my_robot_navigation my_robot_description my_sensor_test || colcon build --symlink-install
fi
'"

echo ""
echo "🎉 ============================================================"
echo "✅ ĐỒNG BỘ VÀ BIÊN DỊCH THÀNH CÔNG TẠI: ${REMOTE_WS}"
echo "   👉 Dự án của bạn chạy độc lập 100% tại ~/robot-vinh trên Pi!"
echo "   👉 Trên Pi chỉ cần gõ: pi-cnn (hoặc pi-cnn-continuous)"
echo "🎉 ============================================================"
