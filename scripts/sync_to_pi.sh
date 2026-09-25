#!/usr/bin/env bash
# ==============================================================================
# Script: sync_to_pi.sh
# Đồng bộ mã nguồn từ PC sang Raspberry Pi và tự động build lại trên Pi
# Cách dùng:
#   ./scripts/sync_to_pi.sh [IP_CUA_PI]
# Ví dụ:
#   ./scripts/sync_to_pi.sh
#   ./scripts/sync_to_pi.sh 10.10.178.200
#   ./scripts/sync_to_pi.sh 192.168.43.10
# ==============================================================================

set -e

SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WS_DIR="$(dirname "$SCRIPT_DIR")"

PI_USER="bao"

# Tự động dò IP của Pi thông minh:
# Nếu IP truyền vào ping được thì dùng, nếu chết/không truyền thì tự động bắt IP đang online
TARGET_IP="${1:-}"
if [ -n "$TARGET_IP" ] && ping -c 1 -W 1 "$TARGET_IP" >/dev/null 2>&1; then
    PI_IP="$TARGET_IP"
else
    PI_CANDIDATES=("10.42.0.236" "${PI_IP:-}" "$TARGET_IP" "10.10.178.200" "bao-desktop.local")
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
echo "🚀 ĐỒNG BỘ MÃ NGUỒN PC ➔ RASPBERRY PI (MIRROR CLEAN CLONE)"
echo "   IP Pi: $PI_IP (User: $PI_USER)"
echo "   Thư mục PC: $WS_DIR"
echo "   Chế độ: Đồng bộ sạch 100% (--delete tự xóa file rác cũ)"
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
    echo "💡 Gợi ý: Nếu biết IP mới của Pi, hãy chạy: ./scripts/sync_to_pi.sh <IP_MOI>"
    exit 1
fi
echo "✅ Kết nối mạng OK!"

# 2. Cấu hình rsync đồng bộ sạch 100% (Xóa file thừa, loại trừ rác python)
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

echo "📦 Đang đồng bộ sạch (Mirror Clone) các package sang Pi..."

rsync "${RSYNC_OPTS[@]}" \
  "$WS_DIR/src/my_robot_controller/" \
  "${PI_USER}@${PI_IP}:/home/${PI_USER}/robot_ws/src/my_robot_controller/"

rsync "${RSYNC_OPTS[@]}" \
  "$WS_DIR/src/my_robot_bringup/" \
  "${PI_USER}@${PI_IP}:/home/${PI_USER}/robot_ws/src/my_robot_bringup/"

rsync "${RSYNC_OPTS[@]}" \
  "$WS_DIR/src/my_robot_navigation/" \
  "${PI_USER}@${PI_IP}:/home/${PI_USER}/robot_ws/src/my_robot_navigation/"

if [ -d "$WS_DIR/src/my_sensor_test" ]; then
  rsync "${RSYNC_OPTS[@]}" \
    "$WS_DIR/src/my_sensor_test/" \
    "${PI_USER}@${PI_IP}:/home/${PI_USER}/robot_ws/src/my_sensor_test/"
fi

if [ -d "$WS_DIR/scripts" ]; then
  rsync "${RSYNC_OPTS[@]}" \
    "$WS_DIR/scripts/" \
    "${PI_USER}@${PI_IP}:/home/${PI_USER}/robot_ws/scripts/"
fi

rsync -avz \
  "$WS_DIR/aliases.sh" \
  "$WS_DIR/cyclonedds.xml" \
  "${PI_USER}@${PI_IP}:/home/${PI_USER}/robot_ws/"

echo "✅ Đã chuyển và làm sạch toàn bộ tệp tin sang Pi thành công!"

# 3. Đồng bộ thời gian hệ thống
echo "🕒 Đang đồng bộ giờ hệ thống giữa PC và Pi..."
PC_TIME=$(date -u +"%Y-%m-%d %H:%M:%S")
ssh -o StrictHostKeyChecking=no "${PI_USER}@${PI_IP}" "sudo date -u -s '$PC_TIME' >/dev/null 2>&1 || true"
echo "✅ Đã đồng bộ giờ hệ thống!"

# 4. Thực hiện colcon build trên Pi
ssh -o StrictHostKeyChecking=no "${PI_USER}@${PI_IP}" "bash -c 'source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null; cd /home/${PI_USER}/robot_ws && (colcon build --symlink-install --packages-select my_robot_controller my_robot_bringup my_robot_navigation my_sensor_test || (rm -rf build/ && colcon build --symlink-install --packages-select my_robot_controller my_robot_bringup my_robot_navigation my_sensor_test))'"

echo ""
echo "🎉 ============================================================"
echo "✅ ĐỒNG BỘ VÀ BIÊN DỊCH THÀNH CÔNG TRÊN RASPBERRY PI!"
echo "   Code trên Pi hiện tại đã giống PC 100% (sạch hoàn toàn)!"
echo "🎉 ============================================================"
