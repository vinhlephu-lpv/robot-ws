#!/usr/bin/env bash

# Tự động lấy thư mục gốc của Workspace (chạy đúng trên cả PC và Raspberry Pi)
WS_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Hàm nạp môi trường ROS 2 và Workspace
load_ws() {
    if [ -f "/opt/ros/jazzy/setup.bash" ]; then
        source /opt/ros/jazzy/setup.bash
    elif [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
    fi

    if [ -f "$WS_DIR/install/setup.bash" ]; then
        source "$WS_DIR/install/setup.bash"
    fi


    export ROS_DOMAIN_ID=0
    export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
    if [ -f "/opt/ros/jazzy/lib/librmw_cyclonedds_cpp.so" ] || [ -f "/opt/ros/humble/lib/librmw_cyclonedds_cpp.so" ] || [ -f "/usr/lib/librmw_cyclonedds_cpp.so" ]; then
        export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
        if [ -f "$WS_DIR/cyclonedds.xml" ]; then
            export CYCLONEDDS_URI="file://$WS_DIR/cyclonedds.xml"
            cp -f "$WS_DIR/cyclonedds.xml" "$HOME/.cyclonedds.xml" 2>/dev/null || true
        elif [ -f "$HOME/.cyclonedds.xml" ]; then
            export CYCLONEDDS_URI="file://$HOME/.cyclonedds.xml"
        fi
    else
        unset RMW_IMPLEMENTATION
    fi
}

# 1. Biên dịch & Cập nhật Workspace
reload_func() {
    [ -f "$HOME/.bashrc" ] && source "$HOME/.bashrc" 2>/dev/null || true
    [ -f "$WS_DIR/aliases.sh" ] && source "$WS_DIR/aliases.sh"
    echo "✅ Đã cập nhật nạp lại toàn bộ phím tắt mới nhất!"
}
alias reload="reload_func"
alias capnhat="reload_func"
alias update-cmd="reload_func"

# Hàm build thông minh (tự động nhận diện distro Jazzy / Humble & tự động nạp phím tắt)
build_func() {
    cd "$WS_DIR"
    if [ -f "/opt/ros/jazzy/setup.bash" ]; then
        source /opt/ros/jazzy/setup.bash
    elif [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
    fi
    colcon build --symlink-install "$@"
    local ret=$?
    if [ $ret -eq 0 ]; then
        if [ -f "$WS_DIR/install/setup.bash" ]; then
            source "$WS_DIR/install/setup.bash"
        fi
        if [ -f "$WS_DIR/aliases.sh" ]; then
            source "$WS_DIR/aliases.sh"
        fi
        echo "✅ [BUILD THÀNH CÔNG] Đã tự động cập nhật & nạp toàn bộ lệnh tắt mới nhất (rviz, quay-rviz, real-robot, wasd,...)!"
    else
        echo "❌ [BUILD THẤT BẠI] Vui lòng kiểm tra lại lỗi code ở trên."
        return $ret
    fi
}
alias build="build_func"
alias rb-build="build_func"

build_all_func() {
    cd "$WS_DIR"
    if [ -f "/opt/ros/jazzy/setup.bash" ]; then
        source /opt/ros/jazzy/setup.bash
    elif [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
    fi
    colcon build --symlink-install "$@"
    local ret=$?
    if [ $ret -eq 0 ]; then
        if [ -f "$WS_DIR/install/setup.bash" ]; then
            source "$WS_DIR/install/setup.bash"
        fi
        if [ -f "$WS_DIR/aliases.sh" ]; then
            source "$WS_DIR/aliases.sh"
        fi
        echo "✅ [BUILD-ALL THÀNH CÔNG] Đã tự động cập nhật & nạp toàn bộ lệnh tắt mới nhất!"
    else
        echo "❌ [BUILD-ALL THẤT BẠI] Vui lòng kiểm tra lại lỗi code ở trên."
        return $ret
    fi
}
alias build-all="build_all_func"

# Lệnh 1-Click đồng bộ nhanh từ GitHub về máy
git_sync_func() {
    cd "$WS_DIR"
    echo "📥 Đang kéo mã nguồn mới nhất từ GitHub..."
    local branch
    branch=$(git branch --show-current 2>/dev/null || echo "main")
    git pull origin "$branch" 2>/dev/null || git pull origin main 2>/dev/null || git pull || true
    echo "🔨 Đang biên dịch lại Workspace & cập nhật phím tắt..."
    build_func "$@"
}
alias git-sync="git_sync_func"
alias dongbo="git_sync_func"
alias sync-code="git_sync_func"
alias sync="git_sync_func"

# 2. Các lệnh chạy Mô phỏng (PC)
alias sim="load_ws && ros2 launch my_robot_simulation sim.launch.py"
alias gazebo="load_ws && ros2 launch my_robot_simulation sim.launch.py use_rviz:=false"
alias sim-only="gazebo"
alias ai="load_ws && ros2 launch my_robot_controller control.launch.py"
alias slam="load_ws && ros2 launch my_robot_slam slam.launch.py"
alias nav="load_ws && ros2 launch my_robot_navigation nav.launch.py"

# Bàn phím điều khiển robot (WASD & IJKL - Chạy liên tục ~15 Hz nuôi Watchdog ESP32)
alias teleop="load_ws && ros2 run my_robot_controller teleop_keyboard"
alias ros-teleop="teleop"
alias keyboard="teleop"
alias lai-xe="teleop"
alias wasd="teleop"
alias teleop-wasd="teleop"

# Mở RViz + Camera USB (Hiển thị mô hình xe 3D + Khung hình USB Webcam)
rviz_record_func() {
    load_ws
    mkdir -p "$WS_DIR/dataset/videos" "$WS_DIR/dataset/imgs"
    ros2 launch my_robot_bringup laptop_record.launch.py "$@"
}
alias rviz-record="rviz_record_func"
alias rviz-cam="rviz_record_func"

# Mở RViz hiển thị mô hình xe 3D, LiDAR và Camera (Tự động nhận cả Camera USB trực tiếp lẫn stream WiFi từ Pi)
rviz_view_func() {
    load_ws
    killall -q wifi_cam_receiver 2>/dev/null || true
    ros2 run my_robot_bringup wifi_cam_receiver &>/dev/null &
    local receiver_pid=$!
    sleep 0.5
    rviz2 -d "$WS_DIR/src/my_robot_description/rviz/display.rviz" "$@"
    kill $receiver_pid 2>/dev/null || true
}
alias rviz="rviz_view_func"
alias laptop-view="rviz_view_func"
alias rviz-only="load_ws && rviz2 -d \"$WS_DIR/src/my_robot_description/rviz/display.rviz\""
alias plot="load_ws && ros2 run my_robot_controller plot_response --mode telemetry"
alias plot-pp="load_ws && ros2 run my_robot_controller plot_response --mode pure_pursuit"
alias plot-smc="load_ws && ros2 run my_robot_controller plot_response --mode smc"
alias plot-gui="load_ws && ros2 run my_robot_controller plot_response --gui"
alias cancel="load_ws && ros2 run my_robot_navigation cancel_nav"

# Hàm lưu bản đồ nhanh
savemap() {
    load_ws
    local map_name="${1:-my_farm_map}"
    mkdir -p "$WS_DIR/maps"
    cd "$WS_DIR/maps" && ros2 run my_robot_slam map_saver -f "$map_name"
    echo "Đã lưu bản đồ '$map_name' vào thư mục: $WS_DIR/maps/"
}

# 3. Các lệnh kiểm tra cảm biến riêng lẻ (1-Click Test)
alias test-lidar="load_ws && ros2 launch my_sensor_test test_lidar.launch.py"
alias test-cam="load_ws && bash \"$WS_DIR/src/my_sensor_test/scripts/run_test_camera.sh\""
alias test-camera="test-cam"
alias test-imu="load_ws && bash \"$WS_DIR/src/my_sensor_test/scripts/run_test_imu.sh\""
alias test-gps="load_ws && ros2 run my_robot_controller gps_driver --ros-args -p serial_port:=/dev/ttyAMA0 -p baudrate:=38400"
alias run-encoder="load_ws && ros2 run encoder_odom encoder_node --ros-args -p serial_port:=/dev/ttyACM0"
alias encoder="run-encoder"
alias test-esp32="load_ws && ros2 run my_robot_bringup esp32_bridge --ros-args -p serial_port:=/dev/esp32"

# Lệnh nạp Firmware ESP32-S3 tự động từ Raspberry Pi
nap_esp32_func() {
    local port="${1:-}"
    if [ -z "$port" ]; then
        if [ -e "/dev/esp32" ]; then port="/dev/esp32"
        elif [ -e "/dev/ttyACM0" ]; then port="/dev/ttyACM0"
        elif [ -e "/dev/ttyUSB0" ]; then port="/dev/ttyUSB0"
        else port="/dev/esp32"
        fi
    fi
    echo "⚡ [ESP32] Đang biên dịch và nạp firmware code0409 vào cổng $port..."
    arduino-cli compile --fqbn esp32:esp32:esp32s3 "$WS_DIR/src/esp32_source/code0409/" -u -p "$port"
}
alias nap-esp32="nap_esp32_func"
alias flash-esp32="nap_esp32_func"
alias test-all="load_ws && ros2 launch my_sensor_test test_all_sensors.launch.py"
alias test-slam="load_ws && bash \"$WS_DIR/src/my_sensor_test/scripts/run_test_slam.sh\""

# 4. Các lệnh chạy trên Robot Thật (Raspberry Pi)
alias real-robot="load_ws && ros2 launch my_robot_bringup real_robot.launch.py"
alias real-slam="load_ws && ros2 launch my_robot_bringup real_slam.launch.py"

# Lệnh TỰ HÀNH XE THẬT THEO BẢN ĐỒ ĐÃ LƯU (Tự động nhận map mới nhất hoặc chỉ định tên map)
unalias real-nav 2>/dev/null || true
real_nav_func() {
    load_ws
    local map_arg="${1:-}"
    if [ -z "$map_arg" ]; then
        local latest_map=$(ls -t "$WS_DIR/maps/"*.yaml 2>/dev/null | head -n 1)
        if [ -n "$latest_map" ]; then
            echo "🗺️ Tự động nạp bản đồ mới nhất: $latest_map"
            ros2 launch my_robot_bringup real_nav.launch.py map:="$latest_map"
        else
            echo "❌ Chưa tìm thấy bản đồ nào trong thư mục maps/! Hãy chạy real-slam trước rồi dùng savemap."
        fi
    elif [[ "$map_arg" == map:=* ]]; then
        ros2 launch my_robot_bringup real_nav.launch.py "$@"
    else
        if [ -f "$WS_DIR/maps/$map_arg.yaml" ]; then
            echo "🗺️ Nạp bản đồ: $WS_DIR/maps/$map_arg.yaml"
            ros2 launch my_robot_bringup real_nav.launch.py map:="$WS_DIR/maps/$map_arg.yaml"
        elif [ -f "$map_arg" ]; then
            echo "🗺️ Nạp bản đồ: $map_arg"
            ros2 launch my_robot_bringup real_nav.launch.py map:="$map_arg"
        else
            echo "❌ Không tìm thấy bản đồ '$map_arg' trong $WS_DIR/maps/"
        fi
    fi
}
# Lệnh TỰ HÀNH XE THẬT BÁM LUỐNG BẰNG AI CNN (Crop Row Following)
alias real-cnn="load_ws && ros2 launch my_robot_bringup real_robot.launch.py enable_cnn:=true"
alias auto-cnn="real-cnn"
alias cnn-auto="real-cnn"

# Lệnh KIỂM TRA CHẨN ĐOÁN TOÀN DIỆN CHUỖI AI CNN
alias check-cnn="load_ws && python3 \"$WS_DIR/scripts/verify_cnn_pipeline.py\""

# Lệnh CHẠY THỬ SUY LUẬN AI CNN TRÊN ẢNH (Mô phỏng 100% xe thật bám luống)
alias test-img="load_ws && python3 \"$WS_DIR/scripts/test_inference_images.py\""
alias cnn-img="test-img"

alias pc-nav="load_ws && ros2 launch my_robot_bringup pc_nav.launch.py"
alias nav-slam="load_ws && ros2 launch my_robot_bringup pc_nav.launch.py"
alias gps-nav="load_ws && ros2 launch my_robot_navigation gps_nav.launch.py"
alias nav-gps="gps-nav"

# Lệnh kích hoạt xe THẬT CÓ QUAY VIDEO THÔ (100% Raw, không hiện gì trên màn hình)
real-record() {
    load_ws
    local vname="${1:-}"
    if [ -n "$vname" ]; then
        ros2 launch my_robot_bringup real_robot.launch.py record:=true record_name:="$vname"
    else
        ros2 launch my_robot_bringup real_robot.launch.py record:=true
    fi
}

# Lệnh MỞ RVIZ + XEM XE 3D + WEBCAM USB DVD20 (1080p Full HD @ 60 FPS) + QUAY VIDEO TÁCH FRAME DATASET RIÊNG (Chạy trên Laptop)
rviz-record() {
    load_ws
    local vname="${1:-}"
    if [ -n "$vname" ]; then
        shift
        mkdir -p "$WS_DIR/dataset/videos" "$WS_DIR/dataset/imgs"
        ros2 launch my_robot_bringup laptop_record.launch.py name:="$vname" "$@"
    else
        mkdir -p "$WS_DIR/dataset/videos" "$WS_DIR/dataset/imgs"
        ros2 launch my_robot_bringup laptop_record.launch.py "$@"
    fi
}
alias record-rviz="rviz-record"
alias laptop-record="rviz-record"
alias quay-rviz="rviz-record"
alias quay="rviz-record"
alias quay-video="rviz-record"

# Lệnh tải video từ Pi xuống Laptop (Chạy trên Laptop)
get-video() {
    local pi_ip="${1:-}"
    if [ -z "$pi_ip" ]; then
        for ip in 10.10.178.200 10.10.177.141; do
            if ping -c 1 -W 1 "$ip" &>/dev/null; then
                pi_ip="$ip"
                break
            fi
        done
    fi
    if [ -z "$pi_ip" ]; then
        pi_ip="10.10.178.200"
    fi
    mkdir -p "$WS_DIR/dataset"
    echo "📥 Đang kéo video thô từ Pi ($pi_ip) về $WS_DIR/dataset/ ..."
    rsync -avP --include='*.mp4' "bao@$pi_ip:~/robot-ws/recordings/" "$WS_DIR/dataset/" 2>/dev/null || \
    rsync -avP --include='*.mp4' "bao@$pi_ip:~/robot_ws/recordings/" "$WS_DIR/dataset/" 2>/dev/null || \
    scp "bao@$pi_ip:~/robot-ws/recordings/*.mp4" "$WS_DIR/dataset/" 2>/dev/null || \
    scp "bao@$pi_ip:~/robot_ws/recordings/*.mp4" "$WS_DIR/dataset/"
    echo "✅ File video đã được lưu tại: $WS_DIR/dataset/"
}
alias get-videos="get-video"
alias extract-dataset="python3 \"$WS_DIR/scripts/extract_dataset.py\""
alias rename-dataset="python3 \"$WS_DIR/scripts/rename_dataset.py\""

# Lệnh xóa toàn bộ video đã quay (chạy được trên cả Pi và Laptop)
clean-video() {
    echo "🗑️ Đang dọn dẹp các video trong thư mục recordings và dataset..."
    rm -f ~/robot-ws/recordings/*.mp4 ~/robot_ws/recordings/*.mp4 "$WS_DIR/recordings/"*.mp4 2>/dev/null || true
    echo "✅ Đã dọn dẹp video hoàn tất!"
}
alias clear-video="clean-video"
alias del-video="clean-video"

# Lệnh xem video nhanh bằng Firefox
play-video() {
    local f="${1:-}"
    if [ -z "$f" ]; then
        f=$(ls -t "$WS_DIR/dataset/videos/"*.mp4 "$WS_DIR/dataset/"*.mp4 2>/dev/null | head -n 1)
    fi
    if [ -n "$f" ] && [ -f "$f" ]; then
        echo "🎬 Đang mở video: $f"
        firefox "$f" &>/dev/null &
    else
        echo "❌ Không tìm thấy file video nào trong $WS_DIR/dataset/videos/"
    fi
}
# =====================================================
# THEO DÕI BỘ LỌC CẢM BIẾN & SENSOR FUSION (IMU, ENCODER, EKF)
# =====================================================

# 1. Xem dữ liệu IMU qua bộ lọc Madgwick (/imu/data)
xem-imu() {
    echo "🧭 [IMU Madgwick] Đang theo dõi hướng và độ nghiêng cảm biến... (Ctrl+C để dừng)"
    python3 -c '
import rclpy, math
from sensor_msgs.msg import Imu

def cb(msg):
    q = msg.orientation
    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

    sinp = 2 * (q.w * q.y - q.z * q.x)
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, sinp))))

    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

    wz = msg.angular_velocity.z
    print(f"\r🧭 [IMU Madgwick] Hướng xe (Yaw): {yaw:+6.1f}° | Nghiêng ngang: {roll:+5.1f}° | Dốc trước/sau: {pitch:+5.1f}° | Tốc độ xoay: {wz:+5.2f}rad/s", end="", flush=True)

rclpy.init()
node = rclpy.create_node("xem_imu_cli")
node.create_subscription(Imu, "/imu/data", cb, 10)
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    print()
'
}
alias show-imu="xem-imu"
alias check-imu="xem-imu"

# 2. Xem dữ liệu Encoder bánh xe từ ESP32 (/odom/raw)
xem-encoder() {
    echo "🚗 [Encoder ESP32] Đang theo dõi vận tốc và quãng đường lăn bánh... (Ctrl+C để dừng)"
    python3 -c '
import rclpy, math
from nav_msgs.msg import Odometry

def cb(msg):
    vx = msg.twist.twist.linear.x
    wz = msg.twist.twist.angular.z
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    print(f"\r🚗 [Encoder ESP32] Vận tốc lăn: {vx:+5.2f}m/s | Bẻ lái bánh: {wz:+5.2f}rad/s | Quãng đường đã lăn: ({x:+5.2f}, {y:+5.2f})m", end="", flush=True)

rclpy.init()
node = rclpy.create_node("xem_enc_cli")
node.create_subscription(Odometry, "/odom/raw", cb, 10)
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    print()
'
}
alias xem-enc="xem-encoder"
alias show-enc="xem-encoder"
alias show-encoder="xem-encoder"

# 3. Xem kết quả dung hợp EKF cuối cùng (/odometry/filtered)
xem-ekf() {
    echo "⭐ [EKF Fusion] Đang theo dõi tọa độ & hướng dung hợp tổng hợp... (Ctrl+C để dừng)"
    python3 -c '
import rclpy, math
from nav_msgs.msg import Odometry

def cb(msg):
    q = msg.pose.pose.orientation
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    vx = msg.twist.twist.linear.x
    wz = msg.twist.twist.angular.z
    print(f"\r⭐ [EKF Tổng Hợp] Tọa độ: (X:{x:+5.2f}m, Y:{y:+5.2f}m) | Hướng chuẩn: {yaw:+6.1f}° | Tốc độ thực: {vx:+5.2f}m/s | Bẻ lái: {wz:+5.2f}rad/s", end="", flush=True)

rclpy.init()
node = rclpy.create_node("xem_ekf_cli")
node.create_subscription(Odometry, "/odometry/filtered", cb, 10)
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    print()
'
}
alias show-ekf="xem-ekf"
alias xem-odom="xem-ekf"
alias show-odom="xem-ekf"
alias check-ekf="load_ws && python3 \"$WS_DIR/src/my_sensor_test/scripts/check_ekf\""
alias xem-fusion="check-ekf"
alias test-ekf="check-ekf"
alias check-imu="load_ws && python3 \"$WS_DIR/src/my_sensor_test/scripts/check_imu\""
alias dual-ekf="load_ws && ros2 launch my_robot_bringup dual_ekf.launch.py"
alias ekf-gps="load_ws && ros2 launch my_robot_bringup dual_ekf.launch.py enable_gps:=true"

# =====================================================
# BẢNG TRA CỨU LỆNH TẮT NHANH (ros-help)
# =====================================================
ros_help_func() {
cat << 'EOF'
================================================================================
  🤖 BẢNG TRA CỨU TOÀN BỘ LỆNH TẮT NHANH (ROS 2 ROBOT CHEAT SHEET)
================================================================================

📊 [THEO DÕI BỘ LỌC & SENSOR FUSION] (Mới nhất)
  xem-imu (show-imu) : Xem IMU qua Bộ lọc Madgwick (Roll, Pitch, Yaw theo Độ °)
  xem-enc (xem-encoder): Xem Odometry bánh xe thô từ ESP32 (Vận tốc m/s, Xoay bánh)
  xem-ekf (xem-odom) : Xem KẾT QUẢ DUNG HỢP EKF CUỐI CÙNG (Tọa độ X/Y, Hướng Yaw, Tốc độ)

💻 [TRÊN LAPTOP] (Màn hình quan sát, Lái xe & Xử lý Dataset)
  quay-rviz [tên]    : Mở RViz + Quay video Full HD 1080p 60FPS + Tách Dataset ảnh
                       (Tên khác: rviz-record, laptop-record)
  laptop-view        : Mở RViz2 nhận luồng Camera nén từ Pi qua Wi-Fi (mượt, không lag)
  teleop (lai-xe)    : Bàn phím lái xe chuẩn gốc ROS 2 (i=tiến, ,=lùi, j/l=rẽ, k=dừng)
  get-video          : Tự động kéo video MP4 mới quay từ Pi về máy tính
  play-video (xem)   : Xem ngay video vừa quay bằng trình duyệt Firefox
  clean-video        : Dọn dẹp các video cũ giải phóng ổ đĩa
  extract-dataset <f>: Cắt video thành bộ ảnh sạch (JPG) để gán nhãn train CNN
  rviz               : Mở giao diện RViz2 đồ họa thuần túy
  rviz-only          : Mở giao diện RViz2 cấu hình chuẩn cho xe thật
  pc-nav (nav-slam)  : Bật Nav2 trên Laptop kết hợp với SLAM trực tiếp từ Pi
  cancel             : Hủy mục tiêu dẫn đường Nav2

🍓 [TRÊN RASPBERRY PI] (Khởi động phần cứng xe & Quay video)
  real-robot         : BẬT XE THẬT (Tự động chạy Madgwick + EKF chuẩn xác)
  real-record [tên]  : BẬT XE THẬT + QUAY VIDEO THÔ (100% Raw, lưu MP4 vào Pi)
  real-cnn           : BẬT XE THẬT TỰ HÀNH BÁM LUỐNG BẰNG AI CNN (1-Click)
  real-slam          : Bật Xe Thật + SLAM Toolbox vẽ bản đồ
  real-nav [tên_map] : Bật Xe Thật + Nav2 tự né vật cản (Tự động nạp map mới nhất)
  savemap <tên_map>  : Lưu bản đồ SLAM vừa quét xong vào thư mục maps/

🔍 [KIỂM TRA CẢM BIẾN & AI] (1-Click Test trên Pi / Laptop)
  check-cnn (test-cnn): Kiểm tra chẩn đoán toàn diện chuỗi AI CNN (512x512, góc lái, ESP32)
  check-ekf (test-ekf): Chẩn đoán bảng số liệu trực tiếp EKF (Wheel, IMU, GPS, Độ lệch)
  check-imu          : Kiểm tra lọc rung & bù trôi tĩnh ZUPT của cảm biến IMU
  test-cam           : Kiểm tra hình ảnh Webcam DVD20 1080p 60FPS (/dev/video0)
  test-lidar         : Kiểm tra tia quét mắt LiDAR RPLIDAR C1 (/dev/ttyUSB0)
  test-imu           : Kiểm tra cảm biến IMU 9 trục ICM-20948 (I2C Pin 3, 5)
  test-esp32         : Kiểm tra kết nối mạch điều khiển ESP32 Bridge
  nap-esp32          : Nạp firmware code0409.ino vào ESP32-S3 (/dev/esp32)
  run-encoder        : Chạy Node tính toán Odometry từ xung RAW 4 bánh
  test-gps           : Kiểm tra module GPS UART GPIO (/dev/ttyAMA0)
  gps-nav            : Tự hành theo cọc tiêu tọa độ GPS ngoài trời
  test-all           : Kiểm tra toàn bộ cảm biến cùng lúc trên RViz

🎮 [MÔ PHỎNG & ĐỒ THỊ] (Chạy trên PC / Laptop)
  sim                : Mở thế giới ảo Gazebo + RViz + Xe mô phỏng
  gazebo (sim-only)  : Mở riêng Gazebo (không mở RViz)
  ai                 : Bật thuật toán AI CNN bám luống trong mô phỏng
  slam               : Bật SLAM vẽ bản đồ ảo
  nav                : Bật Nav2 dẫn đường trong mô phỏng
  plot               : Vẽ biểu đồ quỹ đạo & cảm biến Telemetry
  plot-pp            : Vẽ phân tích đáp ứng Pure Pursuit
  plot-smc           : Vẽ phân tích bộ điều khiển trượt SMC
  plot-gui           : Mở giao diện thanh trượt tinh chỉnh Live Tuning

⚙️ [BIÊN DỊCH & CẬP NHẬT]
  reload (capnhat)   : Nạp lại toàn bộ lệnh mới nhất sau khi git pull
  git-sync (dongbo)  : Kéo code mới nhất từ GitHub + Build lại tự động 1-Click
  build              : Build nhanh workspace (colcon build)
  build-all          : Build toàn bộ tất cả package
  ros-help           : Xem lại bảng hướng dẫn này bất cứ lúc nào
================================================================================
EOF
}
unalias ros-help 2>/dev/null || true
unalias help-robot 2>/dev/null || true
unalias robot-help 2>/dev/null || true
alias ros-help="ros_help_func"
alias robot-help="ros_help_func"
alias help-robot="ros_help_func"

# Lệnh DỪNG KHẨN CẤP & DỌN SẠCH TIẾN TRÌNH ROBOT (Dùng khi Ctrl+C hoặc xe bị nhiễu)
stop_robot_func() {
    echo "🛑 Đang gửi lệnh phanh khẩn cấp & dừng toàn bộ động cơ..."
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" 2>/dev/null || true
    echo "🧹 Đang dọn sạch các tiến trình ROS 2 còn sót lại..."
    killall -9 rplidar_node sllidar_node esp32_bridge imu_driver costmap_node async_slam_toolbox_node 2>/dev/null || true
    echo "✅ Toàn bộ hệ thống Robot đã dừng an toàn và giải phóng cổng Serial!"
}
alias stop-robot="stop_robot_func"
alias stop="stop_robot_func"
alias dung="stop_robot_func"

# Lệnh HỦY DẪN ĐƯỜNG NAV2 (Hủy mục tiêu điểm đến phím G & phanh dừng xe)
cancel_nav_func() {
    echo "🛑 Đang hủy mục tiêu dẫn đường Nav2 & dừng xe..."
    ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal "{goal_info: {goal_id: {uuid: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}, stamp: {sec: 0, nanosec: 0}}}" 2>/dev/null || true
    ros2 service call /navigate_through_poses/_action/cancel_goal action_msgs/srv/CancelGoal "{goal_info: {goal_id: {uuid: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}, stamp: {sec: 0, nanosec: 0}}}" 2>/dev/null || true
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" 2>/dev/null || true
    echo "✅ Đã hủy lệnh Nav2! Xe đã dừng an toàn."
}
alias huy="cancel_nav_func"
alias huy-nav="cancel_nav_func"
alias stop-nav="cancel_nav_func"
alias cancel-nav="cancel_nav_func"
