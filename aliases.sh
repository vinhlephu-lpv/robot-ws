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

    if [ -f "$HOME/astra_ws/install/setup.bash" ]; then
        source "$HOME/astra_ws/install/setup.bash"
    elif [ -f "/tmp/astra_ws/install/setup.bash" ]; then
        source "/tmp/astra_ws/install/setup.bash"
    fi

    export ROS_DOMAIN_ID=0
    export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    if [ -f "$WS_DIR/cyclonedds.xml" ]; then
        export CYCLONEDDS_URI="file://$WS_DIR/cyclonedds.xml"
        cp -f "$WS_DIR/cyclonedds.xml" "$HOME/.cyclonedds.xml" 2>/dev/null || true
    elif [ -f "$HOME/.cyclonedds.xml" ]; then
        export CYCLONEDDS_URI="file://$HOME/.cyclonedds.xml"
    fi
}

# 1. Biên dịch & Cập nhật Workspace
alias reload="source ~/.bashrc && echo '✅ Đã cập nhật nạp lại toàn bộ lệnh mới nhất!'"
alias update-cmd="reload"
alias capnhat="reload"

# Hàm build thông minh (tự động nhận diện distro Jazzy / Humble)
build_func() {
    cd "$WS_DIR"
    if [ -f "/opt/ros/jazzy/setup.bash" ]; then
        source /opt/ros/jazzy/setup.bash
    elif [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
    fi
    colcon build --symlink-install --packages-ignore astra_camera astra_camera_msgs "$@"
    if [ -f "$WS_DIR/install/setup.bash" ]; then
        source "$WS_DIR/install/setup.bash"
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
    if [ -f "$WS_DIR/install/setup.bash" ]; then
        source "$WS_DIR/install/setup.bash"
    fi
}
alias build-all="build_all_func"

# Lệnh 1-Click đồng bộ nhanh từ GitHub về máy
git_sync_func() {
    cd "$WS_DIR"
    echo "📥 Đang kéo mã nguồn mới nhất từ GitHub..."
    git pull origin main
    echo "🔨 Đang biên dịch lại Workspace..."
    build_func
    echo "✅ Đã đồng bộ và nạp lại toàn bộ lệnh thành công!"
}
alias git-sync="git_sync_func"
alias dongbo="git_sync_func"
alias sync-code="git_sync_func"

# 2. Các lệnh chạy Mô phỏng (PC)
alias sim="load_ws && ros2 launch my_robot_simulation sim.launch.py"
alias gazebo="load_ws && ros2 launch my_robot_simulation sim.launch.py use_rviz:=false"
alias sim-only="gazebo"
alias ai="load_ws && ros2 launch my_robot_controller control.launch.py"
alias slam="load_ws && ros2 launch my_robot_slam slam.launch.py"
alias nav="load_ws && ros2 launch my_robot_navigation nav.launch.py"
alias teleop="load_ws && ros2 run my_robot_controller teleop_wasd"
alias wasd="teleop"

# Mở RViz + Camera USB (Hiển thị mô hình xe 3D + Khung hình Webcam DVD20)
rviz_view_func() {
    load_ws
    mkdir -p "$WS_DIR/dataset/videos" "$WS_DIR/dataset/imgs"
    ros2 launch my_robot_bringup laptop_record.launch.py "$@"
}
alias rviz="rviz_view_func"
alias rviz-cam="rviz_view_func"
alias rviz-only="load_ws && rviz2 -d \"$WS_DIR/src/my_robot_description/rviz/display.rviz\""

# Mở RViz nhận stream WiFi nhẹ từ Pi
laptop_view_func() {
    load_ws
    killall -q wifi_cam_receiver 2>/dev/null || true
    ros2 run my_robot_bringup wifi_cam_receiver &
    local receiver_pid=$!
    sleep 1
    rviz2 -d "$WS_DIR/src/my_robot_description/rviz/display.rviz"
    kill $receiver_pid 2>/dev/null || true
}
alias laptop-view="laptop_view_func"
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
alias test-esp32="load_ws && ros2 run my_robot_bringup esp32_bridge --ros-args -p serial_port:=/dev/esp32"
alias test-all="load_ws && ros2 launch my_sensor_test test_all_sensors.launch.py"
alias test-slam="load_ws && bash \"$WS_DIR/src/my_sensor_test/scripts/run_test_slam.sh\""

# 4. Các lệnh chạy trên Robot Thật (Raspberry Pi)
alias real-robot="load_ws && ros2 launch my_robot_bringup real_robot.launch.py"
alias real-slam="load_ws && ros2 launch my_robot_bringup real_slam.launch.py"
alias real-nav="load_ws && ros2 launch my_robot_bringup real_nav.launch.py"

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
    killall -9 astra_camera_node 2>/dev/null || true
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
alias getvideo="get-video"
alias extract-dataset="python3 \"$WS_DIR/scripts/extract_dataset.py\""

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
alias xem-video="play-video"
alias xem="play-video"

# =====================================================
# BẢNG TRA CỨU LỆNH TẮT NHANH (ros-help)
# =====================================================
ros_help_func() {
cat << 'EOF'
================================================================================
  🤖 BẢNG TRA CỨU TOÀN BỘ LỆNH TẮT NHANH (ROS 2 ROBOT CHEAT SHEET)
================================================================================

💻 [TRÊN LAPTOP] (Màn hình quan sát, Lái xe & Xử lý Dataset)
  quay-rviz [tên]    : Mở RViz + Quay video Full HD 1080p 60FPS + Tách Dataset ảnh
                       (Tên khác: rviz-record, laptop-record)
  laptop-view        : Mở RViz2 nhận luồng Camera nén từ Pi qua Wi-Fi (mượt, không lag)
  wasd (teleop)      : Bàn phím lái xe (W=tiến, S=lùi, A/D=rẽ, Space=phanh dừng)
  get-video          : Tự động kéo video MP4 mới quay từ Pi về máy tính
  play-video (xem)   : Xem ngay video vừa quay bằng trình duyệt Firefox
  clean-video        : Dọn dẹp các video cũ giải phóng ổ đĩa
  extract-dataset <f>: Cắt video thành bộ ảnh sạch (JPG) để gán nhãn train CNN
  rviz               : Mở giao diện RViz2 đồ họa thuần túy
  cancel             : Hủy mục tiêu dẫn đường Nav2

🍓 [TRÊN RASPBERRY PI] (Khởi động phần cứng xe & Quay video)
  real-robot         : BẬT XE THẬT (Chế độ bình thường: chỉ xem, KHÔNG lưu)
  real-record [tên]  : BẬT XE THẬT + QUAY VIDEO THÔ (100% Raw, lưu MP4 vào Pi)
  real-slam          : Bật Xe Thật + SLAM Toolbox vẽ bản đồ
  real-nav           : Bật Xe Thật + Nav2 dẫn đường tự né vật cản
                       (Ví dụ: real-nav map:=/path/to/map.yaml)
  savemap <tên_map>  : Lưu bản đồ SLAM vừa quét xong vào thư mục maps/

🔍 [KIỂM TRA CẢM BIẾN] (1-Click Test trên Pi / Laptop)
  test-cam           : Kiểm tra hình ảnh Webcam DVD20 1080p 60FPS (/dev/video0)
  test-lidar         : Kiểm tra tia quét mắt LiDAR RPLIDAR C1 (/dev/ttyUSB0)
  test-imu           : Kiểm tra cảm biến IMU 9 trục ICM-20948 (I2C Pin 3, 5)
  test-esp32         : Kiểm tra kết nối mạch điều khiển ESP32 Bridge
  test-gps           : Kiểm tra module GPS UART GPIO (/dev/ttyAMA0)
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
ros-help() {
    ros_help_func "$@"
}
alias robot-help="ros-help"
alias help-robot="ros-help"
