#!/usr/bin/env bash

# Tự động lấy thư mục gốc của Workspace (chạy đúng trên cả PC và Raspberry Pi)
WS_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Hàm nạp môi trường ROS 2 và Workspace
load_ws() {
    source /opt/ros/jazzy/setup.bash
    if [ -f "$WS_DIR/install/setup.bash" ]; then
        source "$WS_DIR/install/setup.bash"
    fi
    export ROS_DOMAIN_ID=0
    export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    if [ -f "$HOME/.cyclonedds.xml" ]; then
        export CYCLONEDDS_URI="file://$HOME/.cyclonedds.xml"
    elif [ -f "$WS_DIR/cyclonedds.xml" ]; then
        export CYCLONEDDS_URI="file://$WS_DIR/cyclonedds.xml"
    fi
    if [ -d "$WS_DIR/install/astra_camera/lib/OpenNI2/Drivers" ]; then
        export OPENNI2_REDIST="$WS_DIR/install/astra_camera/lib/OpenNI2/Drivers"
        export OPENNI2_DRIVERS_PATH="$WS_DIR/install/astra_camera/lib/OpenNI2/Drivers"
    elif [ -d "$WS_DIR/src/astra_camera/openni2_redist/arm64/OpenNI2/Drivers" ]; then
        export OPENNI2_REDIST="$WS_DIR/src/astra_camera/openni2_redist/arm64/OpenNI2/Drivers"
        export OPENNI2_DRIVERS_PATH="$WS_DIR/src/astra_camera/openni2_redist/arm64/OpenNI2/Drivers"
    fi
}

# 1. Biên dịch Workspace
alias build="cd \"$WS_DIR\" && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-ignore astra_camera astra_camera_msgs && source install/setup.bash"
alias build-all="cd \"$WS_DIR\" && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install && source install/setup.bash"
alias rb-build="build"

# 2. Các lệnh chạy Mô phỏng (PC)
alias sim="load_ws && ros2 launch my_robot_simulation sim.launch.py"
alias gazebo="load_ws && ros2 launch my_robot_simulation sim.launch.py use_rviz:=false"
alias sim-only="gazebo"
alias ai="load_ws && ros2 launch my_robot_controller control.launch.py"
alias slam="load_ws && ros2 launch my_robot_slam slam.launch.py"
alias nav="load_ws && ros2 launch my_robot_navigation nav.launch.py"
alias teleop="load_ws && ros2 run my_robot_controller teleop_wasd"
alias wasd="teleop"
alias rviz="load_ws && rviz2 -d \"$WS_DIR/src/my_robot_description/rviz/display.rviz\""
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

# 3. Các lệnh chạy trên Robot Thật (Raspberry Pi)
alias test-lidar="load_ws && ros2 launch my_sensor_test test_lidar.launch.py"
alias test-cam="load_ws && ros2 launch astra_camera astra.launch.xml enable_color:=true enable_depth:=true color_width:=640 color_height:=480 depth_width:=640 depth_height:=480 enable_point_cloud:=false"
alias test-gps="load_ws && ros2 run my_robot_controller gps_driver --ros-args -p serial_port:=/dev/ttyAMA0 -p baudrate:=38400"
alias test-all="load_ws && ros2 launch my_sensor_test test_all_sensors.launch.py"
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

# =====================================================
# BẢNG TRA CỨU LỆNH TẮT NHANH (ros-help)
# =====================================================
ros_help_func() {
cat << 'EOF'
================================================================================
  🤖 BẢNG TRA CỨU TOÀN BỘ LỆNH TẮT NHANH (ROS 2 ROBOT CHEAT SHEET)
================================================================================

💻 [TRÊN LAPTOP] (Màn hình quan sát, Lái xe & Xử lý Dataset)
  laptop-view        : Mở RViz2 + tự nhận Camera nén từ Pi (mượt, không nghẽn)
  wasd               : Bàn phím lái xe (W=tiến, S=lùi, A/D=rẽ, Space=dừng)
  get-video          : Tự động kéo video MP4 mới quay từ Pi về robot_ws/dataset/
  extract-dataset <f>: Cắt video thành bộ ảnh sạch (JPG) để gán nhãn train CNN
  rviz               : Mở RViz2 đồ họa thuần túy
  cancel             : Hủy mục tiêu dẫn đường Nav2

🍓 [TRÊN RASPBERRY PI] (Khởi động phần cứng xe & Quay video)
  real-robot         : BẬT XE THẬT (Chế độ bình thường: chỉ xem, KHÔNG lưu)
  real-record [tên]  : BẬT XE THẬT + QUAY VIDEO THÔ (100% Raw, lưu MP4 vào Pi)
  real-slam          : Bật Xe Thật + SLAM vẽ bản đồ
  real-nav           : Bật Xe Thật + Nav2 dẫn đường tự né vật cản
                       (Ví dụ: real-nav map:=/path/to/map.yaml)
  savemap <tên_map>  : Lưu bản đồ SLAM vừa quét xong vào thư mục maps/
  test-lidar         : Kiểm tra tia quét mắt LiDAR RPLIDAR C1
  test-cam           : Kiểm tra hình ảnh Camera Astra
  test-all           : Kiểm tra toàn bộ cảm biến trên xe

🎮 [MÔ PHỎNG & ĐỒ THỊ] (Chạy trên PC / Laptop)
  sim                : Mở thế giới ảo Gazebo + RViz + Xe mô phỏng
  gazebo             : Mở riêng Gazebo (không mở RViz)
  ai                 : Bật thuật toán AI CNN bám luống trong mô phỏng
  slam               : Bật SLAM vẽ bản đồ ảo
  nav                : Bật Nav2 dẫn đường trong mô phỏng
  plot               : Vẽ biểu đồ quỹ đạo & cảm biến Telemetry
  plot-pp            : Vẽ phân tích đáp ứng Pure Pursuit
  plot-smc           : Vẽ phân tích bộ điều khiển trượt SMC
  plot-gui           : Mở giao diện thanh trượt tinh chỉnh Live Tuning

⚙️ [BIÊN DỊCH & CÔNG CỤ]
  build              : Build nhanh workspace (bỏ qua astra_camera)
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
