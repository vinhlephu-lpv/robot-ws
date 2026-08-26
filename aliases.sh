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
alias build="cd \"$WS_DIR\" && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install && source install/setup.bash"
alias rb-build="build"

# 2. Các lệnh chạy Mô phỏng (PC)
alias sim="load_ws && ros2 launch my_robot_simulation sim.launch.py"
alias gazebo="load_ws && ros2 launch my_robot_simulation sim.launch.py use_rviz:=false"
alias sim-only="gazebo"
alias ai="load_ws && ros2 launch my_robot_controller control.launch.py"
alias slam="load_ws && ros2 launch my_robot_slam slam.launch.py"
alias teleop="load_ws && ros2 run my_robot_controller teleop_wasd"
alias wasd="teleop"
alias rviz="load_ws && rviz2 -d \"$WS_DIR/src/my_robot_description/rviz/display.rviz\""
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

# Trợ giúp
alias robot-help="cat << 'EOF'
=====================================================
 Danh Sách Lệnh Tắt Nhanh Xe Tự Hành (1-3 Chữ)
=====================================================
[Lệnh Chung]
  build          : Biên dịch toàn bộ workspace
  teleop         : Lái xe bằng bàn phím (U, I, O, J, K, L)
  rviz           : Mở RViz 2 hiển thị đồ họa

[Mô Phỏng & Phân Tích Đồ Thị]
  sim            : Bật thế giới Gazebo + RViz + Xe ảo
  ai             : Bật AI CNN nhận diện luống bắp tự lái
  slam           : Bật SLAM vẽ bản đồ
  nav            : Bật Nav2 dẫn đường tự động
  savemap <tên>  : Lưu bản đồ vào thư mục maps/
  cancel         : Hủy mục tiêu dẫn đường
  plot           : Vẽ dữ liệu quỹ đạo & cảm biến xe vừa chạy (Telemetry)
  plot-pp        : Vẽ 4 biểu đồ phân tích đáp ứng Pure Pursuit
  plot-smc       : Vẽ đáp ứng bước bộ điều khiển trượt SMC
  plot-gui       : Mở giao diện tương tác Live Tuning GUI (thanh trượt)

[Robot Thật - Chạy trên Raspberry Pi]
  test-lidar     : Kiểm tra cảm biến RPLIDAR C1
  test-cam       : Kiểm tra Camera Astra
  test-all       : Kiểm tra toàn bộ cảm biến thật
  real-robot     : Bật toàn bộ phần cứng robot thật
  real-slam      : Bật Robot thật + SLAM vẽ bản đồ
  real-nav       : Bật Robot thật + Nav2 dẫn đường
=====================================================
EOF
"
