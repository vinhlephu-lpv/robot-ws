#!/usr/bin/env python3
import subprocess
import os

img_dir = "/home/vinh/Màn hình nền/robot_ws/images"
os.makedirs(img_dir, exist_ok=True)

# 1. SƠ ĐỒ KHỐI CHỨC NĂNG (BLOCK DIAGRAM)
dot_block = """digraph G {
    graph [rankdir=LR, splines=spline, nodesep=0.45, ranksep=0.6, bgcolor="#f8fafc", fontname="DejaVu Sans"];
    node [fontname="DejaVu Sans", fontsize=10, style="filled,rounded", shape=box, penwidth=1.5];
    edge [fontname="DejaVu Sans", fontsize=9, penwidth=1.5];

    // KHỐI NGUỒN
    subgraph cluster_power {
        label="⚡ KHỐI NGUỒN (POWER SUPPLY)";
        fontname="DejaVu Sans Bold"; fontsize=11; fontcolor="#9a3412";
        style="filled,rounded"; fillcolor="#ffedd5"; color="#ea580c"; penwidth=2;

        bat [label="Ắc quy / Pin Li-ion 24V\n(Nguồn công suất xả cao)", fillcolor="#fed7aa", color="#ea580c"];
        buck1 [label="Mạch Buck 24V -> 5V (5A)\n(Cấp nguồn Pi 4)", fillcolor="#fed7aa", color="#ea580c"];
        buck2 [label="Mạch Buck 24V -> 5V (3A)\n(Cấp nguồn ESP32 cách ly)", fillcolor="#fed7aa", color="#ea580c"];
    }

    // KHỐI CẢM BIẾN
    subgraph cluster_sensors {
        label="📷 KHỐI CẢM BIẾN (SENSING)";
        fontname="DejaVu Sans Bold"; fontsize=11; fontcolor="#166534";
        style="filled,rounded"; fillcolor="#dcfce7"; color="#16a34a"; penwidth=2;

        cam [label="Webcam USB DVD20\n(1080p, FOV 78°)", fillcolor="#bbf7d0", color="#16a34a"];
        lidar [label="RPLIDAR C1\n(Laser 360°, Tầm xa 12m)", fillcolor="#bbf7d0", color="#16a34a"];
        imu [label="IMU ICM-20948\n(9 trục: Gyro + Accel)", fillcolor="#bbf7d0", color="#16a34a"];
        enc [label="4x Encoder Quang Học\n(200 xung/vòng PPR)", fillcolor="#bbf7d0", color="#16a34a"];
    }

    // KHỐI XỬ LÝ CẤP CAO - RASPBERRY PI 4
    subgraph cluster_rpi {
        label="🧠 KHỐI XỬ LÝ CẤP CAO (RASPBERRY PI 4 - ROS 2)";
        fontname="DejaVu Sans Bold"; fontsize=12; fontcolor="#1e40af";
        style="filled,rounded"; fillcolor="#eff6ff"; color="#2563eb"; penwidth=2.5;

        subgraph cluster_perception {
            label="Phân Khối Nhận Thức & Hợp Nhất Dữ Liệu";
            fontname="DejaVu Sans Bold"; fontsize=10; fontcolor="#1e40af";
            style="filled,rounded"; fillcolor="#dbeafe"; color="#3b82f6";
            
            ai [label="Mạng CNN ONNX\n(crop_row_cnn_best_final\nTensor 512x512)", fillcolor="#bfdbfe", color="#2563eb"];
            lidar_proc [label="Lidar Processor\n(Quét cản & Hết hàng EOR)", fillcolor="#bfdbfe", color="#2563eb"];
            madgwick [label="Bộ lọc Madgwick AHRS\n(Tính hướng Yaw/Roll/Pitch 50Hz)", fillcolor="#bfdbfe", color="#2563eb"];
            ekf [label="Bộ lọc EKF Localization\n(Dung hợp Odom + IMU 30Hz)", fillcolor="#bfdbfe", color="#2563eb"];
        }

        subgraph cluster_decision {
            label="Phân Khối Ra Quyết Định & Điều Khiển";
            fontname="DejaVu Sans Bold"; fontsize=10; fontcolor="#1e40af";
            style="filled,rounded"; fillcolor="#dbeafe"; color="#3b82f6";

            fsm [label="FSM Coordinator\n(Quản trị 9 trạng thái tự hành)", fillcolor="#93c5fd", color="#1d4ed8"];
            smc [label="Bộ điều khiển trượt SMC\n(Bám tâm luống v = 0.20m/s)", fillcolor="#93c5fd", color="#1d4ed8"];
            pp [label="Bộ điều khiển Pure Pursuit\n(Bám đường cong U-Turn 180°)", fillcolor="#93c5fd", color="#1d4ed8"];
            rrt [label="RRT* Path Planner\n(Quy hoạch đường né vật cản)", fillcolor="#93c5fd", color="#1d4ed8"];
        }

        bridge [label="Cầu nối esp32_bridge\n(Quy đổi động học vi sai & TF)", fillcolor="#bfdbfe", color="#1d4ed8"];
    }

    // KHỐI ĐIỀU KHIỂN CẤP THẤP - ESP32-S3
    subgraph cluster_esp {
        label="⚙️ KHỐI ĐIỀU KHIỂN CẤP THẤP (ESP32-S3 FIRMWARE)";
        fontname="DejaVu Sans Bold"; fontsize=12; fontcolor="#581c87";
        style="filled,rounded"; fillcolor="#faf5ff"; color="#9333ea"; penwidth=2.5;

        uart [label="Mô-đun UART Serial\n(115200 baud, 20Hz)", fillcolor="#f3e8ff", color="#9333ea"];
        safety [label="Giám sát an toàn\n(Slew Rate Limiter + Stall Detect)", fillcolor="#f3e8ff", color="#9333ea"];
        kalman [label="4 Bộ lọc Kalman 1D\n(Lọc gai nhiễu chổi than)", fillcolor="#f3e8ff", color="#9333ea"];
        pid [label="4 Vòng Lặp Kín PID Độc Lập\n+ Bù đồng tốc K_sync\n+ Khóa hướng thẳng K_heading", fillcolor="#e9d5ff", color="#7e22ce"];
        pwm [label="Bộ phát xung PWM LEDC\n(8 kênh, 7 kHz, 8-bit)", fillcolor="#f3e8ff", color="#9333ea"];
    }

    // KHỐI CÔNG SUẤT & CHẤP HÀNH
    subgraph cluster_actuation {
        label="🚗 KHỐI CÔNG SUẤT & CHẤP HÀNH";
        fontname="DejaVu Sans Bold"; fontsize=11; fontcolor="#991b1b";
        style="filled,rounded"; fillcolor="#fee2e2"; color="#dc2626"; penwidth=2;

        drv [label="4 Mạch cầu H BTS7960\n(Dòng tải công suất 43A)", fillcolor="#fecaca", color="#dc2626"];
        motors [label="4 Động cơ DC 775 24V\n(Hộp số giảm tốc 220 RPM)", fillcolor="#fecaca", color="#dc2626"];
        wheels [label="4 Bánh xe cao su D=200mm\n(Chuyển động thực tế)", fillcolor="#fca5a5", color="#b91c1c"];
    }

    // KHỐI GIÁM SÁT TỪ XA
    subgraph cluster_hmi {
        label="💻 KHỐI GIÁM SÁT TỪ XA (PC / LAPTOP)";
        fontname="DejaVu Sans Bold"; fontsize=11; fontcolor="#334155";
        style="filled,rounded"; fillcolor="#f1f5f9"; color="#64748b"; penwidth=2;

        rviz [label="Giao diện đồ họa RViz 2\n(3D Xe, Lidar, Bản đồ)", fillcolor="#e2e8f0", color="#64748b"];
        teleop [label="Bàn phím lái xe từ xa\n(WASD 15Hz nuôi Watchdog)", fillcolor="#e2e8f0", color="#64748b"];
        stream [label="Bộ nén ảnh Wi-Fi & Dataset", fillcolor="#e2e8f0", color="#64748b"];
    }

    // LIÊN KẾT NGUỒN
    bat -> drv [color="#ea580c", penwidth=2.5, label="24V Động lực"];
    bat -> buck1 [color="#ea580c", penwidth=2];
    bat -> buck2 [color="#ea580c", penwidth=2];
    buck1 -> cluster_rpi [color="#ea580c", style=dashed, label="5V (5A)"];
    buck2 -> cluster_esp [color="#ea580c", style=dashed, label="5V (3A)"];

    // LIÊN KẾT CẢM BIẾN
    cam -> ai [color="#16a34a", label="USB Video Stream"];
    lidar -> lidar_proc [color="#16a34a", label="USB-Serial 460800"];
    imu -> madgwick [color="#16a34a", label="I2C Bus 1"];

    // NỘI BỘ RPI
    ai -> fsm [color="#2563eb", label="Góc lệch & Lệch tâm"];
    lidar_proc -> fsm [color="#2563eb", label="Báo cản & EOR"];
    madgwick -> ekf [color="#2563eb", label="/imu/data"];
    ekf -> fsm [color="#2563eb", label="/odometry/filtered"];

    fsm -> smc [color="#2563eb"];
    fsm -> pp [color="#2563eb"];
    fsm -> rrt [color="#2563eb"];
    smc -> bridge [color="#2563eb", label="cmd_vel"];
    pp -> bridge [color="#2563eb", label="cmd_vel"];

    // GIAO TIẾP PI - ESP32
    bridge -> uart [color="#9333ea", penwidth=2.5, label="Gửi 'V rpm_L rpm_R' (20Hz)"];
    uart -> bridge [color="#9333ea", penwidth=2.5, label="Nhận 'ODOM v_L v_R'"];
    bridge -> ekf [color="#9333ea", style=dashed, label="/odom/raw"];

    // NỘI BỘ ESP32
    uart -> safety -> pid -> pwm [color="#9333ea"];
    pwm -> drv [color="#dc2626", penwidth=2, label="8 Kênh PWM 7kHz"];
    drv -> motors [color="#dc2626", penwidth=2, label="Điện áp 24V"];
    motors -> wheels [color="#dc2626", penwidth=2];
    wheels -> enc [color="#16a34a", style=dashed, label="Trục quay"];
    enc -> kalman [color="#16a34a", label="Xung A/B (200 PPR)"];
    kalman -> pid [color="#9333ea", label="RPM đã lọc"];
    pid -> uart [color="#9333ea", style=dashed, label="Vận tốc thực v_L, v_R"];

    // HMI KHÔNG DÂY
    cluster_rpi -> cluster_hmi [color="#64748b", penwidth=2, style=dashed, dir=both, label="Mạng Wi-Fi / CycloneDDS (Domain 0)"];
}
"""

with open(f"{img_dir}/so_do_khoi.dot", "w", encoding="utf-8") as f:
    f.write(dot_block)

subprocess.run(["dot", "-Tpng", "-Gdpi=200", f"{img_dir}/so_do_khoi.dot", "-o", f"{img_dir}/so_do_khoi.png"], check=True)
subprocess.run(["dot", "-Tsvg", f"{img_dir}/so_do_khoi.dot", "-o", f"{img_dir}/so_do_khoi.svg"], check=True)
print("✅ Rendered so_do_khoi.png and so_do_khoi.svg")

# 2. SƠ ĐỒ HOẠT ĐỘNG (OPERATIONAL FLOWCHART)
dot_flow = """digraph G {
    graph [rankdir=TB, splines=spline, nodesep=0.4, ranksep=0.45, bgcolor="#f8fafc", fontname="DejaVu Sans"];
    node [fontname="DejaVu Sans", fontsize=10, style="filled,rounded", shape=box, penwidth=1.5];
    edge [fontname="DejaVu Sans", fontsize=9, penwidth=1.5];

    start [label="🚀 BẮT ĐẦU VẬN HÀNH", shape=ellipse, fillcolor="#bfdbfe", color="#1d4ed8", fontname="DejaVu Sans Bold"];
    init [label="1. KHỞI TẠO HỆ THỐNG\\n- Nạp URDF mô hình xe 3D & TF Tree\\n- Kết nối RPLIDAR C1, Camera, IMU, ESP32\\n- Khởi động lọc Madgwick, EKF Localization & Nạp mạng AI ONNX", fillcolor="#e0f2fe", color="#0284c7"];
    idle [label="Trạng thái: IDLE\\nSẵn sàng chờ kích hoạt", fillcolor="#f1f5f9", color="#64748b"];
    loop_head [label="2. VÒNG LẶP ĐIỀU KHIỂN CHÍNH\\n(Tần số: 15 - 20 Hz)", fillcolor="#e2e8f0", color="#475569", fontname="DejaVu Sans Bold"];
    
    sense [label="3. THU THẬP & LỌC CẢM BIẾN\\n- Camera: Chụp ảnh thực tế 640x480\\n- LiDAR: Quét 360 độ khoảng cách xung quanh\\n- EKF: Cập nhật tọa độ X, Y, Hướng Yaw chuẩn", fillcolor="#dcfce7", color="#16a34a"];
    ai [label="4. AI SUY LUẬN BÁM LUỐNG\\n- Mạng CNN phân đoạn ảnh 512x512\\n- Tính độ lệch tâm (lane_offset) & góc lệch hướng (heading_error)\\n- Đánh giá độ tin cậy confidence", fillcolor="#dbeafe", color="#2563eb"];
    
    check_obs [label="Phát hiện vật cản\\nphía trước < 0.6m?", shape=diamond, fillcolor="#fef3c7", color="#d97706", fontname="DejaVu Sans Bold"];
    
    // NHÁNH CẢN
    avoid_wait [label="Trạng thái: REACTIVE_AVOID\\n- Phanh dừng xe êm bằng Slew Rate\\n- Đếm thời gian chờ 3 giây", fillcolor="#fee2e2", color="#dc2626"];
    check_blocked [label="Vật cản\\ncòn chắn đường?", shape=diamond, fillcolor="#fef3c7", color="#d97706"];
    plan_rrt [label="Trạng thái: AVOID_PLANNING\\n- RRT* Planner tìm đường vòng né\\n- Pure Pursuit bám đường tránh", fillcolor="#fee2e2", color="#dc2626"];
    check_plan_ok [label="Tìm được\\nđường né?", shape=diamond, fillcolor="#fef3c7", color="#d97706"];
    recovery [label="Trạng thái: RECOVERY\\n- Lùi xe 1.0m giải phóng không gian\\n- Nếu vẫn kẹt: Dừng khẩn cấp EMERGENCY_STOP", fillcolor="#fecaca", color="#b91c1c"];

    // NHÁNH BÁM LUỐNG & HẾT HÀNG
    check_eor [label="Hết hàng bắp EOR?\\nCamera confidence < 0.35\\nVÀ LiDAR trước mặt > 2.0m\\nVÀ hai bên hông > 0.9m", shape=diamond, fillcolor="#fef3c7", color="#d97706", fontname="DejaVu Sans Bold"];
    
    smc [label="Trạng thái: TRACKING\\n- Bộ điều khiển trượt SMC:\\n  v = 0.20 m/s\\n  w = -k*S - eta*sat(S/phi)\\n- Tự động bám thẳng theo tim luống bắp", fillcolor="#bbf7d0", color="#16a34a", fontname="DejaVu Sans Bold"];
    
    uturn [label="Trạng thái: UTURN_PLANNING & PATH_FOLLOWING\\n1. Thoát hàng: Chạy thẳng 0.7m - 2.5m vượt cây cuối\\n2. Tạo đường cong U-Turn bán nguyệt 180 độ\\n3. Pure Pursuit bẻ lái xe quay đầu sang luống mới", fillcolor="#fef08a", color="#ca8a04"];
    check_turn_done [label="Đã xoay >= 90 độ\\nVÀ Camera thấy luống mới\\nconfidence >= 0.50?", shape=diamond, fillcolor="#fef3c7", color="#d97706"];

    bridge [label="5. CẦU NỐI & ĐIỀU KHIỂN ĐỘNG CƠ (esp32_bridge)\\n- Quy đổi cmd_vel sang RPM bánh trái & phải\\n- Gửi Serial 115200 baud: 'V rpm_L rpm_R'", fillcolor="#e9d5ff", color="#7e22ce"];
    
    esp [label="6. THỜI GIAN THỰC TẠI ESP32\\n- Khởi động mềm Slew Rate Limiter\\n- Đọc 4 Encoder ngắt GPIO (200 PPR)\\n- Lọc nhiễu Kalman 1D từng bánh\\n- Chạy 4 bộ PID điều tốc + Khóa hướng thẳng K_heading\\n- Xuất xung PWM 7kHz qua 4 mạch cầu BTS7960", fillcolor="#f3e8ff", color="#9333ea"];

    motors [label="4 ĐỘNG CƠ DC 775 & BÁNH XE\\nLăn bánh trong luống bắp", shape=ellipse, fillcolor="#fca5a5", color="#b91c1c", fontname="DejaVu Sans Bold"];

    start -> init -> idle -> loop_head -> sense -> ai -> check_obs;

    check_obs -> avoid_wait [color="#dc2626", label="Có cản"];
    avoid_wait -> check_blocked;
    check_blocked -> loop_head [color="#16a34a", label="Hết cản"];
    check_blocked -> plan_rrt [color="#dc2626", label="Vẫn cản"];
    plan_rrt -> check_plan_ok;
    check_plan_ok -> loop_head [color="#16a34a", label="Thành công"];
    check_plan_ok -> recovery [color="#dc2626", label="Thất bại"];
    recovery -> loop_head;

    check_obs -> check_eor [color="#16a34a", label="Đường thông thoáng"];
    check_eor -> smc [color="#16a34a", label="Chưa hết hàng (Còn trong luống)"];
    smc -> bridge;

    check_eor -> uturn [color="#ca8a04", label="Đã hết hàng"];
    uturn -> check_turn_done;
    check_turn_done -> bridge [color="#ca8a04", label="Đang quay"];
    check_turn_done -> loop_head [color="#16a34a", label="Đã nhập luống mới"];

    bridge -> esp -> motors;
    motors -> loop_head [style=dashed, color="#475569", label="ESP32 gửi 'ODOM v_L v_R' cập nhật EKF (20Hz)"];
}
"""

with open(f"{img_dir}/so_do_hoat_dong.dot", "w", encoding="utf-8") as f:
    f.write(dot_flow)

subprocess.run(["dot", "-Tpng", "-Gdpi=200", f"{img_dir}/so_do_hoat_dong.dot", "-o", f"{img_dir}/so_do_hoat_dong.png"], check=True)
subprocess.run(["dot", "-Tsvg", f"{img_dir}/so_do_hoat_dong.dot", "-o", f"{img_dir}/so_do_hoat_dong.svg"], check=True)
print("✅ Rendered so_do_hoat_dong.png and so_do_hoat_dong.svg")

# 3. SƠ ĐỒ FSM VÀ TF TREE
dot_fsm = """digraph G {
    graph [rankdir=TB, splines=spline, nodesep=0.5, ranksep=0.6, bgcolor="#f8fafc", fontname="DejaVu Sans"];
    node [fontname="DejaVu Sans", fontsize=10, style="filled,rounded", shape=box, penwidth=1.5];
    edge [fontname="DejaVu Sans", fontsize=9, penwidth=1.5];

    start [shape=circle, width=0.3, label="", fillcolor="#000000"];
    idle [label="IDLE\\n(Chờ kích hoạt)", fillcolor="#f1f5f9", color="#64748b"];
    tracking [label="TRACKING\\n(Điều khiển trượt SMC bám luống bắp)", fillcolor="#bbf7d0", color="#16a34a", penwidth=2, fontname="DejaVu Sans Bold"];
    reactive_avoid [label="REACTIVE_AVOID\\n(Dừng khẩn chờ 3s né vật cản)", fillcolor="#fef08a", color="#ca8a04"];
    avoid_plan [label="AVOID_PLANNING\\n(RRT* Planner lập đường né)", fillcolor="#fed7aa", color="#ea580c"];
    path_follow [label="PATH_FOLLOWING\\n(Pure Pursuit bám quỹ đạo)", fillcolor="#bfdbfe", color="#2563eb"];
    uturn_plan [label="UTURN_PLANNING\\n(Thoát hàng & Tạo đường quay 180°)", fillcolor="#fef08a", color="#ca8a04"];
    uturn_exec [label="UTURN_EXECUTION\\n(Quay tại chỗ dự phòng)", fillcolor="#fef08a", color="#ca8a04"];
    recovery [label="RECOVERY\\n(Lùi xe 1.0m tự giải cứu)", fillcolor="#fecaca", color="#dc2626"];
    emergency [label="EMERGENCY_STOP\\n(Phanh khẩn ngắt toàn bộ motor)", fillcolor="#fca5a5", color="#b91c1c", fontname="DejaVu Sans Bold"];

    start -> idle;
    idle -> tracking [label="LaneDetected"];
    tracking -> reactive_avoid [color="#ca8a04", label="ObstacleDetected (<0.6m)"];
    reactive_avoid -> tracking [color="#16a34a", label="ObstacleCleared"];
    reactive_avoid -> avoid_plan [color="#ea580c", label="Chờ quá 3s"];
    avoid_plan -> path_follow [color="#2563eb", label="PlannerSuccess"];
    avoid_plan -> recovery [color="#dc2626", label="PlannerFailed"];
    path_follow -> tracking [color="#16a34a", label="PathCompleted / Vào luống"];

    tracking -> uturn_plan [color="#ca8a04", label="UTurnRequested (Hết hàng EOR)"];
    uturn_plan -> path_follow [color="#2563eb", label="Tạo đường U-Turn OK"];
    uturn_plan -> uturn_exec [color="#ca8a04", label="Dự phòng"];
    uturn_exec -> tracking [color="#16a34a", label="UTurnFinished"];

    recovery -> tracking [color="#16a34a", label="Lùi xe thành công"];
    recovery -> emergency [color="#dc2626", label="Vẫn kẹt"];
    tracking -> emergency [color="#dc2626", label="Sự cố khẩn cấp"];
}
"""

with open(f"{img_dir}/so_do_fsm.dot", "w", encoding="utf-8") as f:
    f.write(dot_fsm)

subprocess.run(["dot", "-Tpng", "-Gdpi=200", f"{img_dir}/so_do_fsm.dot", "-o", f"{img_dir}/so_do_fsm.png"], check=True)
subprocess.run(["dot", "-Tsvg", f"{img_dir}/so_do_fsm.dot", "-o", f"{img_dir}/so_do_fsm.svg"], check=True)
print("✅ Rendered so_do_fsm.png and so_do_fsm.svg")

# 4. SƠ ĐỒ CÂY TỌA ĐỘ TF
dot_tf = """digraph G {
    graph [rankdir=TB, splines=spline, nodesep=0.4, ranksep=0.5, bgcolor="#f8fafc", fontname="DejaVu Sans"];
    node [fontname="DejaVu Sans", fontsize=10, style="filled,rounded", shape=box, penwidth=1.5];
    edge [fontname="DejaVu Sans", fontsize=9, penwidth=1.5];

    map [label="map\\n(Tọa độ Bản đồ Toàn cục)", fillcolor="#e0e7ff", color="#4338ca", fontname="DejaVu Sans Bold"];
    odom [label="odom\\n(Tọa độ Hành trình Cục bộ)", fillcolor="#dbeafe", color="#1d4ed8", fontname="DejaVu Sans Bold"];
    base_footprint [label="base_footprint\\n(Tâm tiếp đất thân xe)", fillcolor="#bfdbfe", color="#2563eb", fontname="DejaVu Sans Bold"];
    base_link [label="base_link\\n(Trọng tâm thân xe z=+0.30m)", fillcolor="#93c5fd", color="#1d4ed8", fontname="DejaVu Sans Bold"];
    
    laser [label="laser_frame\\n(RPLIDAR C1 [0.2, 0, 0.22] yaw=180°)", fillcolor="#bbf7d0", color="#16a34a"];
    camera [label="camera_link\\n(Webcam [0.3, 0, 0.2] pitch=12.5°)", fillcolor="#bbf7d0", color="#16a34a"];
    cam_opt [label="camera_link_optical\\n(Chuẩn quang học Z tới, X phải, Y xuống)", fillcolor="#dcfce7", color="#16a34a"];
    imu [label="imu_link\\n(ICM-20948 tâm xe [0, 0, 0])", fillcolor="#bbf7d0", color="#16a34a"];
    gps [label="gps_link\\n(GPS nóc xe [0, 0, 0.21])", fillcolor="#bbf7d0", color="#16a34a"];

    fl [label="front_left_wheel\\n[0.20, 0.29, -0.20]", fillcolor="#fecaca", color="#dc2626"];
    fr [label="front_right_wheel\\n[0.20, -0.29, -0.20]", fillcolor="#fecaca", color="#dc2626"];
    rl [label="rear_left_wheel\\n[-0.20, 0.29, -0.20]", fillcolor="#fecaca", color="#dc2626"];
    rr [label="rear_right_wheel\\n[-0.20, -0.29, -0.20]", fillcolor="#fecaca", color="#dc2626"];

    map -> odom [color="#4338ca", penwidth=2, label="SLAM Toolbox / AMCL"];
    odom -> base_footprint [color="#1d4ed8", penwidth=2, label="ekf_filter_node (30Hz)"];
    base_footprint -> base_link [color="#2563eb", penwidth=2, label="base_footprint_joint (fixed)"];
    
    base_link -> laser [color="#16a34a"];
    base_link -> camera [color="#16a34a"];
    camera -> cam_opt [color="#16a34a"];
    base_link -> imu [color="#16a34a"];
    base_link -> gps [color="#16a34a"];

    base_link -> fl [color="#dc2626"];
    base_link -> fr [color="#dc2626"];
    base_link -> rl [color="#dc2626"];
    base_link -> rr [color="#dc2626"];
}
"""

with open(f"{img_dir}/so_do_tf_tree.dot", "w", encoding="utf-8") as f:
    f.write(dot_tf)

subprocess.run(["dot", "-Tpng", "-Gdpi=200", f"{img_dir}/so_do_tf_tree.dot", "-o", f"{img_dir}/so_do_tf_tree.png"], check=True)
subprocess.run(["dot", "-Tsvg", f"{img_dir}/so_do_tf_tree.dot", "-o", f"{img_dir}/so_do_tf_tree.svg"], check=True)
print("✅ Rendered so_do_tf_tree.png and so_do_tf_tree.svg")
