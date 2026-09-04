#!/usr/bin/env python3
import subprocess
import os

img_dir = "/home/vinh/Màn hình nền/robot_ws/images"
os.makedirs(img_dir, exist_ok=True)

dot_content = """digraph G {
    graph [rankdir=TB, splines=spline, nodesep=0.6, ranksep=0.8, bgcolor="#ffffff", fontname="DejaVu Sans"];
    node [fontname="DejaVu Sans", fontsize=11, style="filled,rounded", shape=box, penwidth=2];
    edge [fontname="DejaVu Sans", fontsize=10, penwidth=2];

    // HÀNG TRÊN: CHIỀU ĐIỀU KHIỂN TIẾN (FORWARD DOWNLINK)
    subgraph cluster_forward {
        label="⬇️ NHÁNH ĐIỀU KHIỂN XUỐNG (FORWARD CONTROL PATH)";
        fontname="DejaVu Sans Bold"; fontsize=12; fontcolor="#1e40af";
        style="filled,rounded"; fillcolor="#eff6ff"; color="#3b82f6"; penwidth=2;

        cam [label="📷 WEBCAM USB DVD20\\n[INPUT]: Ánh sáng vườn bắp\\n[OUTPUT]: Ảnh BGR 640x480 (60 FPS)", fillcolor="#dcfce7", color="#16a34a"];
        rpi [label="🧠 RASPBERRY PI 4 (ROS 2)\\n• AI CNN: Phân đoạn tìm tâm luống bắp\\n• SMC Controller: Tính góc bẻ lái ω\\n• esp32_bridge: Quy đổi vi sai sang RPM", fillcolor="#dbeafe", color="#2563eb", fontname="DejaVu Sans Bold"];
        esp_ctrl [label="⚙️ ESP32-S3 (BỘ ĐIỀU TỐC PID)\\n• Slew Rate: Khởi động mềm hạn chế gia tốc\\n• 4 Vòng lặp kín PID điều tốc độc lập\\n• Khóa hướng thẳng vi sai K_heading = 35.0", fillcolor="#f3e8ff", color="#9333ea", fontname="DejaVu Sans Bold"];
        drv [label="⚡ 4 MẠCH CẦU H BTS7960\\n• Nhận 8 kênh PWM 7kHz (0 - 255)\\n• Đóng ngắt Mosfet công suất 43A\\n• Cấp điện áp động lực 24V", fillcolor="#fee2e2", color="#dc2626"];
    }

    // ĐỐI TƯỢNG ĐIỀU KHIỂN (PLANT)
    subgraph cluster_plant {
        label="🚗 ĐỐI TƯỢNG CHẤP HÀNH (ROBOT PLANT)";
        fontname="DejaVu Sans Bold"; fontsize=12; fontcolor="#991b1b";
        style="filled,rounded"; fillcolor="#fff1f2"; color="#e11d48"; penwidth=2;

        motors [label="4 ĐỘNG CƠ DC 775 24V + 4 BÁNH XE (D=200mm)\\n[CHUYỂN ĐỘNG THỰC TẾ TRONG LUỐNG BẮP]", fillcolor="#fecdd3", color="#be123c", fontname="DejaVu Sans Bold"];
    }

    // HÀNG DƯỚI: CHIỀU HỒI TIẾP NGƯỢC (FEEDBACK UPLINK)
    subgraph cluster_feedback {
        label="⬆️ NHÁNH PHẢN HỒI LÊN (CLOSED-LOOP FEEDBACK)";
        fontname="DejaVu Sans Bold"; fontsize=12; fontcolor="#065f46";
        style="filled,rounded"; fillcolor="#ecfdf5"; color="#059669"; penwidth=2;

        enc [label="4x CẢM BIẾN ENCODER\\n[INPUT]: Đĩa đục lỗ gắn trục bánh\\n[OUTPUT]: Xung ngắt GPIO (200 xung/vòng)", fillcolor="#bbf7d0", color="#16a34a"];
        esp_filter [label="ESP32 XỬ LÝ HỒI TIẾP\\n• 4 Bộ lọc Kalman 1D khử gai chổi than\\n• Tính vận tốc thực tế v_L, v_R (m/s)\\n• Khép vòng kín PID tốc độ", fillcolor="#d1fae5", color="#059669", fontname="DejaVu Sans Bold"];
        ekf_node [label="PI 4: DUNG HỢP EKF (robot_localization)\\n• esp32_bridge tính Dead Reckoning\\n• Dung hợp IMU 9 trục ICM-20948\\n• Xuất tọa độ TF: odom -> base_footprint", fillcolor="#a7f3d0", color="#047857", fontname="DejaVu Sans Bold"];
    }

    // 1. LUỒNG TIẾN (ĐI XUỐNG)
    cam -> rpi [color="#16a34a", label="Ảnh số /camera/color/image_raw"];
    rpi -> esp_ctrl [color="#2563eb", penwidth=3, label="⬇️ LỆNH GỬI XUỐNG (Serial 115200 baud, 20Hz)\\nChuỗi: 'V <rpm_L> <rpm_R>\\\\n'"];
    esp_ctrl -> drv [color="#9333ea", penwidth=2.5, label="8 Kênh xung PWM 7kHz\\n(0 - 255)"];
    drv -> motors [color="#dc2626", penwidth=2.5, label="Điện áp 24V\\n(Dòng đỉnh 43A)"];

    // 2. TẠO TÍN HIỆU ĐO LƯỜNG
    motors -> enc [color="#be123c", penwidth=2.5, label="Trục bánh xe quay"];
    enc -> esp_filter [color="#16a34a", penwidth=2, label="Xung A/B 200 PPR (50ms)"];

    // 3. VÒNG KÍN TRONG (PID ESP32)
    esp_filter -> esp_ctrl [color="#9333ea", style=dashed, penwidth=2, label="Hồi tiếp tốc độ thực\\n(Measured RPM)"];

    // 4. LUỒNG HỒI TIẾP NGOÀI (ĐI LÊN PI 4)
    esp_filter -> ekf_node [color="#059669", penwidth=3, label="⬆️ BÁO CÁO GỬI LÊN (Serial 20Hz)\\nChuỗi: 'ODOM <v_L> <v_R>\\\\n'"];
    ekf_node -> rpi [color="#047857", penwidth=2.5, label="Tọa độ chuẩn xác (X, Y, Yaw)\\nKhép kín vòng lặp bám luống bắp!"];
}
"""

with open(f"{img_dir}/so_do_khoi_chi_tiet_io.dot", "w", encoding="utf-8") as f:
    f.write(dot_content)

subprocess.run(["dot", "-Tpng", "-Gdpi=200", f"{img_dir}/so_do_khoi_chi_tiet_io.dot", "-o", f"{img_dir}/so_do_khoi_chi_tiet_io.png"], check=True)
subprocess.run(["dot", "-Tsvg", f"{img_dir}/so_do_khoi_chi_tiet_io.dot", "-o", f"{img_dir}/so_do_khoi_chi_tiet_io.svg"], check=True)
print("✅ Re-rendered loop so_do_khoi_chi_tiet_io.png")
