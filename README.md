# Hướng Dẫn Vận Hành Hệ Thống Xe Tự Hành ROS 2 (LuanVan)

Hệ thống mã nguồn xe tự hành chạy trong vườn bắp (hỗ trợ cả **Mô Phỏng 3D Gazebo** trên PC và **Phần Cứng Thực Tế** trên Raspberry Pi).

---

## ⚡ 1. Cài Đặt Bộ Lệnh Tắt Nhanh (Chỉ làm 1 lần duy nhất)

Để có thể gõ **1-3 chữ** là chạy được tất cả các tính năng mà không cần nhớ đường dẫn dài hay gõ lệnh `source` thủ công:

### **Trên Máy Tính (PC):**
Mở terminal và chạy lệnh:
```bash
echo "source '$HOME/Màn hình nền/robot_ws/aliases.sh'" >> ~/.bashrc
source ~/.bashrc
```

### **Trên Raspberry Pi:**
Mở terminal và chạy lệnh:
```bash
echo "source '$HOME/robot-ws/aliases.sh'" >> ~/.bashrc
source ~/.bashrc
```

*(Sau khi cài đặt, mỗi khi mở terminal mới bạn chỉ cần gõ đúng 1 từ như bảng bên dưới).*

---

## 📋 2. Bảng Tra Cứu Lệnh Tắt Siêu Ngắn (1-3 Chữ)

### 🔹 **A. Các Lệnh Dùng Cho MÔ PHỎNG (Chạy trên PC)**

| Lệnh tắt | Chức năng chi tiết |
| :--- | :--- |
| **`build`** | Tự động `colcon build` và nạp môi trường workspace |
| **`sim`** | Mở thế giới 3D Gazebo + RViz 2 + Xuất hiện xe tự hành |
| **`teleop`** / **`wasd`** | Lái xe bằng bàn phím chuẩn game (**W, A, S, D, Space**) |
| **`ai`** | Bật AI CNN nhận diện hàng bắp & bám luống tự động |
| **`slam`** | Bật SLAM Toolbox vẽ bản đồ thời gian thực |
| **`savemap <tên>`** | Lưu bản đồ (ví dụ: `savemap map_vuon_bap`) vào thư mục `maps/` |
| **`nav`** | Bật Nav2 dẫn đường tự động A* & Pure Pursuit tránh vật cản |
| **`cancel`** | Hủy mục tiêu dẫn đường Nav2 ngay lập tức |
| **`rviz`** | Mở giao diện đồ họa RViz 2 chuẩn |
| **`robot-help`** | Xem nhanh bảng hướng dẫn các phím tắt trong terminal |

---

### 🔹 **B. Các Lệnh Dùng Cho ROBOT THẬT (Chạy trên Raspberry Pi)**

| Lệnh tắt | Chức năng chi tiết |
| :--- | :--- |
| **`build`** | Build lại code trên Pi |
| **`test-lidar`** | Kiểm tra kết nối & dữ liệu cảm biến RPLIDAR C1 |
| **`test-cam`** | Kiểm tra hình ảnh & độ sâu Camera Astra Mini S |
| **`test-all`** | Kiểm tra toàn bộ cảm biến thật cùng lúc |
| **`real-robot`** | Khởi động toàn bộ phần cứng (Lidar, Camera, ESP32 Bridge, TF) |
| **`real-slam`** | Chạy Robot thật + Thuật toán SLAM vẽ bản đồ thực tế |
| **`real-nav`** | Chạy Robot thật + Nav2 dẫn đường thực tế |

---

## 🚀 3. Quy Trình Vận Hành Thường Gặp

### Kịch Bản 1: Chạy Mô Phỏng & AI Tự Lái (Trên PC)
1. **Terminal 1:** `sim` *(Mở Gazebo + RViz)*
2. **Terminal 2:** `ai` *(Bật AI tự lái qua các luống bắp và quay đầu U-Turn)*
3. **Terminal 3:** `slam` *(Vẽ bản đồ khi xe đang di chuyển)*
4. **Terminal 4:** `savemap ban_do_lan_1` *(Lưu bản đồ khi vẽ xong)*

---

### Kịch Bản 2: Dẫn Đường Tự Động Tránh Vật Cản Nav2 (Trên PC)
1. **Terminal 1:** `sim` *(Khởi động mô phỏng)*
2. **Terminal 2:** `slam` *(Cập nhật vị trí và Costmap vật cản)*
3. **Terminal 3:** `nav` *(Kích hoạt hệ thống Nav2)*
4. Trên cửa sổ RViz 2: Bấm phím **`G`** (hoặc nút **2D Goal Pose**), click vào vị trí đích phía sau chướng ngại vật $\to$ Xe sẽ tự động tính đường uốn lượn tránh vật cản và đến đích.
5. Nếu cần dừng khẩn cấp: Gõ `cancel` trên Terminal hoặc bấm nút đỏ **`Cancel Nav`** trên RViz 2.

---

### Kịch Bản 3: Vận Hành Xe Thật & Giám Sát Từ Xa Qua Mạng LAN
*Đảm bảo Pi và PC cùng bắt chung 1 mạng Wi-Fi/LAN.*

1. **Trên Raspberry Pi:**
   - Mở Terminal: `real-slam` *(hoặc `real-robot`)*
2. **Trên Máy Tính (PC):**
   - Mở Terminal 1: `rviz` *(Xem hình ảnh Camera, tia Lidar và Bản đồ đang vẽ trực tiếp từ Pi truyền về)*
   - Mở Terminal 2: `teleop` *(Lái xe thật từ xa bằng bàn phím máy tính)*

---

## 🔄 4. Hướng Dẫn Đồng Bộ Code 100% Giữa PC và Pi (Git)

Mỗi khi bạn chỉnh sửa hoặc viết thêm code trên máy tính, để đẩy sang Raspberry Pi:

### **Bước 1: Trên Máy Tính (PC)**
```bash
cd "$HOME/Màn hình nền/robot_ws"
git add .
git commit -m "update: cap nhat tinh nang moi"
git push
```

### **Bước 2: Trên Raspberry Pi**
```bash
cd ~/robot-ws
git reset --hard origin/main
git pull
build
```

---

## 🚨 CÁC LƯU Ý NGHIÊM TRỌNG & NGUYÊN TẮC BẤT BIẾN (KHÔNG ĐƯỢC PHẠM PHẢI / KHÔNG ĐƯỢC SỬA)

Để đảm bảo toàn bộ hệ thống mô phỏng và điều khiển (AI Tracking, U-Turn Turning, Physics, RViz 2, SLAM) hoạt động ổn định và chính xác 100%, tất cả các quy tắc dưới đây là **BẤT DI BẤT DỊCH**:

### 1. Cấu Hình Vật Lý Ma Sát Mô Phỏng (Physics & Friction Integrity)
- **Ma sát bánh xe (`my_robot_description/urdf/robot.urdf.xacro`)**:
  - Cả 4 bánh xe của robot **bắt buộc giữ nguyên hệ số ma sát chuẩn**:
    ```xml
    <gazebo reference="front_left_wheel">
      <material>Gazebo/Black</material>
      <mu1>1.0</mu1>
      <mu2>1.0</mu2>
    </gazebo>
    <!-- Tương tự cho front_right_wheel, rear_left_wheel, rear_right_wheel -->
    ```
  - **CẤM:** Tuyệt đối không sửa ma sát bánh xe thành dạng bất đối xứng (`mu1=100.0, mu2=0.1`) hay thêm thông số lò xo tiếp xúc cứng (`kp=10000000.0, kd=1.0`). Các thông số này sẽ làm xe đánh lái vi sai 4 bánh (skid-steer) bị kẹt, trượt bánh, mất mô-men xoay, khiến lần quay đầu số 2 không đủ góc cua!
- **Ma sát mặt đất (`my_robot_simulation/worlds/corn_field.sdf`)**:
  - Mặt sàn Gazebo ODE bắt buộc giữ nguyên:
    ```xml
    <surface>
      <friction>
        <ode>
          <mu>100</mu>
          <mu2>50</mu2>
        </ode>
      </friction>
    </surface>
    ```

---

### 2. Chu Trình Điều Khiển & Thuật Toán Quay Đầu (Turning Logic & Pure Pursuit)
Toàn bộ logic quay đầu được thiết lập chuẩn xác:
- **Chu kỳ FSM**: `TRACKING` $\to$ `DRIVE_OUT` $\to$ `UTURN_PLANNING` $\to$ `PATH_FOLLOWING` $\to$ (`UTURN_EXECUTION` nếu cần) $\to$ `TRACKING`.
- **Khoảng cách thoát hàng (`drive_out_distance: 2.5`)**:
  - Sau khi nhận diện hết hàng (EOR), xe phải chạy thẳng $2.5\text{m}$ để thân xe và 4 bánh hoàn toàn vượt qua bụi bắp cuối cùng trước khi xoay.
- **Tốc độ quay đầu (`params_sim.yaml`)**:
  - `turn_linear_speed: 0.25 m/s`
  - `turn_angular_speed: 1.50 rad/s`
- **Quỹ đạo cung tròn U-Turn (`generate_backup_uturn_path`)**:
  - Đoạn clearance thẳng: $0.40\text{m}$.
  - Cung tròn 180° bán nguyệt: Bán kính $R = \frac{|\Delta y|}{2} = 0.40\text{m}$.
  - Đoạn dẫn hướng thẳng vào luống mới: Chiều dài $3.5\text{m}$ nhằm căn chỉnh hướng xe đạt 100% thẳng hàng trước khi bàn giao.
- **Bộ điều khiển Pure Pursuit (`PurePursuitController` trong `controllers.py`)**:
  - Khoảng cách nhìn trước: $L_d = 0.35\text{m}$.
  - Điều tốc thích nghi: $v = v_{turn} \times \cos^2(\text{yaw\_error})$, $\omega = 3.5 \times \text{yaw\_error}$.
- **Điều kiện bàn giao sang `TRACKING`**:
  - Yêu cầu xe đã xoay thực tế $\ge 90^\circ$ VÀ camera nhận diện được luống bắp mới với độ tin cậy `confidence >= 0.50`.
- **Bộ nhận diện hết hàng (`perception_manager.py`)**:
  - LiDAR check: `front_min_dist > 2.0m` VÀ 2 bên `> 0.90m`.
  - Camera check: `confidence < 0.25`.

---

### 3. Đồng Bộ Thời Gian Mô Phỏng & Cấu Hình Môi Trường (Simulation Invariants)
- **Luôn bật `use_sim_time: True`**: Tất cả các node (`cnn_driver`, `rviz2`, `slam_toolbox`, `robot_state_publisher`, `bridge`) phải dùng chung thời gian mô phỏng Gazebo để chống lỗi đứt quãng TF và chớp nháy trắng.
- **Tần số Odometry**: Plugin `diff_drive` trong file URDF được cố định `<update_rate>50</update_rate>` để khớp tần số với SLAM và RViz.
- **Bảo toàn các thành phần phụ trợ**: Tuyệt đối không xóa, không can thiệp làm hỏng các gói `my_robot_slam`, `my_robot_navigation`, `my_robot_bringup`, cũng như các file cấu hình RViz 2 (`display.rviz`).
