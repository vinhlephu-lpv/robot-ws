# Hướng Dẫn Vận Hành Hệ Thống Xe Tự Hành ROS 2 (LuanVan)

Chào mừng bạn đến với không gian làm việc `robot_ws`. Đây là nơi chứa toàn bộ mã nguồn mô phỏng, điều khiển và định vị/bản đồ của hệ thống xe tự hành đi giữa các luống bắp.
Dưới đây là danh sách chi tiết các lệnh để chạy hệ thống trên bất kỳ máy tính nào đã cài đặt ROS 2 Jazzy.

---

> [!IMPORTANT]
> **LƯU Ý KHI MANG SANG MÁY KHÁC:** 
> Cần đổi lại đường dẫn khác (cd vào thư mục robot-ws bằng lệnh cd ~/robot-ws)

## 0. Build (Biên Dịch) Hệ Thống (Chỉ làm ở lần đầu tiên hoặc khi có thay đổi code)
Trước khi chạy bất kỳ lệnh nào, hãy chắc chắn rằng bạn đã build toàn bộ không gian làm việc.
Mở Terminal mới và chạy:
```bash
cd "/home/vinh/Màn hình nền/Luanvan/Luan van/robot_ws"
source /opt/ros/jazzy/setup.bash
colcon build
```

---

## 1. Khởi Động Thế Giới Ảo (Gazebo + RViz + Spawn Xe)
Bước này sẽ mở thế giới ảo Gazebo (chứa các luống bắp), xuất hiện mô hình xe robot, bật các cảm biến (Camera, Lidar, IMU) và mở cửa sổ RViz 2 để quan sát dữ liệu.

* **Mở Terminal 1:**
```bash
cd "/home/vinh/Màn hình nền/Luanvan/Luan van/robot_ws"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch my_robot_simulation sim.launch.py
```
*Ghi chú: RViz đã được cài đặt mặc định để tự động hiển thị Map, Lidar và mô hình xe với giao diện `costmap` dễ nhìn.*

---

## 2. Các Tùy Chọn Điều Khiển Xe
Bạn có thể chọn 1 trong 2 cách dưới đây để điều khiển xe chạy quanh vườn bắp:

### Cách 2A: Điều khiển bằng tay (Bàn phím)
Nếu bạn muốn tự tay lái xe đi dạo xung quanh.
* **Mở Terminal 2:**
```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
*(Sử dụng các phím U, I, O, J, K, L, M, <, > để lái xe)*

### Cách 2B: Bật AI Tự Lái (CNN Controller)
Bật mô hình Trí tuệ Nhân tạo (CNN) để xe tự động nhận diện hàng bắp và tự lái dọc theo các luống bắp.
* **Mở Terminal 2:**
```bash
cd "/home/vinh/Màn hình nền/Luanvan/Luan van/robot_ws"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch my_robot_controller control.launch.py
```

---

## 3. Vẽ Bản Đồ (SLAM)
Song song với việc xe đang chạy (bằng tay hoặc AI), bạn có thể bật thuật toán SLAM để xe tự động xây dựng bản đồ khu vườn thông qua cảm biến Lidar.

* **Mở Terminal 3:**
```bash
cd "/home/vinh/Màn hình nền/Luanvan/Luan van/robot_ws"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch my_robot_slam slam.launch.py
```
*(Bạn sẽ nhìn thấy bản đồ SLAM sắc nét và vùng đệm an toàn Costmap Inflation tự động mở rộng theo bước di chuyển của xe trên cửa sổ RViz)*

---

## 4. Lưu Bản Đồ
Sau khi xe đã chạy quanh vườn và vẽ xong một bản đồ hoàn chỉnh, bạn lưu lại thành file ảnh `.pgm` và file cấu hình `.yaml` để sử dụng cho báo cáo hoặc dẫn đường.

* **Mở Terminal 4:**
```bash
cd "/home/vinh/Màn hình nền/robot_ws"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
mkdir -p maps
cd maps
ros2 run my_robot_slam map_saver -f my_corn_farm_map
```
*(Hoặc dùng lệnh gọi service trực tiếp từ SLAM Toolbox: `ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'my_corn_farm_map'}}"`)*
*Lệnh này sẽ tạo ra 2 file `my_corn_farm_map.yaml` và `my_corn_farm_map.pgm` trong thư mục `robot_ws/maps`.*

---

## 5. Dẫn Đường Tự Động (Nav2 - A* Planner + Regulated Pure Pursuit)
Sau khi bật mô phỏng và SLAM, bạn có thể kích hoạt hệ thống **Nav2** để điều khiển xe tự động tìm đường tránh bụi bắp và di chuyển tới đích:

* **Mở Terminal 4 (hoặc Terminal mới):**
```bash
cd "/home/vinh/Màn hình nền/robot_ws"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch my_robot_navigation nav.launch.py
```

* **Cách điều khiển xe và quan sát A* tránh vật cản trên RViz 2:**
  1. Thế giới mô phỏng đã được thiết lập sẵn **vật cản màu cam/đỏ (`small_obstacle`)** tại vị trí $x = 1.80\text{m}, y = 0.12\text{m}$ (nằm sát bên lề phải luống bắp $y = 0.0\text{m}$, chớm vào đường chạy của bánh phải).
  2. Trên thanh công cụ phía trên của RViz 2, bấm chọn nút **2D Goal Pose** (hoặc phím tắt `G`).
  3. Click chuột trái vào vị trí đích ở phía sau vật cản (ví dụ điểm cuối luống bắp tại $x \approx 3.5\text{m}, y \approx 0.5\text{m}$) và kéo mũi tên hướng thẳng theo chiều $+x$, sau đó thả chuột.
  4. **Quan sát thuật toán A\* và xe điều hướng tránh vật cản:**
     - **LiDAR & Costmap:** Cảm biến LiDAR phát hiện vật cản hình trụ, SLAM cập nhật lên `/map` và lớp chi phí Costmap phồng lên xung quanh vật cản.
     - **Đường đi toàn cục A\* (`/plan` màu xanh lá cây):** Planner A* tự động tính toán đường đi uốn lượn thông minh, lách sang trái ($y \approx 0.58 - 0.60\text{m}$) để thân xe $0.68\text{m}$ lọt qua khoảng trống $0.84\text{m}$ an toàn rồi trở về tim luống $y = 0.50\text{m}$.
     - **Điều khiển bám quỹ đạo RPP (`/local_plan` màu xanh dương):** Bộ điều khiển Regulated Pure Pursuit phát lệnh `/cmd_vel` điều khiển xe chạy mượt mà theo đường cong tránh vật cản để tới đích an toàn mà không va chạm.

* **Cách HỦY Mục Tiêu Dẫn Đường (Cancel Goal) khi xe đi sai hướng:**
  - **Cách 1 (Nút bấm trên RViz 2 - Khuyên dùng):** Trên bảng điều khiển **Navigation 2** (bên cạnh tab Displays/Views trong RViz 2), bấm nút đỏ **`Cancel Nav`**. Xe sẽ lập tức phanh dừng lại tại chỗ, xóa đường vẽ `/plan` và sẵn sàng để bạn bấm `G` vẽ lại đích mới.
  - **Cách 2 (Bằng dòng lệnh Terminal):** Chạy lệnh:
    ```bash
    ros2 run my_robot_navigation cancel_nav
    ```

---

## Lời Khuyên Hữu Ích
- **Luôn nhớ "Source":** Mỗi khi bạn mở một Terminal mới, hệ điều hành sẽ không biết các lệnh `ros2` nằm ở đâu. Bạn luôn phải chạy lệnh `source install/setup.bash` trước khi chạy các phần mềm trong gói.
- **Lỗi ROS_LOCALHOST_ONLY:** Nếu nhiều máy tính đang chạy chung mạng WiFi mà bị nhiễu sóng ROS 2 của nhau, hãy nhớ thiết lập biến môi trường `export ROS_LOCALHOST_ONLY=1` trong file `~/.bashrc`.

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
Toàn bộ logic quay đầu từ thư mục gốc ngày `2-08-2026` được thiết lập chuẩn xác:
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

