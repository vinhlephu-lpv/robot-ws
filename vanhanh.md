# 🚀 HƯỚNG DẪN VẬN HÀNH TOÀN DIỆN XE TỰ HÀNH TRÊN RASPBERRY PI (XE THẬT)

> **Tài liệu quy trình chuẩn từ khi vừa Clone code về Pi đến lúc xe lăn bánh ngoài thực tế**  
> *Hỗ trợ đầy đủ:* LiDAR, USB Webcam, ESP32 Encoders, IMU, GPS và AI CNN Bám luống.

---

## ⚡ QUY TRÌNH KHỞI ĐỘNG HÀNG NGÀY (MỞ XE LÊN LÀ CHẠY)

### 📋 Bước 0: Bật nguồn & Mạng Wi-Fi
1. **Nguồn 24V:** Bật công tắc pin 24V cấp nguồn cho 4 mạch cầu H BTS7960 và cắm cáp USB nối Pi với ESP32.
2. **Wi-Fi:** Đảm bảo Raspberry Pi và Laptop kết nối **cùng một mạng Wi-Fi** (hoặc phát Hotspot từ điện thoại).
3. **Đồng bộ thời gian (Rất quan trọng để RViz mượt, không drop frame):**
   ```bash
   sudo systemctl restart systemd-timesyncd
   ``` 

### 🤖 Bước 1: Trên Raspberry Pi (Khởi động phần cứng xe)
Mở Terminal trên Pi (hoặc SSH từ Laptop):
```bash
real-robot
```
*(Nếu có cập nhật code mới từ GitHub: `cd ~/robot-ws && git pull origin main && real-robot`)*

> [!NOTE]
> Lệnh `real-robot` tự động khởi chạy:
> - LiDAR RPLIDAR C1 (quét 360° 10Hz)
> - USB Webcam + Bridge nén JPEG gửi qua Wi-Fi (~200 KB/s không làm nghẽn LiDAR)
> - ESP32 Hardware Bridge (PID động cơ + đọc 4 Encoder)
> - Mô hình 3D URDF & Cây toạ độ TF

### 💻 Bước 2: Trên Laptop (Mở giao diện quan sát)
Mở Terminal trên Laptop:
```bash
laptop-view
```
*(Tự động kích hoạt receiver giải nén ảnh camera và mở RViz2 hiển thị xe 3D, LiDAR, Camera).*

### 🎮 Bước 3: Trên Laptop (Lái xe bằng bàn phím)
Mở thêm **1 Tab Terminal mới** trên Laptop:
```bash
wasd
```
*(Click chuột vào cửa sổ terminal `wasd`, dùng các phím `W, A, S, D` để lái xe, phím cách `Space` để dừng khẩn cấp).*

### 🗺️ Bảng tra nhanh các chế độ vận hành:
| Mục đích | Lệnh trên Pi | Lệnh trên Laptop |
| :--- | :--- | :--- |
| **Bình thường: Chỉ xem, KHÔNG quay, KHÔNG lưu** | `real-robot` | `laptop-view` & `wasd` |
| **Quay Video THÔ làm Dataset (Có lưu MP4 vào Pi)** | `real-record [tên]` | `get-video` $\to$ `extract-dataset` |
| **Quét bản đồ SLAM** | `real-slam` | `laptop-view` & `wasd` $\to$ `savemap` |
| **Tự hành Nav2 (Tự né vật cản)** | `real-nav` | `laptop-view` $\to$ chọn **2D Goal Pose** |

---

## 📑 MỤC LỤC
0. [Quy Trình Khởi Động Hàng Ngày (Mở Xe Lên Là Chạy)](#-quy-trình-khởi-động-hàng-ngày-mở-xe-lên-là-chạy)
1. [Giai Đoạn 1: Thiết Lập Môi Trường Trên Raspberry Pi (Làm 1 Lần)](#-giai-đoạn-1-thiết-lập-môi-trường-trên-raspberry-pi-làm-1-lần)
2. [Giai Đoạn 2: Cố Định Cổng USB (Udev Rules) Cho Cảm Biến](#-giai-đoạn-2-cố-định-cổng-usb-udev-rules-cho-cảm-biến)
3. [Giai Đoạn 3: Tích Hợp GPS & IMU Vào Hệ Thống (Sensor Fusion EKF)](#-giai-đoạn-3-tích-hợp-gps--imu-vào-hệ-thống-sensor-fusion-ekf)
4. [Giai Đoạn 4: Kiểm Tra Từng Bộ Phận Phần Cứng (Hardware Test)](#-giai-đoạn-4-kiểm-tra-từng-bộ-phận-phần-cứng-hardware-test)
5. [Giai Đoạn 5: Các Kịch Bản Vận Hành Xe Thật](#-giai-đoạn-5-các-kịch-bản-vận-hành-xe-thật)
6. [Bảng Tra Cứu Sự Cố Thường Gặp & Cách Khắc Phục (Troubleshooting)](#-bảng-tra-cứu-sự-cố-thường-gặp--cách-khắc-phục)

---

## 🛠️ GIAI ĐOẠN 1: THIẾT LẬP MÔI TRƯỜNG TRÊN RASPBERRY PI (LÀM 1 LẦN)

Sau khi vừa clone code về Raspberry Pi (ví dụ thư mục `~/robot-ws` hoặc `~/robot_ws`), mở Terminal trên Pi và thực hiện:

### 1. Cài đặt các gói phụ thuộc ROS 2 & Hệ thống:
```bash
sudo apt update
sudo apt install -y python3-pip python3-colcon-common-extensions \
    libuvc-dev libgoogle-glog-dev libgflags-dev libusb-1.0-0-dev \
    ros-jazzy-robot-localization ros-jazzy-slam-toolbox ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup ros-jazzy-tf2-ros ros-jazzy-joint-state-publisher \
    ros-jazzy-robot-state-publisher ros-jazzy-xacro ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-nmea-msgs ros-jazzy-imu-tools
```

### 2. Cài đặt thư viện Python xử lý Serial, AI & Ảnh:
```bash
pip3 install pyserial numpy onnxruntime opencv-python --break-system-packages
```

### 3. Cấp quyền cổng Serial cho tài khoản Pi:
*(Giúp Pi tự động đọc/ghi cổng Serial USB mà không cần gõ `sudo chmod 666` mỗi lần cắm cáp)*
```bash
sudo usermod -a -G dialout $USER
newgrp dialout
```

### 4. Đăng ký bộ phím tắt thông minh (`aliases.sh`):
```bash
# Nếu thư mục là robot-ws:
echo "source '$HOME/robot-ws/aliases.sh'" >> ~/.bashrc
source ~/.bashrc
```

### 5. Cấu hình phần cứng trên Raspberry Pi 5 (Bắt buộc):
- **Mở khóa nguồn USB 1.6A (Tránh sụt áp/ngắt thiết bị khi cắm Camera + LiDAR + ESP32):**
  ```bash
  echo "usb_max_current_enable=1" | sudo tee -a /boot/firmware/config.txt
  ```
- **Bật cổng Serial GPIO cho GPS (`/dev/ttyAMA0`):**
  Chạy `sudo raspi-config` -> **Interface Options** -> **Serial Port**:
  - *Login shell over serial:* Chọn **NO**
  - *Serial port hardware enabled:* Chọn **YES**
  - Khởi động lại: `sudo reboot`

### 6. Biên dịch Workspace:
```bash
cd ~/robot-ws   # hoặc cd ~/robot_ws
build
```
> *(Lệnh `build` sẽ tự động chạy `colcon build --symlink-install` và nạp môi trường `install/setup.bash`).*

---

## 🔌 GIAI ĐOẠN 2: SƠ ĐỒ KẾT NỐI CỔNG & CỐ ĐỊNH USB (UDEV RULES)

### 1. Sơ đồ cắm cổng tối ưu trên Raspberry Pi 5:
* **DVD20 USB Webcam:** Cắm vào **Cổng USB** $\to$ nhận diện cổng `/dev/video0`.
* **LiDAR RPLIDAR C1:** Cắm vào **Cổng USB 3.0 (Màu xanh dương)**.
* **ESP32 Controller:** Cắm vào **Cổng USB 2.0 (Màu đen)** (`/dev/ttyUSB0` hoặc `/dev/ttyACM0`).
* **Module GPS (Dây nhảy GPIO):**
  - `VCC` $\to$ Chân 2 hoặc 4 (5V)
  - `GND` $\to$ Chân 6 (GND)
  - `TX (GPS)` $\to$ **Chân 10 (GPIO 15 / RXD)**
  - `RX (GPS)` $\to$ **Chân 8 (GPIO 14 / TXD)**
  - Cổng đọc: **`/dev/ttyAMA0`** (Tốc độ: **`38400 baud`**).
* **Module IMU ICM-20948 (Dây nhảy GPIO I2C):**
  - `VCC` $\to$ Chân 1 (3.3V), `GND` $\to$ Chân 6 (GND)
  - `SDA` $\to$ Chân 3 (GPIO 2), `SCL` $\to$ Chân 5 (GPIO 3)
  - `CS` $\to$ **Nối lên 3.3V** (Bắt buộc để chọn I2C), `AD0` $\to$ **Nối xuống GND** (Địa chỉ `0x68`).

### 2. Cố định cổng USB (Udev Rules):
1. Tạo file udev rule:
   ```bash
   sudo nano /etc/udev/rules.d/99-robot-serial.rules
   ```
2. Dán nội dung:
   ```bash
   # RPLIDAR C1 / A1
   SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", SYMLINK+="rplidar"

   # ESP32 Motor Controller (CH343 / CH340 / CP2102)
   SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d3", MODE:="0666", SYMLINK+="esp32"
   SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0666", SYMLINK+="esp32"
   ```
3. Áp dụng rule:
   ```bash
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```

---

## 🧭 GIAI ĐOẠN 3: TÍCH HỢP GPS & IMU VÀO HỆ THỐNG (SENSOR FUSION EKF)

### 1. Kiến trúc Cây Khung Toạ Độ Chuẩn (TF Tree - REP 105):
$$\text{map} \xrightarrow{\text{EKF Global (Có GPS)}} \text{odom} \xrightarrow{\text{EKF Local (Wheel + IMU)}} \text{base\_footprint} \xrightarrow{\text{URDF}} \text{base\_link} \rightarrow \{\text{imu\_link}, \text{gps\_link}, \text{laser\_frame}\}$$

### 2. Các Topic chuẩn trong ROS 2:
| Cảm biến | Topic ROS 2 | Loại tin nhắn (Message Type) | Cổng & Tốc độ |
| :--- | :--- | :--- | :--- |
| **Bánh xe (ESP32)** | `/wheel/odom` | `nav_msgs/msg/Odometry` | `/dev/esp32` @ 115200 |
| **IMU (ICM-20948)** | `/imu/data` | `sensor_msgs/msg/Imu` | I2C `0x68` (Pin 3, 5) |
| **GPS Module** | `/gps/fix` | `sensor_msgs/msg/NavSatFix` | `/dev/ttyAMA0` @ 38400 |
| **GPS Chuyển đổi** | `/odometry/gps` | `nav_msgs/msg/Odometry` | NavSat Transform |
| **Vị trí lọc EKF** | `/odometry/filtered` | `nav_msgs/msg/Odometry` | `odom` frame |

---

## 🔍 GIAI ĐOẠN 4: KIỂM TRA TỪNG BỘ PHẬN PHẦN CỨNG (HARDWARE TEST)

Trước khi cho xe chạy tổng thể, hãy kiểm tra từng module:

### 1. Kiểm tra ESP32 & Động cơ bánh xe:
```bash
ros2 run my_robot_bringup esp32_bridge --ros-args -p serial_port:=/dev/esp32
```
*(Kiểm tra log: Thấy `Connected to ESP32 on /dev/esp32 at 115200 baud` là thành công).*

### 2. Kiểm tra Mắt quét LiDAR RPLIDAR C1:
```bash
test-lidar
```
*(LiDAR quay tròn và xuất hiện chùm tia laser 360° trên RViz).*

### 3. Kiểm tra DVD20 USB Webcam (Full 30 FPS VGA 640x480):
```bash
test-cam
```
*Hoặc lệnh chi tiết:*
```bash
ros2 launch my_sensor_test test_camera.launch.py video_device:=/dev/video0
```
*(Kiểm tra: Cửa sổ xem trực tiếp hiển thị mượt mà 30 FPS, topic `/camera/image_raw` hoặc `/camera/color/image_raw`).*

### 4. Kiểm tra Định vị GPS:
- **Cách 1: Xem dữ liệu NMEA gốc từ cổng UART GPIO:**
  ```bash
  stty -F /dev/ttyAMA0 38400 raw -echo && cat /dev/ttyAMA0
  ```
  *(Màn hình in các câu `$GNGGA...`, `$GNRMC...` liên tục).*
- **Cách 2: Khởi chạy ROS 2 GPS Driver:**
  ```bash
  test-gps
  # hoặc:
  ros2 run my_robot_controller gps_driver --ros-args -p serial_port:=/dev/ttyAMA0 -p baudrate:=38400
  ```
  *(Mở tab mới kiểm tra tọa độ: `ros2 topic echo /gps/fix`).*

### 5. Kiểm tra Cảm biến IMU 9 trục ICM-20948:
* **Kiểm tra địa chỉ I2C phần cứng:**
  ```bash
  sudo i2cdetect -y 1
  ```
  *(Thấy địa chỉ `68` xuất hiện trên bảng I2C).*

* **Khởi chạy node đọc IMU (/imu):**
  ```bash
  test-imu
  # Hoặc xem trực tiếp dữ liệu Roll, Pitch, Yaw trên topic:
  ros2 topic echo /imu
  ```

### 6. Kiểm tra tổng quát 1-Click:
```bash
test-all
```

---

## 🚗 GIAI ĐOẠN 5: CÁC KỊCH BẢN VẬN HÀNH XE THẬT

> **Mô hình kết nối:** Đặt Laptop và Raspberry Pi cùng bắt chung 1 mạng Wi-Fi (hoặc phát Wi-Fi từ điện thoại/Router).

```mermaid
graph TD
    subgraph PI_RUN["Terminal trên Pi (SSH)"]
        A["real-robot (Bật phần cứng) <br/> hoặc real-slam (Vẽ bản đồ) <br/> hoặc real-nav (Dẫn đường)"]
    end

    subgraph PC_RUN["Terminal trên Laptop / PC"]
        B["rviz (Màn hình 3D giám sát)"]
        C["wasd / teleop (Lái xe bằng bàn phím)"]
    end

    PI_RUN <==>|Wi-Fi ROS 2 DDS| PC_RUN
```

---

### 🟢 KỊCH BẢN A: Lái thử bằng bàn phím WASD (Test độ ổn định)
1. **Trên Pi (SSH):** Khởi động phần cứng xe
   ```bash
   real-robot
   ```
2. **Trên Laptop:**
   - **Terminal 1:** Mở giao diện RViz 3D:
     ```bash
     rviz
     ```
   - **Terminal 2:** Mở bàn phím lái xe:
     ```bash
     wasd
     ```
     - Nhấn `W` / `S`: Tiến / Lùi.
     - Nhấn `A` / `D`: Rẽ Trái / Phải.
     - Nhấn `Space` / `X`: Phanh dừng khẩn cấp.

---

### 🗺️ KỊCH BẢN B: Quét và dựng bản đồ thực tế (SLAM Mapping)
1. **Trên Pi (SSH):** Bật SLAM quét bản đồ
   ```bash
   real-slam
   ```
2. **Trên Laptop:**
   - Mở `rviz` để quan sát bản đồ đang vẽ trực tiếp từ LiDAR.
   - Mở `wasd` lái xe chạy từ từ quanh phòng/vườn bắp để LiDAR lấp đầy các góc khuất.
3. **Lưu bản đồ khi vẽ xong:**
   - Trên Terminal gõ:
     ```bash
     savemap ban_do_vuon_bap
     ```
   *(Bản đồ `ban_do_vuon_bap.yaml` và `ban_do_vuon_bap.pgm` được lưu vào thư mục `maps/`).*

---

### 📍 KỊCH BẢN C1: Tự hành Nav2 theo bản đồ đã lưu (AMCL Map-Based)
1. **Trên Pi (SSH):**
   ```bash
   real-nav ban_do_vuon_bap
   # Hoặc tự động nạp bản đồ mới nhất: real-nav
   # Hoặc truyền file đầy đủ: real-nav map:=/path/to/map.yaml
   ```
2. **Trên Laptop:**
   - Mở `rviz-only`.
   - Dùng công cụ **2D Pose Estimate** trên thanh menu RViz chấm vào vị trí hiện tại của xe để khởi tạo vị trí ban đầu (AMCL Localization).
   - Nhấn phím **`G`** (hoặc nút **Nav2 Goal**), click chuột vào vị trí bất kỳ trên bản đồ $\to$ Xe sẽ tự động tính đường uốn lượn né chướng ngại vật và chạy đến đích.
   - Khi cần hủy: Gõ `cancel` trên terminal hoặc bấm **Cancel Nav** trên RViz.

---

### 🌐 KỊCH BẢN C2: Vừa chạy SLAM trên Pi vừa Nav2 tự hành trên Laptop (Live SLAM-Sync Navigation)
*(Không cần nạp file map trước, tận dụng CPU của Laptop để chạy A*, Pure Pursuit và Costmaps né vật cản)*
1. **Trên Pi (SSH - 1 Terminal):**
   ```bash
   real-slam
   ```
   *(Bật LiDAR C1 + ESP32 + SLAM Toolbox vẽ map và phát tọa độ liên tục).*
2. **Trên Laptop (2 Terminal):**
   - **Terminal 1:** Mở RViz điều khiển:
     ```bash
     rviz-only
     ```
   - **Terminal 2:** Bật Nav2 Navigation Stack trên Laptop:
     ```bash
     pc-nav
     ```
   - **Cách điều khiển:** Bấm phím **`G`** (hoặc nút **Nav2 Goal**) trên RViz và click điểm đích $\to$ Thuật toán **A\*** lập đường đi, **Regulated Pure Pursuit** bám waypoint, **Costmap** 2 tầng vẽ vật cản thời gian thực và xe tự né chướng ngại vật!

---

### 🌽 KỊCH BẢN D: Tự lái bám hàng bắp bằng AI CNN & Quay đầu U-Turn
1. Đảm bảo file model ONNX đã có tại `models/crop_row_cnn_best_final.onnx`.
2. Đặt xe vào đầu luống bắp, hướng camera dọc theo rãnh giữa 2 hàng bắp.
3. **Trên Pi (SSH):** Chạy lệnh:
   ```bash
   ros2 launch my_robot_bringup real_robot.launch.py enable_camera:=true enable_cnn:=true
   ```
4. **Cơ chế hoạt động:**
   - Camera thu nhận hình ảnh $\to$ Mạng CNN phân đoạn luống bắp và tính tâm đường đi $\to$ Bộ điều khiển bám hàng PID lái xe chạy thẳng mượt mà.
   - Khi hết hàng bắp (LiDAR và Camera nhận diện vùng trống): Xe tự kích hoạt chu trình **Drive Out (2.5m)** $\to$ Tự động bẻ lái cung tròn **U-Turn 180°** sang hàng bắp tiếp theo.

---

### 📹 KỊCH BẢN E: Thu Thập Video THÔ & Tạo Dataset Train CNN

Quy trình 1 lệnh duy nhất để xe vừa chạy vừa tự động ghi hình **video thô 100% (không mở bất kỳ cửa sổ nào, không vẽ chữ hay watermark, chất lượng gốc từ sensor)**:

```mermaid
graph LR
    A["Pi: real-record luong_1 <br/> (Khởi động xe + Ghi MP4 thô trong nền)"] -->|Bấm Ctrl+C dừng xe| B["Video thô lưu tại <br/> ~/robot-ws/recordings/"]
    B -->|1 lệnh duy nhất| C["Laptop: get-video <br/> (Tự động kéo MP4 về Laptop)"]
    C --> D["Laptop: extract-dataset <br/> (Cắt ảnh tự động mỗi 0.3s)"]
    D --> E["dataset/images/ <br/> Sẵn sàng gán nhãn & train CNN"]
```

#### 1. Sự khác biệt giữa chế độ Bình Thường và Chế độ Quay Video:
* **Chế độ bình thường (`real-robot`):** Khởi động xe, chỉ truyền ảnh nén nhẹ qua Wi-Fi để xem trên RViz, **KHÔNG quay, KHÔNG lưu video** $\to$ không tốn dung lượng thẻ nhớ.
* **Chế độ quay video (`real-record`):** Khởi động xe và **TỰ ĐỘNG BẬT BỘ GHI VIDEO THÔ** chạy ngầm trong nền:
  - 100% Headless: Không mở cửa sổ giao diện nào trên màn hình.
  - 100% Raw Video: Không vẽ bất kỳ chữ/thông tin nào lên ảnh, dữ liệu sạch nguyên bản để train CNN chuẩn xác nhất.
  - Ghi chuẩn MP4 640x480 @ 30 FPS trực tiếp vào ổ cứng/thẻ nhớ của Pi.

#### 2. Thao tác trên Raspberry Pi:
Gõ đúng **1 lệnh duy nhất** trên Pi:
```bash
real-record luong_bap_1
```
*(Nếu không truyền tên file, hệ thống sẽ tự động đặt tên theo ngày giờ: `dataset_raw_YYYYMMDD_HHMMSS.mp4`).*

* Bạn mở terminal trên Laptop gõ `wasd` để lái xe chạy qua luống bắp.
* Khi xe chạy xong luống, quay lại terminal Pi bấm **`Ctrl + C`** để dừng xe và hoàn tất đóng gói file video MP4.

#### 3. Thao tác trên Laptop (Kéo video về chỉ bằng 1 lệnh):
Mở Terminal trên Laptop gõ:
```bash
get-video
# Hoặc: get-video <IP_PI> (nếu muốn chỉ định rõ IP)
```
*(Lệnh này tự động kéo toàn bộ file MP4 mới nhất từ Pi về thư mục `dataset/` trên Laptop).*

#### 4. Tách frame video thành bộ ảnh Dataset (Thực hiện trên Laptop):
Sau khi video đã về Laptop, gõ lệnh:
```bash
extract-dataset dataset/luong_bap_1.mp4 --interval 0.3
```
* Cứ mỗi `0.3 giây` lấy 1 ảnh (tự động loại bỏ ảnh trùng khi xe dừng).
* Toàn bộ ảnh được lưu vào `dataset/images/` với tên chuẩn: `crop_row_00001.jpg`, `crop_row_00002.jpg`,... sẵn sàng đưa vào Roboflow/LabelImg để gán nhãn!

---

## 🛠️ BẢNG TRA CỨU SỰ CỐ THƯỜNG GẶP & CÁCH KHẮC PHỤC

| Hiện tượng | Nguyên nhân | Cách khắc phục xử lý nhanh |
| :--- | :--- | :--- |
| **`Permission Denied /dev/ttyUSB*`** | Chưa cấp quyền người dùng vào nhóm cổng Serial. | Chạy `sudo usermod -a -G dialout $USER && newgrp dialout` hoặc `sudo chmod 666 /dev/ttyUSB*`. |
| **Không thấy topic giữa Pi và PC** | Khác mạng Wi-Fi hoặc lệch biến `ROS_DOMAIN_ID`. | Đảm bảo 2 máy kết nối chung Wi-Fi và đặt cùng `export ROS_DOMAIN_ID=0`. |
| **Bản đồ RViz bị giật/nhấp nháy** | Hai node cùng phát TF `odom -> base_footprint`. | Kiểm tra `esp32_bridge`: Đảm bảo đã đặt `publish_tf: false` khi sử dụng EKF. |
| **Xe không nhận lệnh `wasd`** | Chuột chưa click vào cửa sổ Terminal `wasd`. | Click chuột vào trong cửa sổ Terminal chạy `wasd` rồi mới bấm phím. |
| **GPS báo `NO_FIX`** | Ăng-ten GPS bị che khuất trong nhà. | Đưa xe/ăng-ten ra không gian thoáng ngoài trời 1-2 phút để bắt đủ $> 6$ vệ tinh. |
| **Động cơ quay ngược hướng** | Đấu nhầm dây motor hoặc đảo vế trái/phải. | Đổi lại chân dây tín hiệu PWM hoặc chỉnh dấu âm/dương trong `esp32_bridge.py`. |
