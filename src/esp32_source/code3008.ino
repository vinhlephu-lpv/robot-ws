#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

// ============================================================
//  🤖 FIRMWARE ĐIỀU KHIỂN ROBOT AMR 4 BÁNH VI SAI (ESP32-S3)
//  Tối ưu 100% cho ROS 2 Jazzy, Nav2, SLAM Toolbox, Madgwick & EKF
// ============================================================

// ============================================================
//  1. CẤU HÌNH PHẦN CỨNG & THÔNG SỐ VẬT LÝ XE
// ============================================================
#define ENCODER_PPR             200       // Số xung/vòng trục bánh xe (sau giảm tốc)
#define WHEEL_DIAMETER_M        0.20f     // Đường kính bánh xe: 200mm (0.2m)
#define WHEEL_CIRCUMFERENCE     (PI * WHEEL_DIAMETER_M)  // Chu vi bánh ≈ 0.6283m
#define MAX_RPM_PHYSICAL        220.0f    // Tốc độ vòng quay tối đa không tải (RPM)

// Chu kỳ thời gian các vòng lặp
#define SPEED_CALC_INTERVAL_MS  50        // Tính toán vận tốc & PID mỗi 50ms (20 Hz)
#define RAMP_INTERVAL_MS        25        // Khởi động mềm Slew Rate mỗi 25ms (40 Hz)
#define SERIAL_FEEDBACK_MS      50        // Gửi Odometry lên Pi mỗi 50ms (20 Hz)

// Ngưỡng Deadzone & Slew Rate bảo vệ động cơ 775 & Hộp số
#define MIN_PWM                 85        // Ngưỡng vượt vùng chết ma sát motor 775 24V
#define RAMP_STEP_ACCEL         12.0f     // Bước tăng tốc PWM tối đa mỗi chu kỳ 25ms
#define RAMP_STEP_DECEL         18.0f     // Bước giảm tốc PWM mượt mà bảo vệ cơ cấu nhông

// Lọc nhiễu xung Encoder chống gai tia lửa chổi than motor 775
#define MIN_ENC_INTERVAL_US     200       // Bỏ qua xung nhiễu ngắn hơn 200us
#define MOVING_AVG_SIZE         4         // Kích thước bộ đệm trung bình trượt

// Thời gian Watchdog an toàn (ngắt motor nếu Pi mất kết nối)
#define ROS_WATCHDOG_TIMEOUT_MS 600       // 600ms không có lệnh từ Pi -> Tự phanh dừng

// ============================================================
//  2. SƠ ĐỒ CHÂN GPIO BTS7960 (4 DRIVER ĐỘC LẬP)
// ============================================================
#define DRV1_RPWM   47   // Bánh 0: Trái trước (FL)
#define DRV1_LPWM   4
#define DRV2_RPWM   45   // Bánh 1: Trái sau (RL)
#define DRV2_LPWM   18
#define DRV3_RPWM   13   // Bánh 2: Phải trước (FR)
#define DRV3_LPWM   15
#define DRV4_RPWM   7    // Bánh 3: Phải sau (RR)
#define DRV4_LPWM   8

// Chiều quay chuẩn cho từng động cơ (+1 bình thường, -1 đảo chiều)
#define INV_DRV1     1
#define INV_DRV2     1
#define INV_DRV3     1
#define INV_DRV4    -1

#define RGB_LED_PIN 48   // LED RGB trên bo mạch ESP32-S3

// Cấu hình PWM Timer
const int PWM_FREQ = 7000;
const int PWM_RES  = 8;      // 8-bit: 0 - 255
const int CH_DRV1_F = 0, CH_DRV1_R = 1;
const int CH_DRV2_F = 2, CH_DRV2_R = 3;
const int CH_DRV3_F = 4, CH_DRV3_R = 5;
const int CH_DRV4_F = 6, CH_DRV4_R = 7;

// ============================================================
//  3. CẤU HÌNH WIFI & WEBSERVER (NỀN TẢNG GIÁM SÁT)
// ============================================================
const char* ssid     = "CTU";
const char* password = "";
WebServer server(80);

// ============================================================
//  4. CẤU TRÚC DỮ LIỆU ENCODER, KALMAN 1D & PID
// ============================================================
struct EncoderData {
  int   pinA, pinB;
  int   sign;                       // +1 bình thường, -1 đảo chiều xung
  volatile long count;              // Tổng số xung tích lũy
  long  lastSpeedCount;             // Số xung tại chu kỳ trước
  float rpm;                        // Tốc độ vòng quay thực tế sau lọc (RPM)
  float prevFilteredRpm;
  float speed_ms;                   // Vận tốc bánh xe (m/s)
  float distance_m;                 // Quãng đường lăn bánh (m)
  float rpmBuffer[MOVING_AVG_SIZE];
  int   bufferIndex;
  float rawRpmHist[2];              // Lịch sử 2 mẫu cho bộ lọc Median-3
};

// Khai báo 4 kênh Encoder: FL(16,17), RL(38,39), FR(40,41), RR(10,11)
EncoderData enc[4] = {
  {16, 17, -1, 0, 0, 0, 0, 0, 0, {0}, 0, {0, 0}}, // Bánh 0: Trái trước
  {38, 39, -1, 0, 0, 0, 0, 0, 0, {0}, 0, {0, 0}}, // Bánh 1: Trái sau
  {40, 41,  1, 0, 0, 0, 0, 0, 0, {0}, 0, {0, 0}}, // Bánh 2: Phải trước
  {10, 11,  1, 0, 0, 0, 0, 0, 0, {0}, 0, {0, 0}}  // Bánh 3: Phải sau
};

// Cấu trúc Bộ Lọc Kalman 1D lọc sạch nhiễu đo lường cho từng bánh xe
struct Kalman1D {
  float x; // Ước lượng vận tốc RPM
  float p; // Sai số ước lượng
  float q; // Độ bất định mô hình
  float r; // Độ nhiễu đo lường
  float k; // Hệ số Kalman gain
};

Kalman1D kfRpm[4] = {
  {0.0f, 1.0f, 0.10f, 5.0f, 0.0f},
  {0.0f, 1.0f, 0.10f, 5.0f, 0.0f},
  {0.0f, 1.0f, 0.10f, 5.0f, 0.0f},
  {0.0f, 1.0f, 0.10f, 5.0f, 0.0f}
};

float updateKalman1D(Kalman1D &kf, float measurement) {
  kf.p = kf.p + kf.q;
  kf.k = kf.p / (kf.p + kf.r);
  kf.x = kf.x + kf.k * (measurement - kf.x);
  kf.p = (1.0f - kf.k) * kf.p;
  return kf.x;
}

// Cấu trúc Bộ điều khiển PID bám tốc độ
struct WheelPID {
  float kp, ki, kd;
  float targetRPM;                  // Tốc độ mục tiêu (luôn >= 0)
  float integral;                   // Khâu tích phân
  float lastError;                  // Sai số chu kỳ trước
  float filteredDeriv;              // Khâu vi phân đã lọc
  int   pwmOutput;                  // Giá trị PWM tính toán (0 - 255)
  bool  enabled;                    // Cờ bật/tắt PID
};

// Bộ thông số PID tối ưu cho xe 4 bánh động cơ 775 tải nặng
#define PID_KP              1.100f
#define PID_KI              0.800f
#define PID_KD              0.060f

WheelPID wpid[4] = {
  {PID_KP, PID_KI, PID_KD, 0, 0, 0, 0, 0, true},
  {PID_KP, PID_KI, PID_KD, 0, 0, 0, 0, 0, true},
  {PID_KP, PID_KI, PID_KD, 0, 0, 0, 0, 0, true},
  {PID_KP, PID_KI, PID_KD, 0, 0, 0, 0, 0, true}
};

// Cấu trúc Slew Rate (Khởi động mềm chống sốc cơ khí)
struct SlewChannel {
  int   target;                     // PWM mục tiêu có dấu (-255 đến +255)
  float current;                    // PWM xuất hiện tại sau khi làm mịn
};
SlewChannel slew[4] = {};

// Biến trạng thái toàn cục
bool          pidGlobalEnabled      = true;     // Mặc định BẬT PID bám tốc độ
bool          isMoving              = false;    // Xe đang nhận lệnh di chuyển
String        currentDirection      = "STOP";
float         rosTargetRpmSigned[4] = {0.0f, 0.0f, 0.0f, 0.0f};
unsigned long lastRosCmdTime        = 0;
unsigned long lastSpeedCalcTime     = 0;
unsigned long lastRampTime          = 0;
volatile unsigned long lastEncTime[4] = {0, 0, 0, 0};

// ============================================================
//  5. CÁC HÀM NGẮT ENCODER TỐC ĐỘ CAO (IRAM ISR)
// ============================================================
void IRAM_ATTR isr_enc0() {
  unsigned long now = micros();
  if (now - lastEncTime[0] < MIN_ENC_INTERVAL_US) return;
  if (digitalRead(enc[0].pinA) == LOW) return;
  int b1 = digitalRead(enc[0].pinB);
  lastEncTime[0] = now;
  enc[0].count += (b1 > 0) ? enc[0].sign : -enc[0].sign;
}

void IRAM_ATTR isr_enc1() {
  unsigned long now = micros();
  if (now - lastEncTime[1] < MIN_ENC_INTERVAL_US) return;
  if (digitalRead(enc[1].pinA) == LOW) return;
  int b1 = digitalRead(enc[1].pinB);
  lastEncTime[1] = now;
  enc[1].count += (b1 > 0) ? enc[1].sign : -enc[1].sign;
}

void IRAM_ATTR isr_enc2() {
  unsigned long now = micros();
  if (now - lastEncTime[2] < MIN_ENC_INTERVAL_US) return;
  if (digitalRead(enc[2].pinA) == LOW) return;
  int b1 = digitalRead(enc[2].pinB);
  lastEncTime[2] = now;
  enc[2].count += (b1 > 0) ? enc[2].sign : -enc[2].sign;
}

void IRAM_ATTR isr_enc3() {
  unsigned long now = micros();
  if (now - lastEncTime[3] < MIN_ENC_INTERVAL_US) return;
  if (digitalRead(enc[3].pinA) == LOW) return;
  int b1 = digitalRead(enc[3].pinB);
  lastEncTime[3] = now;
  enc[3].count += (b1 > 0) ? enc[3].sign : -enc[3].sign;
}

// ============================================================
//  6. ĐIỀU KHIỂN PHẦN CỨNG DRIVER BTS7960
// ============================================================
void pwmSetup(int pin, int channel) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcAttach(pin, PWM_FREQ, PWM_RES);
  ledcWrite(pin, 0);
#else
  ledcSetup(channel, PWM_FREQ, PWM_RES);
  ledcAttachPin(pin, channel);
  ledcWrite(channel, 0);
#endif
}

void pwmWrite(int pin, int channel, int value) {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcWrite(pin, value);
#else
  ledcWrite(channel, value);
#endif
}

void writeSingleDrive(int rpwmPin, int lpwmPin, int chF, int chR, int speed, int inv) {
  speed = speed * inv;
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    pwmWrite(rpwmPin, chF, speed);
    pwmWrite(lpwmPin, chR, 0);
  } else if (speed < 0) {
    pwmWrite(rpwmPin, chF, 0);
    pwmWrite(lpwmPin, chR, -speed);
  } else {
    pwmWrite(rpwmPin, chF, 0);
    pwmWrite(lpwmPin, chR, 0);
  }
}

void writeAllDrives(int s0, int s1, int s2, int s3) {
  writeSingleDrive(DRV1_RPWM, DRV1_LPWM, CH_DRV1_F, CH_DRV1_R, s0, INV_DRV1);
  writeSingleDrive(DRV2_RPWM, DRV2_LPWM, CH_DRV2_F, CH_DRV2_R, s1, INV_DRV2);
  writeSingleDrive(DRV3_RPWM, DRV3_LPWM, CH_DRV3_F, CH_DRV3_R, s2, INV_DRV3);
  writeSingleDrive(DRV4_RPWM, DRV4_LPWM, CH_DRV4_F, CH_DRV4_R, s3, INV_DRV4);
}

void stopMotor(bool emergency) {
  isMoving = false;
  currentDirection = "STOP";
  for (int i = 0; i < 4; i++) {
    rosTargetRpmSigned[i] = 0.0f;
    wpid[i].targetRPM = 0.0f;
    wpid[i].integral = 0.0f;
    wpid[i].lastError = 0.0f;
    wpid[i].pwmOutput = 0;
    slew[i].target = 0;
    if (emergency) slew[i].current = 0.0f;
  }
  if (emergency) writeAllDrives(0, 0, 0, 0);
}

// ============================================================
//  7. TÍNH VẬN TỐC THỰC TẾ & BỘ LỌC NOISE ENCODER (20 Hz)
// ============================================================
inline float median3(float a, float b, float c) {
  if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
  if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
  return c;
}

float calculateMovingAverage(float buffer[], int size) {
  float sum = 0.0f;
  for (int i = 0; i < size; i++) sum += buffer[i];
  return sum / size;
}

void updatePID(float dt);

void calculateSpeed() {
  unsigned long now = millis();
  if (now - lastSpeedCalcTime < SPEED_CALC_INTERVAL_MS) return;
  float dt = (now - lastSpeedCalcTime) / 1000.0f;
  lastSpeedCalcTime = now;

  if (dt <= 0.0f) return;

  for (int i = 0; i < 4; i++) {
    noInterrupts();
    long curCount = enc[i].count;
    interrupts();

    long pulses = curCount - enc[i].lastSpeedCount;
    enc[i].lastSpeedCount = curCount;

    // Tính tốc độ RPM thô
    float rawRPM = ((float)pulses / (float)ENCODER_PPR) * (60.0f / dt);
    rawRPM = constrain(rawRPM, -MAX_RPM_PHYSICAL, MAX_RPM_PHYSICAL);

    // Bộ lọc Trung vị 3 điểm (Median-3)
    float medRPM = median3(rawRPM, enc[i].rawRpmHist[0], enc[i].rawRpmHist[1]);
    enc[i].rawRpmHist[1] = enc[i].rawRpmHist[0];
    enc[i].rawRpmHist[0] = rawRPM;

    // Bộ lọc Trung bình trượt + EMA
    enc[i].rpmBuffer[enc[i].bufferIndex] = medRPM;
    enc[i].bufferIndex = (enc[i].bufferIndex + 1) % MOVING_AVG_SIZE;
    float avgRpm = calculateMovingAverage(enc[i].rpmBuffer, MOVING_AVG_SIZE);
    float emaRpm = 0.60f * avgRpm + 0.40f * enc[i].prevFilteredRpm;
    enc[i].prevFilteredRpm = emaRpm;

    // Bộ lọc Kalman 1D
    enc[i].rpm = updateKalman1D(kfRpm[i], emaRpm);
    enc[i].speed_ms = enc[i].rpm * WHEEL_CIRCUMFERENCE / 60.0f;
    enc[i].distance_m += (labs(pulses) * WHEEL_CIRCUMFERENCE) / (float)ENCODER_PPR;
  }

  // Gửi Odometry phản hồi lên ROS 2 (20 Hz)
  float v_left  = (enc[0].speed_ms + enc[1].speed_ms) / 2.0f;
  float v_right = (enc[2].speed_ms + enc[3].speed_ms) / 2.0f;
  unsigned int dt_ms = (unsigned int)(dt * 1000.0f + 0.5f);

  // 1. Gói tin ODOM (m/s)
  Serial.printf("ODOM %.3f %.3f\n", v_left, v_right);

  // 2. Gói tin ENC (tick counts)
  Serial.printf("ENC %ld %ld %ld %ld %u\n", 
                enc[0].count, enc[1].count, enc[2].count, enc[3].count, dt_ms);

  // Cập nhật thuật toán điều khiển PID
  updatePID(dt);
}

// ============================================================
//  8. THUẬT TOÁN ĐIỀU KHIỂN TỐC ĐỘ PID + FEEDFORWARD (20 Hz)
// ============================================================
void updatePID(float dt) {
  if (!isMoving || dt <= 0.0f) return;

  for (int i = 0; i < 4; i++) {
    float target = wpid[i].targetRPM;
    float actual = fabsf(enc[i].rpm);
    int   sign   = (rosTargetRpmSigned[i] >= 0.0f) ? 1 : -1;

    if (target <= 0.1f) {
      wpid[i].pwmOutput = 0;
      wpid[i].integral = 0.0f;
      slew[i].target = 0;
      continue;
    }

    if (!pidGlobalEnabled || !wpid[i].enabled) {
      // Chế độ PWM hở (Open Loop có bù Deadzone)
      int openPwm = (int)(MIN_PWM + (target / MAX_RPM_PHYSICAL) * (255 - MIN_PWM));
      slew[i].target = sign * constrain(openPwm, 0, 255);
      continue;
    }

    // 1. Feedforward cơ sở (Bù deadzone chính xác cho motor 775)
    float ff_pwm = MIN_PWM + (target / MAX_RPM_PHYSICAL) * (255.0f - MIN_PWM);

    // 2. Tính toán sai số bám tốc độ
    float error = target - actual;

    // 3. Khâu tích phân (Anti-Windup)
    wpid[i].integral = constrain(wpid[i].integral + error * dt, -45.0f, 45.0f);

    // 4. Khâu vi phân (có lọc thông thấp)
    float rawDeriv = (error - wpid[i].lastError) / dt;
    wpid[i].filteredDeriv = 0.60f * rawDeriv + 0.40f * wpid[i].filteredDeriv;
    wpid[i].lastError = error;

    // 5. Tính toán phản hồi PID
    float pid_adj = (wpid[i].kp * error) + (wpid[i].ki * wpid[i].integral) + (wpid[i].kd * wpid[i].filteredDeriv);

    // 6. Tổng hợp PWM hoàn chỉnh
    int totalPwm = constrain((int)(ff_pwm + pid_adj), MIN_PWM, 255);
    wpid[i].pwmOutput = totalPwm;

    // Đặt mục tiêu cho khâu làm mịn Slew Rate
    slew[i].target = sign * totalPwm;
  }
}

// ============================================================
//  9. SLEW RATE LIMITER — KHỞI ĐỘNG MỀM BẢO VỆ NHÔNG (40 Hz)
// ============================================================
void updateSpeedRamp() {
  unsigned long now = millis();
  if (now - lastRampTime < RAMP_INTERVAL_MS) return;
  lastRampTime = now;

  for (int i = 0; i < 4; i++) {
    float &cur = slew[i].current;
    int   tgt  = slew[i].target;

    if (cur < tgt) {
      cur += RAMP_STEP_ACCEL;
      if (cur > tgt) cur = tgt;
    } else if (cur > tgt) {
      cur -= RAMP_STEP_DECEL;
      if (cur < tgt) cur = tgt;
    }
  }

  // Xuất PWM vật lý ra 4 driver BTS7960
  writeAllDrives((int)slew[0].current, (int)slew[1].current,
                 (int)slew[2].current, (int)slew[3].current);
}

// ============================================================
//  10. GIẢI MÃ LỆNH TỪ ROS 2 QUA SERIAL
// ============================================================
void handleCommand(String command) {
  command.trim();
  if (command.length() == 0) return;

  // Lệnh vận tốc ROS 2: "V <FL> <RL> <FR> <RR>" hoặc "V <L> <R>" (RPM)
  if (command.startsWith("V ") || command.startsWith("v ") || command.startsWith("V\t") || command.startsWith("v\t")) {
    float r[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    int parsed = sscanf(command.c_str() + 2, "%f %f %f %f", &r[0], &r[1], &r[2], &r[3]);
    if (parsed == 2) {
      r[3] = r[1]; // Phải sau = Phải
      r[2] = r[1]; // Phải trước = Phải
      r[1] = r[0]; // Trái sau = Trái
      parsed = 4;
    }
    if (parsed == 4) {
      lastRosCmdTime = millis();
      if (fabsf(r[0]) < 0.1f && fabsf(r[1]) < 0.1f && fabsf(r[2]) < 0.1f && fabsf(r[3]) < 0.1f) {
        stopMotor(false);
      } else {
        isMoving = true;
        currentDirection = "ROS";
        for (int i = 0; i < 4; i++) {
          rosTargetRpmSigned[i] = r[i];
          wpid[i].targetRPM = fabsf(r[i]);
          wpid[i].enabled = pidGlobalEnabled;
        }
      }
      return;
    }
  }

  command.toUpperCase();

  // Lệnh dừng xe
  if (command == "STOP" || command == "S" || command == "X") {
    stopMotor(false);
    Serial.println("[ROBOT] Da dung xe.");
    return;
  }

  // Lệnh dừng khẩn cấp
  if (command == "ESTOP" || command == "EMERGENCY") {
    stopMotor(true);
    Serial.println("[ROBOT] !!! DUNG KHAN CAP ESTOP !!!");
    return;
  }

  // Lệnh Reset Odometry
  if (command == "RESET_ODOM" || command == "ZERO" || command == "RESET") {
    for (int i = 0; i < 4; i++) {
      enc[i].count = 0;
      enc[i].lastSpeedCount = 0;
      enc[i].distance_m = 0.0f;
    }
    Serial.println("[ODOM] Da reset quang duong 4 banh ve 0.");
    return;
  }

  // Lệnh bật/tắt PID
  if (command == "PID 0" || command == "PID OFF" || command == "PID=0") {
    pidGlobalEnabled = false;
    for (int i = 0; i < 4; i++) wpid[i].enabled = false;
    Serial.println("[MODE] Che do PWM truc tiep (Khong dung PID).");
    return;
  }
  if (command == "PID 1" || command == "PID ON" || command == "PID=1") {
    pidGlobalEnabled = true;
    for (int i = 0; i < 4; i++) wpid[i].enabled = true;
    Serial.println("[MODE] Che do PID bám toc do dong bo 4 banh.");
    return;
  }
}

// ============================================================
//  11. WEBSERVER CƠ BẢN GIÁM SÁT QUA TRÌNH DUYỆT
// ============================================================
void handleRoot() {
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>Robot Status</title></head>";
  html += "<body style='font-family:sans-serif; text-align:center; padding:20px;'>";
  html += "<h2>🤖 Robot AMR ESP32 Status</h2>";
  html += "<p>Mode: <b>" + currentDirection + "</b></p>";
  html += "<p>PID: <b>" + String(pidGlobalEnabled ? "ON" : "OFF") + "</b></p>";
  html += "<p>Encoders (Xung): " + String(enc[0].count) + " | " + String(enc[1].count) + " | " + String(enc[2].count) + " | " + String(enc[3].count) + "</p>";
  html += "<p>RPM: " + String(enc[0].rpm, 1) + " | " + String(enc[1].rpm, 1) + " | " + String(enc[2].rpm, 1) + " | " + String(enc[3].rpm, 1) + "</p>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

// ============================================================
//  12. HÀM SETUP & VÒNG LẶP CHÍNH (MAIN LOOP)
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  // 1. Khởi tạo chân ngắt Encoder
  for (int i = 0; i < 4; i++) {
    pinMode(enc[i].pinA, INPUT_PULLUP);
    pinMode(enc[i].pinB, INPUT_PULLUP);
  }
  attachInterrupt(digitalPinToInterrupt(enc[0].pinA), isr_enc0, RISING);
  attachInterrupt(digitalPinToInterrupt(enc[1].pinA), isr_enc1, RISING);
  attachInterrupt(digitalPinToInterrupt(enc[2].pinA), isr_enc2, RISING);
  attachInterrupt(digitalPinToInterrupt(enc[3].pinA), isr_enc3, RISING);

  // 2. Khởi tạo PWM 4 kênh Driver BTS7960
  pwmSetup(DRV1_RPWM, CH_DRV1_F);
  pwmSetup(DRV1_LPWM, CH_DRV1_R);
  pwmSetup(DRV2_RPWM, CH_DRV2_F);
  pwmSetup(DRV2_LPWM, CH_DRV2_R);
  pwmSetup(DRV3_RPWM, CH_DRV3_F);
  pwmSetup(DRV3_LPWM, CH_DRV3_R);
  pwmSetup(DRV4_RPWM, CH_DRV4_F);
  pwmSetup(DRV4_LPWM, CH_DRV4_R);

  writeAllDrives(0, 0, 0, 0);

  // 3. Khởi tạo WiFi Station (nền)
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  server.on("/", handleRoot);
  server.begin();

  Serial.println("\n[ESP32] READY - Firmware AMR 4-Wheel Differential Drive v3.0");
}

void loop() {
  // 1. Đọc dữ liệu lệnh từ Serial
  while (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    handleCommand(cmd);
  }

  // 2. Khởi động mềm Slew Rate (40 Hz)
  updateSpeedRamp();

  // 3. An toàn ROS Watchdog (600ms không có lệnh từ Pi -> Tự động dừng xe)
  if (currentDirection == "ROS" && isMoving) {
    if (millis() - lastRosCmdTime > ROS_WATCHDOG_TIMEOUT_MS) {
      stopMotor(false);
    }
  }

  // 4. Tính toán vận tốc & phản hồi Odometry lên ROS 2 (20 Hz)
  calculateSpeed();

  // 5. Xử lý WebServer giám sát
  server.handleClient();

  delay(1);
}
