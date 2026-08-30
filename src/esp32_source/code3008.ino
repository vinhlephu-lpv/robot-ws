#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

// ============================================================
//  1. CẤU HÌNH PHẦN CỨNG & THÔNG SỐ XE
// ============================================================
#define ENCODER_PPR             200       // Xung/vòng trục bánh xe
#define WHEEL_DIAMETER_M        0.20f     // Đường kính bánh xe: 200mm (0.2m)
#define WHEEL_CIRCUMFERENCE     (PI * WHEEL_DIAMETER_M)  // Chu vi bánh ≈ 0.6283m

// Khoảng thời gian chu kỳ tính toán
#define SPEED_CALC_INTERVAL_MS  50        // Tính tốc độ mỗi 50ms (20 Hz)
#define RAMP_INTERVAL_MS        25        // Khởi động mềm mỗi 25ms (40 Hz)
#define DEBUG_PRINT_INTERVAL_MS 500       // In debug Serial mỗi 500ms

// Ngưỡng Deadzone & Giới hạn gia tốc PWM (Tối ưu lực kéo vượt địa hình với nguồn xả cao 90A)
#define MIN_PWM                 95        // Tăng từ 75 lên 95: Vượt triệt để deadzone motor 775 24V có tải nặng
#define MAX_PWM_CHANGE_UP       55        // Tăng từ 35 lên 55: Bơm lực cực nhanh khi khởi động và leo cản dốc
#define MAX_PWM_CHANGE_DOWN     25        // Giảm tốc mượt mà bảo vệ cơ cấu nhông
#define RAMP_STEP_MAX           12.0f     // Tăng từ 8.0f lên 12.0f: Bơm gia tốc nhanh
#define RAMP_STEP_MIN           2.5f      // Tăng từ 0.5f lên 2.5f: Bước khởi động dứt khoát không bị ì
#define RAMP_STEP_STOP_MAX      5.0f      // Bước giảm tốc êm ái bảo vệ hộp số

// Lọc nhiễu Encoder & Hiệu chuẩn khoảng cách
#define MIN_ENC_INTERVAL_US     350       // Tăng từ 200 lên 350us: Chặn đứng 100% gai nhiễu điện từ tia lửa chổi than 775
#define MAX_PLAUSIBLE_RPM       350.0f    // Giới hạn vật lý lọc đột biến RPM
#define MOVING_AVG_SIZE         4         // Số mẫu trung bình trượt RPM
#define DERIVATIVE_FILTER       0.6f      // Hệ số lọc thông thấp vi phân PID
#define DEFAULT_CALIB_SCALE     1.0f      // Hệ số hiệu chuẩn quãng đường thực tế
#define DECEL_DIST_THRESHOLD    0.75f     // Khoảng cách bắt đầu giảm tốc êm trước đích: 75cm (0.75m)
#define DEFAULT_CRAWL_RPM       25.0f     // Tốc độ chạy bò chuẩn xác khi gần đích (RPM)

// 3 Bộ lọc nâng cao: Khử rung tốc độ thấp, Khóa hướng thẳng và Vùng chết PID
#define EMA_LOW_SPEED_ALPHA     0.35f     // Hệ số làm mịn EMA tốc độ thấp (15-30 RPM) triệt tiêu tiếng rung
#define EMA_HIGH_SPEED_ALPHA    0.70f     // Hệ số EMA tốc độ cao (đáp ứng tức thì)
#define K_HEADING_LOCK          35.0f     // Hệ số khóa hướng vi sai (chống xẹo xe khi bánh trượt)
#define PID_ERROR_DEADBAND      0.8f      // Vùng chết sai số PID (giúp động cơ êm và mát driver)

// Phát hiện kẹt bánh (Stall Detection) - Cho phép 1.2s phát huy 100% momen xoắn vượt gờ trước khi ngắt nhiệt
#define STALL_DETECT_MS         1200
#define STALL_PWM_THRESHOLD     180
#define STALL_RPM_THRESHOLD     4.0f

// Chế độ dự phòng nếu có 1 encoder bị hỏng (false: đủ 4 encoder)
#define WHEEL3_ENCODER_FAULT    false

// ============================================================
//  2. SƠ ĐỒ CHÂN GPIO BTS7960 (4 DRIVER ĐỘC LẬP)
// ============================================================
#define DRV1_RPWM   47   // Bánh 0: Trái trước
#define DRV1_LPWM   4
#define DRV2_RPWM   45   // Bánh 1: Trái sau
#define DRV2_LPWM   18
#define DRV3_RPWM   13   // Bánh 2: Phải trước
#define DRV3_LPWM   15
#define DRV4_RPWM   7   // Bánh 3: Phải sau
#define DRV4_LPWM   8

// Cờ đảo chiều driver (+1: bình thường, -1: đảo chiều nếu đấu ngược dây)
#define INV_DRV1     1
#define INV_DRV2     1
#define INV_DRV3     1
#define INV_DRV4    -1

#define RGB_LED_PIN 48   // Chân LED RGB tích hợp ESP32-S3

// ============================================================
//  3. CẤU HÌNH WIFI & WEBSERVER
// ============================================================
const char* ssid     = "CTU";
const char* password = "";
WebServer server(80);

const int PWM_FREQ = 7000;
const int PWM_RES  = 8;      // 8-bit: 0 - 255
const int CH_DRV1_F = 0, CH_DRV1_R = 1;
const int CH_DRV2_F = 2, CH_DRV2_R = 3;
const int CH_DRV3_F = 4, CH_DRV3_R = 5;
const int CH_DRV4_F = 6, CH_DRV4_R = 7;

// ============================================================
//  4. CẤU TRÚC DỮ LIỆU ENCODER, PID & SLEW RATE
// ============================================================
struct EncoderData {
  int   pinA, pinB;
  int   sign;                       // +1 bình thường, -1 đảo dấu đếm xung
  volatile long count;
  long  lastSpeedCount;
  float rpm;
  float prevFilteredRpm;
  float speed_ms;
  float distance_m;
  float rpmBuffer[MOVING_AVG_SIZE];
  int   bufferIndex;
  float rawRpmHist[2];              // Bộ nhớ 2 chu kỳ quá khứ cho Bộ lọc Trung vị 3 điểm (Median-3 Filter)
};

// Sơ đồ 4 Encoder: Trái trước (16,17), Trái sau (38,39), Phải trước (40,41), Phải sau (10,11)
EncoderData enc[4] = {
  {16, 17, -1, 0, 0, 0, 0, 0, 0, {0}, 0, {0, 0}}, // enc[0]: Bánh trái trước (đảo dấu đếm dương khi tiến)
  {38, 39, -1, 0, 0, 0, 0, 0, 0, {0}, 0, {0, 0}}, // enc[1]: Bánh trái sau (đảo dấu đếm dương khi tiến)
  {40, 41,  1, 0, 0, 0, 0, 0, 0, {0}, 0, {0, 0}}, // enc[2]: Bánh phải trước (đếm dương khi tiến)
  {10, 11,  1, 0, 0, 0, 0, 0, 0, {0}, 0, {0, 0}}  // enc[3]: Bánh phải sau (đếm dương khi tiến)
};

struct WheelPID {
  float kp, ki, kd;
  float targetRPM;
  float integral;
  float lastError;
  float filteredDeriv;
  int   pwmOutput;
  int   prevPwmOutput;
  bool  enabled;
};

// Hệ số PID tối ưu đồng bộ 4 bánh: Cân bằng vàng (Êm ái, bám tốc chính xác, lực kéo cực khỏe)
#define PID_KP              1.050f
#define PID_KI              0.900f
#define PID_KD              0.080f
#define K_SYNC_CROSS_WHEEL  0.850f  // Hệ số bù đồng tốc liên bánh xe
#define K_LR_BALANCE        1.500f  // Hệ số khóa cân bằng đồng tốc cụm Trái - Phải

WheelPID wpid[4] = {
  {PID_KP, PID_KI, PID_KD, 0, 0, 0, 0, 0, 0, true}, // Bánh 1: Trái trước
  {PID_KP, PID_KI, PID_KD, 0, 0, 0, 0, 0, 0, true}, // Bánh 2: Trái sau
  {PID_KP, PID_KI, PID_KD, 0, 0, 0, 0, 0, 0, true}, // Bánh 3: Phải trước
  {PID_KP, PID_KI, PID_KD, 0, 0, 0, 0, 0, 0, true}  // Bánh 4: Phải sau
};

// Cấu trúc Bộ Lọc Kalman 1D cho từng bánh xe (Tách nhiễu Gaussian và triệt tiêu trễ pha)
struct Kalman1D {
  float x; // Ước lượng vận tốc RPM
  float p; // Sai số ước lượng
  float q; // Độ bất định mô hình vật lý
  float r; // Độ nhiễu đo lường encoder
  float k; // Hệ số tăng Kalman
};

Kalman1D kfRpm[4] = {
  {0.0f, 1.0f, 0.10f, 6.0f, 0.0f}, // Bánh 1: Tăng độ tin cậy mô hình, triệt tiêu hoàn toàn gai nhiễu encoder
  {0.0f, 1.0f, 0.10f, 6.0f, 0.0f}, // Bánh 2
  {0.0f, 1.0f, 0.10f, 6.0f, 0.0f}, // Bánh 3
  {0.0f, 1.0f, 0.10f, 6.0f, 0.0f}  // Bánh 4
};

float updateKalman1D(Kalman1D &kf, float measurement) {
  kf.p = kf.p + kf.q;
  kf.k = kf.p / (kf.p + kf.r);
  kf.x = kf.x + kf.k * (measurement - kf.x);
  kf.p = (1.0f - kf.k) * kf.p;
  return kf.x;
}

#define K_COULOMB_FRICTION      25.0f     // Lực bù ma sát khô nhông hộp số cực đại (PWM)
#define K_POSITION_ANCHOR       1.2f      // Hệ số neo giữ vị trí vạch đích khi dừng (PWM / xung)

struct SlewChannel {
  int   target;                     // Tốc độ mục tiêu (-255 đến +255)
  float current;                    // Tốc độ hiện tại mượt mà
  float step;                       // Bước ramp động
};

SlewChannel slew[4] = {};

struct WheelHealth {
  bool isFaulty;                    // Đang bị sự cố (kẹt cứng hoặc hẫng lên không trung)
  bool isStalled;                   // Bị kẹt cứng cơ khí / hỏng encoder / hỏng motor
  bool isHanging;                   // Bị hẫng lên không trung (mất ma sát / trượt quay tự do)
  unsigned long stallStartTime;
  unsigned long hangStartTime;
};
WheelHealth wHealth[4] = {};

// Biến điều khiển xe & Quãng đường chính xác
int           currentSpeed            = 200;
bool          isMoving                = false;
bool          manualDriveActive       = false;
bool          pidGlobalEnabled        = true;         // Mặc định BẬT PID mô-men xoắn cao cho cả 4 bánh
float         globalTargetRPM         = 60.0f;        // Tốc độ mục tiêu khởi đầu (chạy chậm lực kéo lớn)
String        currentDirection        = "STOP";

// Điều khiển quãng đường, hiệu chuẩn và Khóa Neo Tọa Độ
bool          isDistanceMode          = false;
float         targetDistMeters        = 0.0f;
float         startDistMeters[4]      = {0, 0, 0, 0};
float         distCalibScale          = DEFAULT_CALIB_SCALE;
float         distCruiseRPM           = DEFAULT_CRAWL_RPM;
bool          isPositionAnchorLocked  = false;        // Cờ khóa neo tọa độ vạch đích (sai số < 3mm)
long          anchorTargetPulses[4]   = {0, 0, 0, 0}; // Tọa độ xung khóa khi chạm vạch đích

unsigned long lastSpeedCalcTime       = 0;
unsigned long lastRampTime            = 0;
unsigned long lastDebugPrintTime      = 0;
volatile unsigned long lastEncTime[4] = {0, 0, 0, 0};

// Điều khiển ROS 2 & Watchdog an toàn
float         rosTargetRpmSigned[4]   = {0.0f, 0.0f, 0.0f, 0.0f};
unsigned long lastRosCmdTime          = 0;
String        serialRxBuf             = "";
#define ROS_WATCHDOG_TIMEOUT_MS         1500

// ============================================================
//  5. CÁC HÀM NGẮT ENCODER (ISR CÓ LỌC GAI NHIỄU IRAM)
// ============================================================
void IRAM_ATTR isr_enc0() {
  unsigned long now = micros();
  if (now - lastEncTime[0] < MIN_ENC_INTERVAL_US) return;
  if (digitalRead(enc[0].pinA) == LOW) return;
  int b1 = digitalRead(enc[0].pinB);
  for (volatile int w = 0; w < 15; w++);
  if (digitalRead(enc[0].pinB) != b1) return;
  lastEncTime[0] = now;
  enc[0].count += (b1 > 0) ? enc[0].sign : -enc[0].sign;
}

void IRAM_ATTR isr_enc1() {
  unsigned long now = micros();
  if (now - lastEncTime[1] < MIN_ENC_INTERVAL_US) return;
  if (digitalRead(enc[1].pinA) == LOW) return;
  int b1 = digitalRead(enc[1].pinB);
  for (volatile int w = 0; w < 15; w++);
  if (digitalRead(enc[1].pinB) != b1) return;
  lastEncTime[1] = now;
  enc[1].count += (b1 > 0) ? enc[1].sign : -enc[1].sign;
}

void IRAM_ATTR isr_enc2() {
  unsigned long now = micros();
  if (now - lastEncTime[2] < MIN_ENC_INTERVAL_US) return;
  if (digitalRead(enc[2].pinA) == LOW) return;
  int b1 = digitalRead(enc[2].pinB);
  for (volatile int w = 0; w < 15; w++);
  if (digitalRead(enc[2].pinB) != b1) return;
  lastEncTime[2] = now;
  enc[2].count += (b1 > 0) ? enc[2].sign : -enc[2].sign;
}

void IRAM_ATTR isr_enc3() {
  unsigned long now = micros();
  if (now - lastEncTime[3] < MIN_ENC_INTERVAL_US) return;
  if (digitalRead(enc[3].pinA) == LOW) return;
  int b1 = digitalRead(enc[3].pinB);
  for (volatile int w = 0; w < 15; w++);
  if (digitalRead(enc[3].pinB) != b1) return;
  lastEncTime[3] = now;
  enc[3].count += (b1 > 0) ? enc[3].sign : -enc[3].sign;
}

// ============================================================
//  6. KHÓA CỨNG & XUẤT PWM MOTOR (CHỐNG GIẬT KHỞI ĐỘNG)
// ============================================================
void lockAllDriverPins() {
  const int pins[] = {DRV1_RPWM, DRV1_LPWM, DRV2_RPWM, DRV2_LPWM, DRV3_RPWM, DRV3_LPWM, DRV4_RPWM, DRV4_LPWM};
  for (int p : pins) {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }
}

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

// Khai báo nguyên mẫu hàm
void stopMotor(bool emergency);
void stopMotor();
void setVehicleSpeed(int spd);
void updateWheelHealth();
void updatePID(float dt);

// ============================================================
//  7. TÍNH TOÁN VẬN TỐC & SỨC KHỎE BÁNH XE
// ============================================================
inline float median3(float a, float b, float c) {
  if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
  if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
  return c;
}

float calculateMovingAverage(float buffer[], int size) {
  float sum = 0;
  for (int i = 0; i < size; i++) sum += buffer[i];
  return sum / size;
}

void calculateSpeed() {
  unsigned long now = millis();
  if (now - lastSpeedCalcTime < SPEED_CALC_INTERVAL_MS) return;
  float dt = (now - lastSpeedCalcTime) / 1000.0f;
  lastSpeedCalcTime = now;

  for (int i = 0; i < 4; i++) {
    noInterrupts();
    long curCount = enc[i].count;
    interrupts();

    long pulses = curCount - enc[i].lastSpeedCount;
    enc[i].lastSpeedCount = curCount;

    // 1. TẤM KHIÊN GIỚI HẠN VẬT LÝ TUYỆT ĐỐI (Physical Pulse Clamp):
    if (labs(pulses) > 45) {
      pulses = (pulses > 0) ? 45 : -45;
    }

    // 2. BỘ LỌC CHỐNG TRÔI ẢO KHI DỪNG (Deadband Zero-Motion Suppression):
    if (!isMoving && wpid[i].targetRPM <= 0.1f && labs(pulses) <= 1) {
      pulses = 0;
    }

    float rawRPM = (pulses * 60.0f) / (ENCODER_PPR * dt);

    // 3. BỘ LỌC TRUNG VỊ 3 ĐIỂM (Median-3 Filter):
    float medRPM = median3(rawRPM, enc[i].rawRpmHist[0], enc[i].rawRpmHist[1]);
    enc[i].rawRpmHist[1] = enc[i].rawRpmHist[0];
    enc[i].rawRpmHist[0] = rawRPM;

    // 4. BỘ LỌC QUÁN TÍNH CHỐNG ĐỘT BIẾN (Physical Inertia Outlier Filter):
    if (fabsf(medRPM) > MAX_PLAUSIBLE_RPM || (isMoving && fabsf(medRPM - enc[i].prevFilteredRpm) > 50.0f)) {
      medRPM = enc[i].prevFilteredRpm;
    }

    enc[i].rpmBuffer[enc[i].bufferIndex] = medRPM;
    enc[i].bufferIndex = (enc[i].bufferIndex + 1) % MOVING_AVG_SIZE;
    
    // 1. Bộ lọc Trung bình trượt + EMA lọc thô
    float movingAvgRPM = calculateMovingAverage(enc[i].rpmBuffer, MOVING_AVG_SIZE);
    float alpha = (fabsf(movingAvgRPM) < 40.0f) ? EMA_LOW_SPEED_ALPHA : EMA_HIGH_SPEED_ALPHA;
    float emaRpm = alpha * movingAvgRPM + (1.0f - alpha) * enc[i].prevFilteredRpm;
    enc[i].prevFilteredRpm = emaRpm;

    // 2. BỘ LỌC KALMAN 1D TỐI ƯU: Loại bỏ nhiễu Gauss, khử trễ pha, cung cấp vận tốc chuẩn xác cho PID
    enc[i].rpm = updateKalman1D(kfRpm[i], emaRpm);

    enc[i].speed_ms   = enc[i].rpm * WHEEL_CIRCUMFERENCE / 60.0f;
    enc[i].distance_m += (labs(pulses) * WHEEL_CIRCUMFERENCE * distCalibScale) / ENCODER_PPR;
  }

#if WHEEL3_ENCODER_FAULT
  enc[3].rpm        = enc[2].rpm;
  enc[3].speed_ms   = enc[2].speed_ms;
  enc[3].distance_m = enc[2].distance_m;
  enc[3].count      = enc[2].count;
#endif

  // Gửi Odometry phản hồi lên ROS 2 qua Serial (20 Hz)
  float v_left  = (enc[0].speed_ms + enc[1].speed_ms) / 2.0f;
  float v_right = (enc[2].speed_ms + enc[3].speed_ms) / 2.0f;

  if (currentDirection == "BACKWARD") {
    v_left  = -fabsf(v_left);
    v_right = -fabsf(v_right);
  } else if (currentDirection == "LEFT") {
    v_left  = -fabsf(v_left);
    v_right = fabsf(v_right);
  } else if (currentDirection == "RIGHT") {
    v_left  = fabsf(v_left);
    v_right = -fabsf(v_right);
  } else if (currentDirection == "ROS") {
    v_left  = (rosTargetRpmSigned[0] < 0) ? -fabsf(v_left) : fabsf(v_left);
    v_right = (rosTargetRpmSigned[2] < 0) ? -fabsf(v_right) : fabsf(v_right);
  } else {
    if (slew[0].target < 0) v_left  = -fabsf(v_left);
    if (slew[2].target < 0) v_right = -fabsf(v_right);
  }

  if (!isMoving || (fabsf(enc[0].rpm) < 1.0f && fabsf(enc[1].rpm) < 1.0f)) v_left = 0.0f;
  if (!isMoving || (fabsf(enc[2].rpm) < 1.0f && fabsf(enc[3].rpm) < 1.0f)) v_right = 0.0f;

  // 1. Giao thức ODOM chuẩn ROS 2 (m/s)
  Serial.printf("ODOM %.3f %.3f\n", v_left, v_right);

  // 2. Giao thức ENC 4 bánh (ticks & dt_ms)
  unsigned int dt_ms = (unsigned int)(dt * 1000.0f + 0.5f);
  Serial.printf("ENC %ld %ld %ld %ld %u\n", enc[0].count, enc[1].count, enc[2].count, enc[3].count, dt_ms);

  // Điều khiển tự động chạy đúng quãng đường theo mét (Smooth Trapezoidal Distance Profile)
  if (isDistanceMode && isMoving) {
    float traveled0 = abs(enc[0].distance_m - startDistMeters[0]);
    float traveled1 = abs(enc[1].distance_m - startDistMeters[1]);
    float traveled2 = abs(enc[2].distance_m - startDistMeters[2]);
    float traveled3 = abs(enc[3].distance_m - startDistMeters[3]);
    float currentTraveled = (traveled0 + traveled1 + traveled2 + traveled3) / 4.0f;

    float remaining = targetDistMeters - currentTraveled;

    if (remaining <= 0.02f) { // Khi vừa chạm đích (còn dưới 2cm)
      isDistanceMode = false;
      manualDriveActive = false;
      currentDirection = "STOP";
      for (int i = 0; i < 4; i++) {
        wpid[i].targetRPM = 0.0f;
        slew[i].target = 0;
      }

      float maxRpm = 0.0f;
      for (int k = 0; k < 4; k++) {
        float r = fabs(enc[k].rpm);
        if (r > maxRpm) maxRpm = r;
      }
      if (maxRpm < 2.0f) {
        writeAllDrives(0, 0, 0, 0);
        isMoving = false;
        isPositionAnchorLocked = true;
        for (int i = 0; i < 4; i++) anchorTargetPulses[i] = enc[i].count;
        Serial.printf("\n[DIST] >>> DA TOI DICH! Dung chinh xac: %.3fm | KICH HOAT KHOA NEO TOA DO <<<\n\n", currentTraveled);
      }
    } else if (remaining < DECEL_DIST_THRESHOLD) {
      float ratio = constrain(remaining / DECEL_DIST_THRESHOLD, 0.0f, 1.0f);
      float decelRPM = 4.0f + (distCruiseRPM - 4.0f) * (ratio * ratio);
      decelRPM = constrain(decelRPM, 4.0f, distCruiseRPM);
      for (int i = 0; i < 4; i++) wpid[i].targetRPM = decelRPM;
    }
  }

  updateWheelHealth();
  updatePID(dt);
}

void updateWheelHealth() {
  if (!isMoving) {
    for (int i = 0; i < 4; i++) wHealth[i] = {false, false, false, 0, 0};
    return;
  }
  unsigned long now = millis();

  float totalRpm = 0.0f;
  for (int i = 0; i < 4; i++) totalRpm += abs(enc[i].rpm);
  float avgRpm = totalRpm / 4.0f;

  for (int i = 0; i < 4; i++) {
#if WHEEL3_ENCODER_FAULT
    if (i == 3) { wHealth[3] = wHealth[2]; continue; }
#endif
    float rpm = abs(enc[i].rpm);
    int   pwm = abs(slew[i].target);
    float tgt = wpid[i].targetRPM;

    // 1. Phát hiện kẹt cứng / hỏng encoder
    if (pwm > STALL_PWM_THRESHOLD && rpm < STALL_RPM_THRESHOLD) {
      if (wHealth[i].stallStartTime == 0) wHealth[i].stallStartTime = now;
      if (now - wHealth[i].stallStartTime > STALL_DETECT_MS && !wHealth[i].isStalled) {
        wHealth[i].isStalled = true;
        Serial.printf("[HEALTH] CANH BAO: Banh %d bi KHOA/HONG!\n", i + 1);
      }
    } else {
      wHealth[i].stallStartTime = 0;
      wHealth[i].isStalled = false;
    }

    // 2. Phát hiện bánh bị hẫng: Chỉ áp dụng khi chạy chế độ tự hành quãng đường
    if (isDistanceMode && tgt > 0 && avgRpm > 0.1f && rpm > 1.4f * tgt && rpm > 1.5f * avgRpm) {
      if (wHealth[i].hangStartTime == 0) wHealth[i].hangStartTime = now;
      if (now - wHealth[i].hangStartTime > 400 && !wHealth[i].isHanging) {
        wHealth[i].isHanging = true;
        Serial.printf("[HEALTH] PHAT HIEN: Banh %d bi HANG mat tiep xuc dat!\n", i + 1);
      }
    } else {
      wHealth[i].hangStartTime = 0;
      wHealth[i].isHanging = false;
    }

    wHealth[i].isFaulty = (wHealth[i].isStalled || wHealth[i].isHanging);
  }
}

// ============================================================
//  8. THUẬT TOÁN PID + TORQUE BOOST + TỰ BÙ MÔ-MEN BÁNH HỎNG/HẪNG
// ============================================================
void updatePID(float dt) {
  if (!pidGlobalEnabled) return;

  // 1. BỘ KHÓA NEO TỌA ĐỘ VỊ TRÍ VẠCH ĐÍCH (Position Anchor Filter)
  if (isPositionAnchorLocked && !isMoving) {
    for (int i = 0; i < 4; i++) {
      long pulseDiff = enc[i].count - anchorTargetPulses[i];
      if (abs(pulseDiff) > 2) {
        int holdPWM = constrain((int)(-pulseDiff * 0.8f * enc[i].sign), -50, 50);
        slew[i].target = holdPWM;
      } else {
        slew[i].target = 0;
      }
    }
    return;
  }

  if (!isMoving) return;

  float actualRPM[4];
  for (int j = 0; j < 4; j++) actualRPM[j] = fabsf(enc[j].rpm);

  float avgLeft  = (actualRPM[0] + actualRPM[1]) / 2.0f;
  float avgRight = (actualRPM[2] + actualRPM[3]) / 2.0f;
  float avgTotal = (avgLeft + avgRight) / 2.0f;

  // 2. BỘ KHÓA ĐỒNG TỐC VI SAI TRÁI - PHẢI TỨC THỜI:
  float lrDiff = 0.0f;
  if ((currentDirection == "FORWARD" || currentDirection == "BACKWARD") && isMoving) {
    lrDiff = avgLeft - avgRight;
  }
  float lrCorrection = constrain(lrDiff * K_LR_BALANCE, -18.0f, 18.0f);

  for (int i = 0; i < 4; i++) {
#if WHEEL3_ENCODER_FAULT
    if (i == 3) {
      wpid[3].pwmOutput     = wpid[2].pwmOutput;
      wpid[3].prevPwmOutput = wpid[2].prevPwmOutput;
      slew[3].target        = slew[2].target;
      continue;
    }
#endif
    if (!wpid[i].enabled) continue;

    float rpm_act = actualRPM[i];
    float target  = wpid[i].targetRPM;
    bool  isLeft  = (i == 0 || i == 1);

    if (target <= 0.1f) {
      wpid[i].pwmOutput = 0; wpid[i].integral = 0; slew[i].target = 0;
      continue;
    }

    // 1. Feedforward PWM cơ sở
    float ff_pwm = (target / 220.0f) * (255.0f - MIN_PWM) + MIN_PWM;
    float coulomb_ff = (target > 1.0f) ? K_COULOMB_FRICTION : 0.0f;

    // 2. Sai số bám tốc độ mục tiêu (Tracking Error)
    float track_error = target - rpm_act;
    if (fabsf(track_error) < PID_ERROR_DEADBAND) {
      track_error = 0.0f;
    }

    // 3. Sai số đồng tốc Cross-Coupling 4 bánh:
    float sync_error = 0.0f;
    if (currentDirection == "FORWARD" || currentDirection == "BACKWARD" || 
        (currentDirection == "ROS" && fabsf(rosTargetRpmSigned[0] - rosTargetRpmSigned[2]) < 1.0f)) {
      sync_error = (avgTotal - rpm_act);
    }

    // 4. Sai số tổng hợp đưa vào khâu PID:
    float total_error = track_error + (sync_error * K_SYNC_CROSS_WHEEL);

    // 5. Khâu tích phân (Integral) có Anti-Windup thích ứng theo dải tốc độ:
    float maxIntegral = (target <= 65.0f) ? 50.0f : 100.0f;
    wpid[i].integral = constrain(wpid[i].integral + total_error * dt, -maxIntegral, maxIntegral);

    // 6. Khâu vi phân (Derivative) có lọc nhiễu
    float rawDeriv = (total_error - wpid[i].lastError) / dt;
    wpid[i].filteredDeriv = DERIVATIVE_FILTER * rawDeriv + (1.0f - DERIVATIVE_FILTER) * wpid[i].filteredDeriv;
    wpid[i].lastError = total_error;

    // 7. Tính toán PID cơ sở
    float pid_corr = (wpid[i].kp * total_error) + (wpid[i].ki * wpid[i].integral) + (wpid[i].kd * wpid[i].filteredDeriv);

    // 8. Bù cân bằng cụm Trái - Phải trực tiếp:
    float balance_corr = isLeft ? (-lrCorrection) : (lrCorrection);

    // Đồng bộ nội bộ giữa 2 bánh cùng bên
    float side_sync = isLeft ? (avgLeft - rpm_act) : (avgRight - rpm_act);
    float internal_sync_corr = side_sync * 0.40f;

    // 9. BÙ MÔ-MEN XOẮN TỐC ĐỘ THẤP THÍCH ỨNG TẢI NẶNG (Adaptive High-Torque Low-Speed Boost)
    // Cung cấp lực kéo cực đại ở dải tốc độ chậm (10-60 RPM) để xe chở nặng 20-35kg leo dốc, vượt cản không bị lịm
    float torque_boost = 0.0f;
    if (target < 75.0f && track_error > 0.0f) {
      float defectRatio = constrain(track_error / target, 0.0f, 1.0f);
      torque_boost = defectRatio * 70.0f; // Bơm thêm tới +70 PWM khi tải nặng làm tụt tốc
    }
    // Mồi lực khởi động phá ma sát tĩnh ban đầu khi bánh xe gần như đứng yên (< 3 RPM)
    if (rpm_act < 3.0f && target > 2.0f) {
      torque_boost += 35.0f;
    }

    // Khâu ghì hãm êm ái chống vọt tốc độ khi chạy chậm
    if (rpm_act > target + 1.5f) {
      pid_corr -= (rpm_act - target) * 2.0f;
    }

    // 10. Tổng hợp PWM điều khiển hoàn chỉnh:
    int desired = constrain((int)(ff_pwm + coulomb_ff + pid_corr + balance_corr + internal_sync_corr + torque_boost), 0, 255);

    // Trần an toàn thích ứng tải ở tốc độ thấp (Adaptive Load-Aware Low-Speed Cap):
    // Cho phép mở rộng PWM tự động theo khâu tích phân tải để xe không bị nghẽn lực khi chở nặng
    if (target <= 65.0f) {
      int safeLowSpeedCap = constrain((int)(MIN_PWM + (target / 65.0f) * 70.0f + (track_error > 3.0f ? 60 : 0) + max(0.0f, wpid[i].integral * 0.8f)), MIN_PWM, 255);
      if (rpm_act >= target * 0.90f && desired > safeLowSpeedCap) {
        desired = safeLowSpeedCap;
      }
    } else {
      int userPwmCap = max((int)currentSpeed, MIN_PWM);
      if (rpm_act >= target * 0.5f && desired > userPwmCap) {
        desired = userPwmCap;
      }
    }

    // Giới hạn biến thiên PWM bất đối xứng: Bơm lực tăng tốc nhanh khi leo cản
    int maxChange = (desired > wpid[i].prevPwmOutput) ? MAX_PWM_CHANGE_UP : MAX_PWM_CHANGE_DOWN;
    int delta   = constrain(desired - wpid[i].prevPwmOutput, -maxChange, maxChange);
    wpid[i].pwmOutput     = constrain(wpid[i].prevPwmOutput + delta, 0, 255);
    wpid[i].prevPwmOutput = wpid[i].pwmOutput;

    if (wpid[i].pwmOutput > 0 && wpid[i].pwmOutput < MIN_PWM) wpid[i].pwmOutput = MIN_PWM;
    
    // XỬ LÝ ĐẶC BIỆT CHO BÁNH BỊ SỰ CỐ:
    if (wHealth[i].isStalled) { 
      wpid[i].pwmOutput = max(0, wpid[i].pwmOutput - 50); 
      wpid[i].integral = 0; 
    } else if (wHealth[i].isHanging) {
      wpid[i].pwmOutput = MIN_PWM;
      wpid[i].integral = 0;
    }

    // 9. Quy tắc chiều quay 4 bánh (Tank Drive & ROS)
    int sign = 0;
    if (currentDirection == "FORWARD")       sign = 1;
    else if (currentDirection == "BACKWARD") sign = -1;
    else if (currentDirection == "LEFT")     sign = isLeft ? -1 : 1;
    else if (currentDirection == "RIGHT")    sign = isLeft ? 1 : -1;
    else if (currentDirection == "ROS")      sign = (rosTargetRpmSigned[i] >= 0.0f) ? 1 : -1;

    slew[i].target = sign * wpid[i].pwmOutput;
  }
}

// ============================================================
//  9. SLEW RATE LIMITER — KHỞI ĐỘNG MỀM (40 Hz)
// ============================================================
void updateSpeedRamp() {
  unsigned long now = millis();
  if (now - lastRampTime < RAMP_INTERVAL_MS) return;
  lastRampTime = now;

  for (int i = 0; i < 4; i++) {
    float &cur  = slew[i].current;
    int    tgt  = slew[i].target;
    float &step = slew[i].step;

    if (tgt == 0) {
      step = constrain(abs(cur) / 8.0f, RAMP_STEP_MIN, RAMP_STEP_STOP_MAX);
    } else {
      step = constrain(abs((float)tgt - cur) / 40.0f, RAMP_STEP_MIN, RAMP_STEP_MAX);
    }

    if (abs((float)tgt - cur) <= step) cur = (float)tgt;
    else if (cur < (float)tgt)         cur += step;
    else                               cur -= step;
  }

  writeSingleDrive(DRV1_RPWM, DRV1_LPWM, CH_DRV1_F, CH_DRV1_R, (int)slew[0].current, INV_DRV1);
  writeSingleDrive(DRV2_RPWM, DRV2_LPWM, CH_DRV2_F, CH_DRV2_R, (int)slew[1].current, INV_DRV2);
  writeSingleDrive(DRV3_RPWM, DRV3_LPWM, CH_DRV3_F, CH_DRV3_R, (int)slew[2].current, INV_DRV3);
  writeSingleDrive(DRV4_RPWM, DRV4_LPWM, CH_DRV4_F, CH_DRV4_R, (int)slew[3].current, INV_DRV4);
}

// ============================================================
//  10. ĐIỀU KHIỂN HƯỚNG & NHẬN LỆNH
// ============================================================
void setGroupTargets(int speedL, int speedR) {
  if (speedL != 0 && abs(speedL) < MIN_PWM) speedL = (speedL > 0) ? MIN_PWM : -MIN_PWM;
  if (speedR != 0 && abs(speedR) < MIN_PWM) speedR = (speedR > 0) ? MIN_PWM : -MIN_PWM;
  slew[0].target = slew[1].target = speedL;
  slew[2].target = slew[3].target = speedR;
}

void writeSpeed(int speed) {
  if (pidGlobalEnabled) return;
  int s = constrain(speed, 0, 255);
  if (s > 0 && s < MIN_PWM) s = MIN_PWM;

  if (currentDirection == "FORWARD")       setGroupTargets(s, s);
  else if (currentDirection == "BACKWARD") setGroupTargets(-s, -s);
  else if (currentDirection == "LEFT")     setGroupTargets(-s, s);
  else if (currentDirection == "RIGHT")    setGroupTargets(s, -s);
  else                                     setGroupTargets(0, 0);
}

void stopMotor(bool emergency) {
  manualDriveActive = false;
  isDistanceMode    = false;
  currentDirection  = "STOP";

  for (int i = 0; i < 4; i++) {
    slew[i].target = 0;
    wpid[i].targetRPM = 0.0f;
    wpid[i].integral = 0; 
    wpid[i].lastError = 0; 
    wpid[i].filteredDeriv = 0;
    wHealth[i] = {false, false, false, 0, 0};
  }

  if (emergency) {
    for (int i = 0; i < 4; i++) {
      slew[i].current = 0.0f;
      wpid[i].pwmOutput = 0;
      wpid[i].prevPwmOutput = 0;
    }
    writeAllDrives(0, 0, 0, 0);
    isMoving = false;
    isPositionAnchorLocked = false;
    Serial.println("\n[STOP] !!! DUNG KHAN CAP (ESTOP) - NGAT DIEN TUC THI !!!\n");
  } else {
    for (int i = 0; i < 4; i++) {
      wpid[i].pwmOutput = 0;
      wpid[i].prevPwmOutput = 0;
    }
    Serial.println("\n[STOP] Dung em ai - Giam toc muot qua Slew Rate...\n");
  }
}

void stopMotor() {
  stopMotor(false);
}

// ============================================================
// HÀM CẬP NHẬT TỐC ĐỘ XE TẬP TRUNG (DÙNG CHO CẢ APP, WEB, SERIAL)
// ============================================================
void setVehicleSpeed(int spd) {
  currentSpeed = constrain(spd, 0, 255);
  
  if (currentSpeed == 0) {
    stopMotor(false);
    Serial.println("[SPEED] Toc do = 0 -> Dung xe.");
    return;
  }
  
  globalTargetRPM = (currentSpeed / 255.0f) * 220.0f;
  for (int i = 0; i < 4; i++) {
    wpid[i].targetRPM = globalTargetRPM;
  }
  
  if (!pidGlobalEnabled && isMoving && currentDirection != "STOP") {
    writeSpeed(currentSpeed);
  }
  
  Serial.printf("[SPEED] >> DA CAP NHAT TOC DO: PWM = %d (%.0f%%) | Target RPM = %.1f\n", 
                currentSpeed, (currentSpeed / 255.0f) * 100.0f, globalTargetRPM);
}

void handleCommand(String command) {
  command.trim();
  if (command.length() == 0) return;

  // Lệnh kiểm tra IP kết nối App từ Serial Monitor
  if (command == "IP" || command == "ip") {
    Serial.println();
    Serial.println("=========================================");
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print  ("  >> IP KET NOI APP: ");
      Serial.println(WiFi.localIP());
      Serial.printf ("  >> WiFi SSID    : %s (Tin hieu: %d dBm)\n", WiFi.SSID().c_str(), WiFi.RSSI());
    } else {
      Serial.println("  [CHUA CO IP] Xe chua ket noi vao WiFi!");
      Serial.printf ("  SSID dang cau hinh: %s\n", ssid);
      Serial.println("  Hay kiem tra lai ten WiFi va mat khau o dong 71-72!");
    }
    Serial.println("=========================================\n");
    return;
  }

  // Lệnh chạy khoảng cách theo mét: chỉ kích hoạt khi gõ lệnh "DIST <mét>"
  if (command.startsWith("DIST ") || command.startsWith("dist ")) {
    float dist = 0.0f;
    float rpm_req = DEFAULT_CRAWL_RPM;
    int parsed = sscanf(command.c_str() + 5, "%f %f", &dist, &rpm_req);
    if (parsed >= 1 && dist > 0.01f) {
      isPositionAnchorLocked = false;
      isDistanceMode = true;
      targetDistMeters = dist;
      distCruiseRPM = (rpm_req > 5.0f && rpm_req <= 120.0f) ? rpm_req : DEFAULT_CRAWL_RPM;
      for (int i = 0; i < 4; i++) {
        startDistMeters[i] = enc[i].distance_m;
        wpid[i].targetRPM = distCruiseRPM;
        wpid[i].integral = 0;
      }
      isMoving = true;
      manualDriveActive = true;
      currentDirection = "FORWARD";
      Serial.printf("\n[DIST] >>> CHAY QUANG DUONG: %.2fm | Toc do: %.1f RPM <<<\n", targetDistMeters, distCruiseRPM);
      return;
    }
  }

  // Lệnh hiệu chuẩn khoảng cách: "CALIB <scale>" (Ví dụ nếu đo 2m ra 2.5m -> gõ "CALIB 0.80")
  if (command.startsWith("CALIB ") || command.startsWith("calib ")) {
    float sc = 1.0f;
    if (sscanf(command.c_str() + 6, "%f", &sc) == 1 && sc > 0.1f && sc < 3.0f) {
      distCalibScale = sc;
      Serial.printf("[CALIB] Da cap nhat he so Calib quang duong = %.4f\n", distCalibScale);
      return;
    }
  }

  // Lệnh Reset Odometry về 0.00m: "RESET_ODOM" hoặc "ZERO"
  if (command == "RESET_ODOM" || command == "ZERO" || command == "RESET" || command == "reset") {
    for (int i = 0; i < 4; i++) {
      enc[i].count = 0;
      enc[i].distance_m = 0.0f;
      enc[i].lastSpeedCount = 0;
      startDistMeters[i] = 0.0f;
    }
    Serial.println("[ODOM] Da Reset quang duong 4 banh ve 0.00m!");
    return;
  }

  // Lệnh vận tốc ROS 2: "V <FL> <RL> <FR> <RR>" hoặc "V <L> <R>" (RPM)
  if (command.startsWith("V ") || command.startsWith("v ") || command.startsWith("V\t") || command.startsWith("v\t")) {
    float r[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    int parsed = sscanf(command.c_str() + 2, "%f %f %f %f", &r[0], &r[1], &r[2], &r[3]);
    if (parsed == 2) {
      r[3] = r[1];
      r[2] = r[1];
      r[1] = r[0];
      parsed = 4;
    }
    if (parsed == 4) {
      lastRosCmdTime = millis();
      if (fabsf(r[0]) < 0.1f && fabsf(r[1]) < 0.1f && fabsf(r[2]) < 0.1f && fabsf(r[3]) < 0.1f) {
        for (int i = 0; i < 4; i++) rosTargetRpmSigned[i] = 0.0f;
        stopMotor(false);
      } else {
        isPositionAnchorLocked = false;
        isDistanceMode = false;
        isMoving = true;
        manualDriveActive = true;
        currentDirection = "ROS";

        for (int i = 0; i < 4; i++) {
          rosTargetRpmSigned[i] = r[i];
          wpid[i].targetRPM = fabsf(r[i]);
          wpid[i].enabled = pidGlobalEnabled;
          int sign = (r[i] >= 0.0f) ? 1 : -1;
          int pwm = (int)(sign * (MIN_PWM + (fabsf(r[i]) / 220.0f) * (255 - MIN_PWM)));
          slew[i].target = pwm;
        }
      }
      return;
    }
  }

  command.toUpperCase();

  // Lệnh bật/tắt chế độ PID:
  if (command == "PID 0" || command == "PID OFF" || command == "PID=0") {
    pidGlobalEnabled = false;
    for (int i = 0; i < 4; i++) {
      wpid[i].enabled = false;
      wpid[i].integral = 0;
    }
    if (isMoving && currentDirection != "STOP") writeSpeed(currentSpeed);
    Serial.println("\n[MODE] >> DA CHUYEN SANG CHE DO DIEU KHIEN PWM TRUC TIEP (Khong dung PID) <<\n");
    return;
  }
  if (command == "PID 1" || command == "PID ON" || command == "PID=1") {
    pidGlobalEnabled = true;
    for (int i = 0; i < 4; i++) {
      wpid[i].enabled = true;
      wpid[i].integral = 0;
      wpid[i].targetRPM = globalTargetRPM;
    }
    Serial.println("\n[MODE] >> DA CHUYEN SANG CHE DO DONG TOC PID MO-MEN XOAN CAO <<\n");
    return;
  }

  // 1. Nhận lệnh chỉnh tốc độ: "SPEED <val>", "PWM <val>", "TOCDO <val>", "S <val>"
  if (command.startsWith("SPEED ") || command.startsWith("PWM ") || 
      command.startsWith("TOCDO ") || command.startsWith("S ")) {
    int spd = command.substring(command.indexOf(' ') + 1).toInt();
    setVehicleSpeed(spd);
    return;
  }

  // 2. Nhận lệnh dạng: "SPEED=150", "PWM=150"
  if (command.startsWith("SPEED=") || command.startsWith("PWM=")) {
    int spd = command.substring(command.indexOf('=') + 1).toInt();
    setVehicleSpeed(spd);
    return;
  }

  // 3. Nhận lệnh dạng "S150" hoặc "P180" (thường dùng trong app joystick)
  if ((command.startsWith("S") || command.startsWith("P")) && command.length() > 1 && isDigit(command.charAt(1))) {
    int spd = command.substring(1).toInt();
    setVehicleSpeed(spd);
    return;
  }

  // 4. Nhận trực tiếp số từ 1 đến 255
  int numVal = command.toInt();
  if (numVal > 0 && numVal <= 255 && command.length() <= 3) {
    bool isAllDigits = true;
    for (unsigned int c = 0; c < command.length(); c++) {
      if (!isDigit(command.charAt(c))) { isAllDigits = false; break; }
    }
    if (isAllDigits) {
      setVehicleSpeed(numVal);
      return;
    }
  }

  // 5. Chuẩn ký tự tốc độ của các App điều khiển xe RC (0..9 và Q):
  if (command == "0") { setVehicleSpeed(0); return; }
  else if (command == "1") { setVehicleSpeed(40); return; }
  else if (command == "2") { setVehicleSpeed(65); return; }
  else if (command == "3") { setVehicleSpeed(90); return; }
  else if (command == "4") { setVehicleSpeed(120); return; }
  else if (command == "5") { setVehicleSpeed(150); return; }
  else if (command == "6") { setVehicleSpeed(175); return; }
  else if (command == "7") { setVehicleSpeed(200); return; }
  else if (command == "8") { setVehicleSpeed(225); return; }
  else if (command == "9") { setVehicleSpeed(245); return; }
  else if (command == "Q") { setVehicleSpeed(255); return; }

  // Chuẩn hóa tên lệnh điều khiển liên tục (chạy cho đến khi bấm dừng)
  if (command == "FORWARD" || command == "TIEN" || command == "F" || command == "W" || command.startsWith("TIEN"))      command = "FORWARD";
  else if (command == "BACKWARD" || command == "LUI" || command == "B" || command == "S" || command.startsWith("LUI"))  command = "BACKWARD";
  else if (command == "LEFT" || command == "TRAI" || command == "L" || command == "A" || command.startsWith("TRAI"))    command = "LEFT";
  else if (command == "RIGHT" || command == "PHAI" || command == "R" || command == "D" || command.startsWith("PHAI"))   command = "RIGHT";
  else if (command == "STOP" || command == "DUNG" || command == "X" || command == "SPACE")                               command = "STOP";
  else if (command == "EMERGENCY_STOP" || command == "ESTOP")                                                            command = "EMERGENCY_STOP";

  if (command == "FORWARD" || command == "BACKWARD" || command == "LEFT" || command == "RIGHT") {
    isPositionAnchorLocked = false;
    isDistanceMode    = false;
    isMoving          = true;
    manualDriveActive = true;
    currentDirection  = command;

    if (pidGlobalEnabled) {
      for (int i = 0; i < 4; i++) {
        wpid[i].targetRPM = globalTargetRPM;
        if (abs(enc[i].rpm) < 2.0f) wpid[i].integral = 30.0f;
      }
      Serial.printf("[RUN] Chay lien tuc: %s | Target: %.1f RPM | MIN_PWM: %d (Nhan STOP de dung)\n", 
                    command.c_str(), globalTargetRPM, MIN_PWM);
    } else {
      writeSpeed(currentSpeed);
      Serial.println("[RUN] Chay lien tuc: " + command + " | PWM: " + String(currentSpeed) + " (Nhan STOP de dung)");
    }
  } else if (command == "STOP" || command == "stop") {
    stopMotor(false);
  } else if (command == "EMERGENCY_STOP" || command == "estop") {
    stopMotor(true);
  }
}

// ============================================================
//  11. IN DEBUG SERIAL MONITOR (500ms)
// ============================================================
void printDebugInfo() {
  if (currentDirection == "ROS") return; // Nhường băng thông Serial cho gói ODOM của ROS 2

  unsigned long now = millis();
  if (now - lastDebugPrintTime < DEBUG_PRINT_INTERVAL_MS) return;
  lastDebugPrintTime = now;

  bool active = isMoving;
  for (int i = 0; i < 4; i++) if (enc[i].rpm != 0) active = true;
  if (!active) return;

  int healthyCount = 0;
  for (int i = 0; i < 4; i++) if (!wHealth[i].isFaulty) healthyCount++;

  Serial.println("----");
  if (isDistanceMode) {
    float traveled = (abs(enc[0].distance_m - startDistMeters[0]) + abs(enc[1].distance_m - startDistMeters[1]) +
                      abs(enc[2].distance_m - startDistMeters[2]) + abs(enc[3].distance_m - startDistMeters[3])) / 4.0f;
    Serial.printf("[DIST TRACKING] Đang chạy: %.3f / %.2fm (Còn lại: %.3fm | Target: %.1f RPM)\n",
                  traveled, targetDistMeters, max(0.0f, targetDistMeters - traveled), wpid[0].targetRPM);
  }
  if (healthyCount < 4) {
    Serial.printf("[TRACTION] CHẾ ĐỘ BÙ MÔ-MEN: Phát hiện %d bánh sự cố! Bơm lực kéo cho %d bánh lành.\n",
                  4 - healthyCount, healthyCount);
  }

  for (int i = 0; i < 4; i++) {
    Serial.printf("[W%d] RPM:%6.1f | Spd:%.3fm/s | Dist:%.3fm | PWM:%4d",
                  i + 1, enc[i].rpm, enc[i].speed_ms, enc[i].distance_m, slew[i].target);
    if (pidGlobalEnabled && wpid[i].enabled) {
      Serial.printf(" | PID_tgt:%.0f | PID_pwm:%d", wpid[i].targetRPM, wpid[i].pwmOutput);
    }
    if (wHealth[i].isStalled) Serial.print(" [KHOA/HONG]");
    else if (wHealth[i].isHanging) Serial.print(" [HANG/TRUOT]");
    Serial.println();
  }
}

// ============================================================
//  12. SETUP & WEBSERVER CỐT LÕI
// ============================================================
void setup() {
  lockAllDriverPins();

  Serial.begin(115200);
  Serial.setTimeout(10);

#if defined(RGB_LED_PIN)
  neopixelWrite(RGB_LED_PIN, 0, 0, 0);
#endif

  pwmSetup(DRV1_RPWM, CH_DRV1_F); pwmSetup(DRV1_LPWM, CH_DRV1_R);
  pwmSetup(DRV2_RPWM, CH_DRV2_F); pwmSetup(DRV2_LPWM, CH_DRV2_R);
  pwmSetup(DRV3_RPWM, CH_DRV3_F); pwmSetup(DRV3_LPWM, CH_DRV3_R);
  pwmSetup(DRV4_RPWM, CH_DRV4_F); pwmSetup(DRV4_LPWM, CH_DRV4_R);
  writeAllDrives(0, 0, 0, 0);

  void (*isrFn[4])() = {isr_enc0, isr_enc1, isr_enc2, isr_enc3};
  for (int i = 0; i < 4; i++) {
    pinMode(enc[i].pinA, INPUT_PULLUP);
    pinMode(enc[i].pinB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(enc[i].pinA), isrFn[i], RISING);
    wpid[i].enabled = true;
    wpid[i].targetRPM = globalTargetRPM;
  }

  Serial.println("\n=== XE TU HANH 4 BANH PID (ROBOT CONTROLLER) ===");
  Serial.printf("Encoder: %d PPR | Banh: %.0fmm | MIN_PWM: %d | 4-Wheel Torque Boost: ON\n",
                ENCODER_PPR, WHEEL_DIAMETER_M * 1000, MIN_PWM);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.printf("\nDang ket noi den WiFi '%s'...", ssid);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 25) {
    writeAllDrives(0, 0, 0, 0);
    delay(500); 
    Serial.print("."); 
    attempts++;
  }
  Serial.println();
  Serial.println("=========================================");
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("  WIFI DA KET NOI THANH CONG!");
    Serial.print  ("  >> IP KET NOI APP: ");
    Serial.println(WiFi.localIP());
    Serial.println("  (Go chu 'IP' tren Serial de xem lai bat ky luc nao)");
  } else {
    Serial.println("  [CANH BAO] Khong the ket noi WiFi!");
    Serial.printf ("  Khong tim thay hoac sai mat khau mang: '%s'\n", ssid);
    Serial.println("  Vui long sua lai ssid/password o dong 71-72!");
  }
  Serial.println("=========================================\n");

  server.enableCORS(true);

  server.on("/ping", HTTP_GET, []() {
    server.send(200, "text/plain", "ESP32 4-Wheel PID - IP: " + WiFi.localIP().toString());
  });

  server.on("/control", []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");

    int spd = -1;
    if (server.hasArg("speed"))      spd = server.arg("speed").toInt();
    else if (server.hasArg("val"))   spd = server.arg("val").toInt();
    else if (server.hasArg("pwm"))   spd = server.arg("pwm").toInt();
    else if (server.hasArg("value")) spd = server.arg("value").toInt();

    if (spd >= 0) {
      setVehicleSpeed(spd);
    }

    String cmd = "";
    if (server.hasArg("cmd"))          cmd = server.arg("cmd");
    else if (server.hasArg("command")) cmd = server.arg("command");
    else if (server.hasArg("action"))  cmd = server.arg("action");
    else if (server.hasArg("dir"))     cmd = server.arg("dir");

    if (cmd.length() > 0) {
      handleCommand(cmd);
      server.send(200, "text/plain", "OK: " + cmd + " | Speed: " + String(currentSpeed));
      return;
    }

    if (spd >= 0) {
      server.send(200, "text/plain", "OK: Speed updated to " + String(currentSpeed));
      return;
    }

    server.send(400, "text/plain", "Missing cmd or speed");
  });

  server.on("/speed", []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    int spd = currentSpeed;
    if (server.hasArg("val"))        spd = server.arg("val").toInt();
    else if (server.hasArg("value")) spd = server.arg("value").toInt();
    else if (server.hasArg("speed")) spd = server.arg("speed").toInt();
    else if (server.hasArg("pwm"))   spd = server.arg("pwm").toInt();
    setVehicleSpeed(spd);
    server.send(200, "text/plain", "OK: Speed = " + String(currentSpeed));
  });

  server.on("/pwm", []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    int spd = currentSpeed;
    if (server.hasArg("val"))        spd = server.arg("val").toInt();
    else if (server.hasArg("pwm"))   spd = server.arg("pwm").toInt();
    else if (server.hasArg("value")) spd = server.arg("value").toInt();
    setVehicleSpeed(spd);
    server.send(200, "text/plain", "OK: PWM = " + String(currentSpeed));
  });

  server.on("/status", HTTP_GET, []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    String json = "{\"is_moving\":" + String(isMoving ? "true" : "false") + ",";
    json += "\"direction\":\"" + currentDirection + "\",";
    json += "\"speed\":" + String(currentSpeed) + ",";
    json += "\"pid_enabled\":" + String(pidGlobalEnabled ? "true" : "false") + ",";
    json += "\"rpm_left\":" + String((enc[0].rpm + enc[1].rpm) / 2.0f, 1) + ",";
    json += "\"rpm_right\":" + String((enc[2].rpm + enc[3].rpm) / 2.0f, 1) + "}";
    server.send(200, "application/json", json);
  });

  server.on("/encoders", HTTP_GET, []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    String json = "{";
    for (int i = 0; i < 4; i++) {
      json += "\"enc" + String(i + 1) + "\":{";
      json += "\"count\":"      + String(enc[i].count)        + ",";
      json += "\"rpm\":"        + String(enc[i].rpm,       1) + ",";
      json += "\"speed_ms\":"   + String(enc[i].speed_ms,  3) + ",";
      json += "\"distance_m\":" + String(enc[i].distance_m,3) + "}";
      if (i < 3) json += ",";
    }
    json += "}";
    server.send(200, "application/json", json);
  });

  server.on("/pid", []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    if (server.hasArg("enabled")) {
      String en = server.arg("enabled");
      pidGlobalEnabled = (en == "1" || en == "true");
      for (int i = 0; i < 4; i++) {
        wpid[i].enabled = pidGlobalEnabled;
        wpid[i].integral = 0; wpid[i].lastError = 0;
        wpid[i].pwmOutput = currentSpeed; wpid[i].prevPwmOutput = currentSpeed;
      }
    }
    if (server.hasArg("target_rpm")) {
      globalTargetRPM = server.arg("target_rpm").toFloat();
      for (int i = 0; i < 4; i++) wpid[i].targetRPM = globalTargetRPM;
    }
    server.send(200, "application/json", "{\"pid_enabled\":" + String(pidGlobalEnabled ? "true" : "false") + ",\"target_rpm\":" + String(globalTargetRPM, 1) + "}");
  });

  server.begin();
  Serial.println("Web Server da san sang!");
}

// ============================================================
//  13. VÒNG LẶP CHÍNH (LOOP)
// ============================================================
void loop() {
  server.handleClient();

  // Đọc Serial non-blocking không làm trễ chu kỳ điều khiển động cơ
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialRxBuf.length() > 0) {
        handleCommand(serialRxBuf);
        serialRxBuf = "";
      }
    } else {
      if (serialRxBuf.length() < 64) serialRxBuf += c;
    }
  }

  updateSpeedRamp();

  if (currentDirection == "STOP" && isMoving) {
    bool allStopped = true;
    for (int i = 0; i < 4; i++) {
      if (fabsf(slew[i].current) > 2.0f || fabsf(enc[i].rpm) > 1.5f) {
        allStopped = false;
        break;
      }
    }
    if (allStopped) {
      writeAllDrives(0, 0, 0, 0);
      isMoving = false;
    }
  }

  // An toàn ROS Watchdog (600ms không có lệnh từ Pi -> Tự động dừng xe an toàn)
  if (currentDirection == "ROS" && isMoving) {
    if (millis() - lastRosCmdTime > ROS_WATCHDOG_TIMEOUT_MS) {
      stopMotor(false);
      Serial.println("# [WATCHDOG] Pi mat ket noi qua 600ms -> Tu dong ngat xe an toan!");
    }
  }

  if (manualDriveActive && currentDirection != "STOP" && currentDirection != "ROS" && !pidGlobalEnabled) {
    writeSpeed(currentSpeed);
  }

  calculateSpeed();
  printDebugInfo();

  delay(1);
}
