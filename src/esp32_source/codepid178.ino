#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

// ============================================================
//  THÔNG SỐ ROBOT
// ============================================================
#define ENCODER_PPR             200       // Xung/vòng (encoder gắn trục bánh xe)
#define WHEEL_DIAMETER_M        0.20      // Đường kính bánh xe (m)
#define WHEEL_CIRCUMFERENCE     (PI * WHEEL_DIAMETER_M)  // ≈ 0.6283 m

#define SPEED_CALC_INTERVAL_MS  50        // Tính tốc độ mỗi 50ms (20 Hz) giúp PID phản hồi nhanh gấp đôi
#define MOVING_AVG_SIZE         4         // Số mẫu trung bình trượt (mịn và không trễ)
#define DEBUG_PRINT_INTERVAL_MS 500       // In debug mỗi 500ms

// Deadzone & rate limiting
#define MIN_PWM                 75        // Deadzone thực tế motor 775 24V hộp số
#define MAX_PWM_CHANGE          10        // Giới hạn thay đổi PWM/chu kỳ (đáp ứng nhanh và mượt)
#define DERIVATIVE_FILTER       0.6f      // Lọc vi phân chống nhiễu encoder

// ============================================================
//  CHẾ ĐỘ DỰ PHÒNG KHI CÓ ENCODER BỊ HỎNG (FAULT TOLERANCE)
//  false: Đủ 4 encoder hoạt động độc lập (16,17 | 38,39 | 10,11 | 40,41)
//  true:  Nếu có encoder nào hỏng thì bật để tự động mirror
// ============================================================
#define WHEEL3_ENCODER_FAULT    false

// Phát hiện bất thường bánh xe
#define STALL_DETECT_MS         600       // Hộp số 220RPM khởi động chậm hơn → tăng timeout
#define STALL_PWM_THRESHOLD     90        // Ngưỡng PWM để xét khoá (cao hơn deadzone)
#define STALL_RPM_THRESHOLD     8.0f      // RPM dưới ngưỡng này khi PWM cao = khoá
#define SPIN_RPM_RATIO          2.0f      // Hộp số ít bị treo → ratio thấp hơn để phát hiện sớm

// ============================================================
//  CHÂN ĐIỀU KHIỂN BTS7960 (IBT-2) — 4 DRIVER ĐỘC LẬP
//  DRV1 = bánh trái trước  | DRV2 = bánh trái sau
//  DRV3 = bánh phải trước  | DRV4 = bánh phải sau

// ============================================================
//  Sơ đồ GPIO BTS7960:
//    RPWM = chiều TIẾN (HIGH → motor quay thuận)
//    LPWM = chiều LÙI  (HIGH → motor quay ngược)
//
//  DRV1 = bánh trái  trước  | RPWM=47  LPWM=4 (đã đổi từ 48 để tránh xung đột LED RGB)
//  DRV2 = bánh trái  sau    | RPWM=45  LPWM=18
//  DRV3 = bánh phải  trước  | RPWM=13  LPWM=15
//  DRV4 = bánh phải  sau    | RPWM=20  LPWM=21
// ============================================================
#define DRV1_RPWM   47
#define DRV1_LPWM   4
#define DRV2_RPWM   45
#define DRV2_LPWM   18
#define DRV3_RPWM   13
#define DRV3_LPWM   15
#define DRV4_RPWM   20
#define DRV4_LPWM   21

// Đảo chiều từng driver nếu đấu dây motor ngược (+1: bình thường, -1: đảo chiều)
#define INV_DRV1    1   // DRV1 (bánh trái trước)
#define INV_DRV2    1   // DRV2 (bánh trái sau)
#define INV_DRV3    1   // DRV3 (bánh phải trước)
#define INV_DRV4    1   // DRV4 (bánh phải sau)

// ============================================================
//  WiFi
// ============================================================
const char* ssid     = "CTU";
const char* password = "";

WebServer server(80);

// ============================================================
//  PWM Setup (ESP32 LEDC)
// ============================================================
const int freq       = 7000;
const int resolution = 8;   // 8-bit → 0–255

// Kênh LEDC: mỗi driver cần 2 kênh (tiến + lùi)
const int CH_DRV1_F = 0, CH_DRV1_R = 1;
const int CH_DRV2_F = 2, CH_DRV2_R = 3;
const int CH_DRV3_F = 4, CH_DRV3_R = 5;
const int CH_DRV4_F = 6, CH_DRV4_R = 7;

// ============================================================
//  BIẾN ĐIỀU KHIỂN CHUNG
// ============================================================
int     currentSpeed     = 255;
bool    isMoving         = false;
bool    manualDriveActive = false;
String  currentDirection = "STOP";

// Quay vòng cung — bánh bên trong chạy chậm hơn
float turnInnerRatio = 0.30f;   // Bánh trong = 30% tốc độ đang set

// ============================================================
//  SLEW RATE LIMITER — KHỞI ĐỘNG MỀM (4 kênh độc lập)
// ============================================================
struct SlewChannel {
  int   target;            // Tốc độ mục tiêu (có dấu, ±255)
  float current;           // Tốc độ hiện tại (float để ramp mịn)
  float step;              // Bước tăng/giảm mỗi chu kỳ
};

SlewChannel slew[4] = {};   // slew[0]–slew[3] tương ứng DRV1–DRV4

unsigned long lastRampTime  = 0;
const unsigned long RAMP_INTERVAL_MS = 25;    // 40 Hz
// Motor 24V hộp số 220RPM: moment lớn, cần ramp chậm để tránh giật dây curoa/xích
const float RAMP_STEP_MAX       = 2.0f;   // Tăng tốc tối đa 2 PWM/25ms = 80 PWM/s
const float RAMP_STEP_MIN       = 0.3f;
const float RAMP_STEP_STOP_MAX  = 1.5f;  // Giảm tốc chậm hơn tăng tốc (bảo vệ hộp số)

// ============================================================
//  ENCODER DATA STRUCT
// ============================================================
struct EncoderData {
  int pinA;
  int pinB;
  int sign;   // +1 bình thường, -1 nếu cần đảo dấu
  volatile long count;
  long  lastSpeedCount;
  float rpm;
  float speed_ms;
  float distance_m;
  float rpmBuffer[MOVING_AVG_SIZE];
  int   bufferIndex;
};

EncoderData enc[4] = {
  // pinA, pinB, sign, count, lastCount, rpm, spd, dist, buffer, idx
  {16, 17, 1, 0, 0, 0, 0, 0, {0}, 0}, // enc[0]: Bánh trái trước (GPIO 16: Pha A, GPIO 17: Pha B)
  {38, 39, 1, 0, 0, 0, 0, 0, {0}, 0}, // enc[1]: Bánh trái sau   (GPIO 38: Pha A, GPIO 39: Pha B)
  {10, 11, 1, 0, 0, 0, 0, 0, {0}, 0}, // enc[2]: Bánh phải trước (GPIO 10: Pha A, GPIO 11: Pha B)
  {40, 41, 1, 0, 0, 0, 0, 0, {0}, 0}  // enc[3]: Bánh phải sau   (GPIO 40: Pha A, GPIO 41: Pha B)
};

// ============================================================
//  PID CONTROLLER — 4 KÊNH ĐỘC LẬP (mỗi bánh 1 bộ PID)
// ============================================================
struct WheelPID {
  float kp, ki, kd;
  float targetRPM;
  float integral;
  float lastError;
  float filteredDeriv;
  int   pwmOutput;          // PWM hiện tại do PID tính
  int   prevPwmOutput;      // PWM chu kỳ trước (rate limiting)
  bool  enabled;
};

// ============================================================
//  HỆ SỐ PID + FEEDFORWARD (FF) — CÀI ĐẶT TỪNG BÁNH
//  Robot 30kg, 50×60cm | Motor 24V hộp số 220RPM | Encoder 200PPR
//  Kết hợp Feedforward + Positional PID + Cross-Coupling Synchronization
//
//  ┌─────────────┬────────┬────────┬────────┬──────────────────────────┐
//  │   Bánh      │   Kp   │   Ki   │   Kd   │  Ghi chú                 │
//  ├─────────────┼────────┼────────┼────────┼──────────────────────────┤
//  │ 0 Trái trước│ 0.650  │ 0.850  │ 0.040  │ Tải trước 45%            │
//  │ 1 Trái sau  │ 0.750  │ 1.050  │ 0.050  │ Tải sau 55%              │
//  │ 2 Phải trước│ 0.650  │ 0.850  │ 0.040  │ Tải trước 45%            │
//  │ 3 Phải sau  │ 0.750  │ 1.050  │ 0.050  │ Tải sau 55%              │
//  └─────────────┴────────┴────────┴────────┴──────────────────────────┘
// ============================================================

// ----- Bánh 0: TRÁI TRƯỚC (6.75 kg) -----
#define PID0_KP   0.650f
#define PID0_KI   0.850f
#define PID0_KD   0.040f

// ----- Bánh 1: TRÁI SAU (8.25 kg) -------
#define PID1_KP   0.750f
#define PID1_KI   1.050f
#define PID1_KD   0.050f

// ----- Bánh 2: PHẢI TRƯỚC (6.75 kg) -----
#define PID2_KP   0.650f
#define PID2_KI   0.850f
#define PID2_KD   0.040f

// ----- Bánh 3: PHẢI SAU (8.25 kg) -------
#define PID3_KP   0.750f
#define PID3_KI   1.050f
#define PID3_KD   0.050f

#define K_SYNC_CROSS_WHEEL   0.35f   // Hệ số bù trừ chéo đồng tốc giữa 4 bánh

WheelPID wpid[4] = {
  //    kp        ki        kd     target  integral  lastErr  filtDrv  pwm  prevPwm  enabled
  {PID0_KP, PID0_KI, PID0_KD,  0,      0,        0,       0,       0,   0,      false}, // bánh 0: TRÁI TRƯỚC
  {PID1_KP, PID1_KI, PID1_KD,  0,      0,        0,       0,       0,   0,      false}, // bánh 1: TRÁI SAU
  {PID2_KP, PID2_KI, PID2_KD,  0,      0,        0,       0,       0,   0,      false}, // bánh 2: PHẢI TRƯỚC
  {PID3_KP, PID3_KI, PID3_KD,  0,      0,        0,       0,       0,   0,      false}, // bánh 3: PHẢI SAU
};

bool pidGlobalEnabled = false;  // Bật/tắt toàn bộ PID cùng lúc

// Target RPM cho 4 bánh (đồng tốc toàn bộ hoặc theo bên)
float globalTargetRPM = 100.0f; // Target RPM chuẩn cho cả 4 bánh
float targetRPM_Left  = 100.0f;
float targetRPM_Right = 100.0f;

// ============================================================
//  SỨC KHOẺ BÁNH XE (4 bánh độc lập)
// ============================================================
struct WheelHealth {
  bool isStalled;
  bool isSpinning;
  unsigned long stallStartTime;
};

WheelHealth wHealth[4] = {};

// ============================================================
//  TIMER VARIABLES
// ============================================================
unsigned long lastSpeedCalcTime  = 0;
unsigned long lastDebugPrintTime = 0;

// ============================================================
//  CALIBRATION
// ============================================================
bool          isCalibrating   = false;
unsigned long calibStartTime  = 0;
int           calibPWM        = 128;
unsigned long calibDuration   = 5000;
long          calibStartCount[4] = {0, 0, 0, 0};

#define RGB_LED_PIN             48        // Chân LED RGB (WS2812) tích hợp trên ESP32-S3

// ============================================================
//  KHAI BÁO HÀM TRƯỚC
// ============================================================
void pwmSetup(int pin, int channel);
void pwmWrite(int pin, int channel, int value);
void writeSingleDrive(int rpwmPin, int lpwmPin, int chF, int chR, int speed, int inv = 1);
void writeAllDrives(int s0, int s1, int s2, int s3);
void setWheelTarget(int wheel, int speed);
void setGroupTargets(int speedL, int speedR);
void writeSpeed(int speed);
void applyMotorDirection(String direction);
void stopMotor();
void brake();
void emergencyStop();
void escapeObstacle();
void handleCommand(String command);
void updateSpeedRamp();
void calculateSpeed();
void updateWheelHealth();
void updatePID();
void updateCalibration();
void printDebugInfo();
float calculateMovingAverage(float buffer[], int size);
void turnOffRGB();

// ============================================================
//  LỌC NHIỄU ENCODER (GLITCH FILTER)
//  Motor 220 RPM, 200 PPR -> Chu kỳ xung nhỏ nhất lúc quay max tốc ~ 1360 us.
//  Mọi xung kích hoạt ngắt cách nhau < 200 us là nhiễu điện từ (EMI) từ motor/driver.
// ============================================================
#define MIN_ENC_INTERVAL_US     200       // Lọc bỏ gai nhiễu < 200 microgiây
#define MAX_PLAUSIBLE_RPM       350.0f    // Giới hạn vật lý (motor max 220 RPM, trên 350 là nhiễu)

volatile unsigned long lastEncTime[4] = {0, 0, 0, 0};

// ============================================================
//  ISR — Ngắt đọc encoder có lọc nhiễu thời gian thực (IRAM)
// ============================================================
void IRAM_ATTR isr_enc0() {
  unsigned long now = micros();
  if (now - lastEncTime[0] < MIN_ENC_INTERVAL_US) return; // Bỏ qua gai nhiễu
  lastEncTime[0] = now;
  if (digitalRead(enc[0].pinB) > 0) enc[0].count += enc[0].sign;
  else                               enc[0].count -= enc[0].sign;
}

void IRAM_ATTR isr_enc1() {
  unsigned long now = micros();
  if (now - lastEncTime[1] < MIN_ENC_INTERVAL_US) return; // Bỏ qua gai nhiễu
  lastEncTime[1] = now;
  if (digitalRead(enc[1].pinB) > 0) enc[1].count += enc[1].sign;
  else                               enc[1].count -= enc[1].sign;
}

void IRAM_ATTR isr_enc2() {
  unsigned long now = micros();
  if (now - lastEncTime[2] < MIN_ENC_INTERVAL_US) return; // Bỏ qua gai nhiễu
  lastEncTime[2] = now;
  if (digitalRead(enc[2].pinB) > 0) enc[2].count += enc[2].sign;
  else                               enc[2].count -= enc[2].sign;
}

void IRAM_ATTR isr_enc3() {
  unsigned long now = micros();
  if (now - lastEncTime[3] < MIN_ENC_INTERVAL_US) return; // Bỏ qua gai nhiễu
  lastEncTime[3] = now;
  if (digitalRead(enc[3].pinB) > 0) enc[3].count += enc[3].sign;
  else                               enc[3].count -= enc[3].sign;
}

// ============================================================
//  KHÓA CỨNG TOÀN BỘ CHÂN DRIVER VỀ 0V (CHỐNG TỰ QUAY KHI KHỞI ĐỘNG)
// ============================================================
void lockAllDriverPins() {
  const int drvPins[] = {
    DRV1_RPWM, DRV1_LPWM,
    DRV2_RPWM, DRV2_LPWM,
    DRV3_RPWM, DRV3_LPWM,
    DRV4_RPWM, DRV4_LPWM
  };
  for (int i = 0; i < 8; i++) {
    pinMode(drvPins[i], OUTPUT);
    digitalWrite(drvPins[i], LOW);
  }
}

// ============================================================
//  PWM WRAPPER (ESP32 Arduino Core >= 3.0.0)
//  API mới: ledcAttach(pin, freq, bits) + ledcWrite(pin, duty)
//  Không còn ledcSetup / ledcAttachPin / channel riêng
// ============================================================
void pwmSetup(int pin, int channel) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW); // Ghim mức 0V ngay lập tức chống giật motor
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcAttach(pin, freq, resolution);
  ledcWrite(pin, 0);
#else
  ledcSetup(channel, freq, resolution);
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

// ============================================================
//  MOVING AVERAGE
// ============================================================
float calculateMovingAverage(float buffer[], int size) {
  float sum = 0;
  for (int i = 0; i < size; i++) sum += buffer[i];
  return sum / size;
}

// ============================================================
//  TÍNH TỐC ĐỘ ENCODER (gọi mỗi 100ms)
// ============================================================
void calculateEncoderSpeed(EncoderData &e, float dt) {
  noInterrupts();
  long currentCount = e.count;
  interrupts();

  long pulses = currentCount - e.lastSpeedCount;
  e.lastSpeedCount = currentCount;

  float rawRPM = (pulses * 60.0f) / (ENCODER_PPR * dt);

  // Lọc giá trị RPM bất thường do nhiễu đột biến vượt quá tốc độ vật lý
  if (abs(rawRPM) > MAX_PLAUSIBLE_RPM) {
    rawRPM = 0;
  }

  e.rpmBuffer[e.bufferIndex] = rawRPM;
  e.bufferIndex = (e.bufferIndex + 1) % MOVING_AVG_SIZE;
  e.rpm = calculateMovingAverage(e.rpmBuffer, MOVING_AVG_SIZE);

  e.speed_ms   = e.rpm * WHEEL_CIRCUMFERENCE / 60.0f;
  e.distance_m += (abs(pulses) * WHEEL_CIRCUMFERENCE) / ENCODER_PPR;
}

void calculateSpeed() {
  unsigned long now = millis();
  if (now - lastSpeedCalcTime < SPEED_CALC_INTERVAL_MS) return;
  float dt = (now - lastSpeedCalcTime) / 1000.0f;
  lastSpeedCalcTime = now;

  for (int i = 0; i < 4; i++) {
    calculateEncoderSpeed(enc[i], dt);
  }

#if WHEEL3_ENCODER_FAULT
  // Khi encoder 3 bị hỏng: Tự động copy giá trị từ Bánh 2 (cùng phía bên Phải)
  enc[3].rpm         = enc[2].rpm;
  enc[3].speed_ms    = enc[2].speed_ms;
  enc[3].distance_m  = enc[2].distance_m;
  enc[3].count       = enc[2].count;
#endif
}

// ============================================================
//  KIỂM TRA SỨC KHOẺ TỪNG BÁNH
// ============================================================
void updateWheelHealth() {
  if (!isMoving) {
    for (int i = 0; i < 4; i++) {
      wHealth[i] = {false, false, 0};
    }
    return;
  }

  unsigned long now = millis();

  for (int i = 0; i < 4; i++) {
#if WHEEL3_ENCODER_FAULT
    if (i == 3) {
      wHealth[3] = wHealth[2]; // Đồng bộ trạng thái theo Bánh 2
      continue;
    }
#endif

    float rpm = abs(enc[i].rpm);
    int   pwm = abs(slew[i].target);

    // Phát hiện bánh khoá
    if (pwm > STALL_PWM_THRESHOLD && rpm < STALL_RPM_THRESHOLD) {
      if (wHealth[i].stallStartTime == 0) wHealth[i].stallStartTime = now;
      if (now - wHealth[i].stallStartTime > STALL_DETECT_MS && !wHealth[i].isStalled) {
        wHealth[i].isStalled = true;
        Serial.printf("[HEALTH] CANH BAO: Banh %d bi KHOA!\n", i + 1);
      }
    } else {
      if (wHealth[i].isStalled) Serial.printf("[HEALTH] Banh %d da phuc hoi\n", i + 1);
      wHealth[i].stallStartTime = 0;
      wHealth[i].isStalled = false;
    }

    // Phát hiện bánh treo
    if (pidGlobalEnabled && wpid[i].targetRPM > 10) {
      bool spinning = (rpm > wpid[i].targetRPM * SPIN_RPM_RATIO);
      if (spinning && !wHealth[i].isSpinning)
        Serial.printf("[HEALTH] Banh %d quay tu do (treo?)\n", i + 1);
      wHealth[i].isSpinning = spinning;
    } else {
      wHealth[i].isSpinning = false;
    }
  }
}

// ============================================================
//  PID + FEEDFORWARD + ĐỒNG TỐC CHÉO 4 BÁNH (CROSS-COUPLING)
//  Mỗi bánh có bộ PID riêng, kết hợp Feedforward tức thời
//  và bộ bù lệch vận tốc trung bình để 4 bánh luôn quay cùng tốc độ
// ============================================================
void updatePID() {
  if (!pidGlobalEnabled) return;
  if (!isMoving) return;

  float dt = SPEED_CALC_INTERVAL_MS / 1000.0f;

  // 1. Tính tốc độ thực tế trung bình của các bánh
  float r0 = abs(enc[0].rpm);
  float r1 = abs(enc[1].rpm);
  float r2 = abs(enc[2].rpm);
  float r3 = abs(enc[3].rpm);

  float avgLeft  = (r0 + r1) / 2.0f;
#if WHEEL3_ENCODER_FAULT
  float avgRight = r2;
  float avgTotal = (r0 + r1 + 2.0f * r2) / 4.0f;
#else
  float avgRight = (r2 + r3) / 2.0f;
  float avgTotal = (r0 + r1 + r2 + r3) / 4.0f;
#endif

  for (int i = 0; i < 4; i++) {
#if WHEEL3_ENCODER_FAULT
    if (i == 3) {
      // Bánh 3 tự động bám theo (Mirror) ngõ ra PWM và target của Bánh 2
      wpid[3].pwmOutput     = wpid[2].pwmOutput;
      wpid[3].prevPwmOutput = wpid[2].prevPwmOutput;
      slew[3].target        = slew[2].target;
      continue;
    }
#endif

    if (!wpid[i].enabled) continue;

    float actualRPM = abs(enc[i].rpm);
    float target    = wpid[i].targetRPM;
    bool  isLeft    = (i == 0 || i == 1);

    if (target <= 0) {
      wpid[i].pwmOutput = 0;
      wpid[i].integral = 0;
      slew[i].target = 0;
      continue;
    }

    // 2. FEEDFORWARD (FF) — Cấp trước PWM cơ sở theo mô hình motor 24V 220RPM
    float ff_pwm = (target / 220.0f) * (255.0f - MIN_PWM) + MIN_PWM;

    // 3. POSITIONAL PID — Bù trừ sai số chính xác
    float error = target - actualRPM;

    // Tích phân với Anti-Windup chặt
    wpid[i].integral += error * dt;
    wpid[i].integral  = constrain(wpid[i].integral, -35.0f, 35.0f);

    // Vi phân có lọc thông thấp
    float rawDeriv = (error - wpid[i].lastError) / dt;
    wpid[i].filteredDeriv = DERIVATIVE_FILTER * rawDeriv
                          + (1.0f - DERIVATIVE_FILTER) * wpid[i].filteredDeriv;
    wpid[i].lastError = error;

    float pid_corr = (wpid[i].kp * error)
                   + (wpid[i].ki * wpid[i].integral)
                   + (wpid[i].kd * wpid[i].filteredDeriv);

    // 4. CROSS-COUPLING SYNC — Bù lệch giữa bánh này với các bánh còn lại
    float sync_avg  = (currentDirection == "FORWARD" || currentDirection == "BACKWARD") ? avgTotal : (isLeft ? avgLeft : avgRight);
    float sync_corr = (sync_avg - actualRPM) * K_SYNC_CROSS_WHEEL;

    // Tổng hợp PWM mong muốn
    int desired = (int)(ff_pwm + pid_corr + sync_corr);
    desired = constrain(desired, 0, 255);

    // Rate limiting: Giữ độ mượt cơ khí
    int delta = constrain(desired - wpid[i].prevPwmOutput, -MAX_PWM_CHANGE, MAX_PWM_CHANGE);
    wpid[i].pwmOutput     = constrain(wpid[i].prevPwmOutput + delta, 0, 255);
    wpid[i].prevPwmOutput = wpid[i].pwmOutput;

    // Đảm bảo vượt ngưỡng deadzone ma sát khi đang chạy
    if (wpid[i].pwmOutput > 0 && wpid[i].pwmOutput < MIN_PWM) {
      wpid[i].pwmOutput = MIN_PWM;
    }

    // Xử lý bất thường (kẹt/treo bánh)
    if (wHealth[i].isStalled) {
      wpid[i].pwmOutput = max(0, wpid[i].pwmOutput - 30);
      wpid[i].integral  = 0;
    }
    if (wHealth[i].isSpinning) {
      wpid[i].pwmOutput = max(0, wpid[i].pwmOutput - 20);
    }

    // Xác định chiều quay theo hướng di chuyển đồng tốc:
    // FORWARD:  Trái (+), Phải (+)
    // BACKWARD: Trái (-), Phải (-)
    // LEFT:     Trái (-), Phải (+) (Xoay trái tại chỗ)
    // RIGHT:    Trái (+), Phải (-) (Xoay phải tại chỗ)
    int sign = 0;
    if (currentDirection == "FORWARD") {
      sign = 1;
    } else if (currentDirection == "BACKWARD") {
      sign = -1;
    } else if (currentDirection == "LEFT") {
      sign = isLeft ? -1 : 1;
    } else if (currentDirection == "RIGHT") {
      sign = isLeft ? 1 : -1;
    }

    slew[i].target = sign * wpid[i].pwmOutput;
  }
}

// ============================================================
//  SLEW RATE LIMITER — KHỞI ĐỘNG MỀM (40 Hz)
//  Cập nhật currentSpeed từng bánh tiến/lùi về target rồi xuất PWM
// ============================================================
void updateSpeedRamp() {
  unsigned long now = millis();
  if (now - lastRampTime < RAMP_INTERVAL_MS) return;
  lastRampTime = now;

  for (int i = 0; i < 4; i++) {
    float &cur    = slew[i].current;
    int    tgt    = slew[i].target;
    float &step   = slew[i].step;

    // Tính step động theo độ chênh lệch
    if (tgt == 0) {
      float mag = abs(cur);
      step = constrain(mag / 8.0f, RAMP_STEP_MIN, RAMP_STEP_STOP_MAX);
    } else {
      float diff = abs((float)tgt - cur);
      step = constrain(diff / 40.0f, RAMP_STEP_MIN, RAMP_STEP_MAX);
    }

    // Tiến dần cur về tgt
    if (abs((float)tgt - cur) <= step) {
      cur = (float)tgt;
    } else if (cur < (float)tgt) {
      cur += step;
    } else {
      cur -= step;
    }
  }

  // Xuất PWM ra từng driver với cờ đảo chiều tương ứng
  writeSingleDrive(DRV1_RPWM, DRV1_LPWM, CH_DRV1_F, CH_DRV1_R, (int)slew[0].current, INV_DRV1);
  writeSingleDrive(DRV2_RPWM, DRV2_LPWM, CH_DRV2_F, CH_DRV2_R, (int)slew[1].current, INV_DRV2);
  writeSingleDrive(DRV3_RPWM, DRV3_LPWM, CH_DRV3_F, CH_DRV3_R, (int)slew[2].current, INV_DRV3);
  writeSingleDrive(DRV4_RPWM, DRV4_LPWM, CH_DRV4_F, CH_DRV4_R, (int)slew[3].current, INV_DRV4);
}

// ============================================================
//  MOTOR DRIVER WRITE
// ============================================================
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

// ============================================================
//  ĐẶT TARGET TỐC ĐỘ (qua Slew hoặc PID)
// ============================================================
// Đặt target cho 1 bánh cụ thể (có deadzone tối thiểu)
void setWheelTarget(int wheel, int speed) {
  if (wheel < 0 || wheel > 3) return;
  // MIN_PWM = 75 cho motor 24V hộp số
  if (speed != 0 && abs(speed) < MIN_PWM) {
    speed = (speed > 0) ? MIN_PWM : -MIN_PWM;
  }
  slew[wheel].target = speed;
}

// Đặt target cho 2 nhóm bánh (trái = 0,1 | phải = 2,3)
void setGroupTargets(int speedL, int speedR) {
  if (speedL != 0 && abs(speedL) < MIN_PWM) speedL = (speedL > 0) ? MIN_PWM : -MIN_PWM;
  if (speedR != 0 && abs(speedR) < MIN_PWM) speedR = (speedR > 0) ? MIN_PWM : -MIN_PWM;

  slew[0].target = speedL;
  slew[1].target = speedL;
  slew[2].target = speedR;
  slew[3].target = speedR;
}

// ============================================================
//  writeSpeed — Áp dụng tốc độ theo hướng hiện tại
//
//  Kiểu xoay tại chỗ (tank turn):
//    2 bánh TRÁI  = DRV1 (trái trước) + DRV2 (trái sau)
//    2 bánh PHẢI  = DRV3 (phải trước) + DRV4 (phải sau)
//
//  TIẾN (FORWARD):
//    Trái RPWM=speed  LPWM=0  → quay thuận  (+speed)
//    Phải RPWM=speed  LPWM=0  → quay thuận  (+speed)
//
//  LÙI (BACKWARD):
//    Trái RPWM=0  LPWM=speed  → quay ngược  (-speed)
//    Phải RPWM=0  LPWM=speed  → quay ngược  (-speed)
//
//  RẼ TRÁI tại chỗ (LEFT):
//    Trái RPWM=0  LPWM=speed  → LÙI          (-speed)
//    Phải RPWM=speed  LPWM=0  → TIẾN          (+speed)
//    → Robot xoay ngược chiều kim đồng hồ (quay về bên trái)
//
//  RẼ PHẢI tại chỗ (RIGHT):
//    Trái RPWM=speed  LPWM=0  → TIẾN          (+speed)
//    Phải RPWM=0  LPWM=speed  → LÙI           (-speed)
//    → Robot xoay theo chiều kim đồng hồ (quay về bên phải)
// ============================================================
void writeSpeed(int speed) {
  if (pidGlobalEnabled) return;  // PID tự quản lý, không ghi đè

  if (currentDirection == "FORWARD") {
    // Tất cả 4 bánh tiến: RPWM=speed, LPWM=0
    setGroupTargets(speed, speed);

  } else if (currentDirection == "BACKWARD") {
    // Tất cả 4 bánh lùi: RPWM=0, LPWM=speed
    setGroupTargets(-speed, -speed);

  } else if (currentDirection == "LEFT") {
    // Rẽ trái tại chỗ (tank turn):
    //   Bánh TRÁI (DRV1+DRV2): LPWM=speed → LÙI  → setGroupTargets(-speed, ...)
    //   Bánh PHẢI (DRV3+DRV4): RPWM=speed → TIẾN → setGroupTargets(..., +speed)
    setGroupTargets(-speed, speed);

  } else if (currentDirection == "RIGHT") {
    // Rẽ phải tại chỗ (tank turn):
    //   Bánh TRÁI (DRV1+DRV2): RPWM=speed → TIẾN → setGroupTargets(+speed, ...)
    //   Bánh PHẢI (DRV3+DRV4): LPWM=speed → LÙI  → setGroupTargets(..., -speed)
    setGroupTargets(speed, -speed);

  } else {
    setGroupTargets(0, 0);
  }
}

// ============================================================
//  ĐIỀU KHIỂN HƯỚNG & DỪNG
// ============================================================
void applyMotorDirection(String direction) {
  currentDirection = direction;
}

void stopMotor() {
  manualDriveActive = false;
  currentDirection  = "STOP";

  for (int i = 0; i < 4; i++) {
    slew[i].target  = 0;
    slew[i].current = 0.0f;
  }

  writeAllDrives(0, 0, 0, 0);
  isMoving = false;

  // Reset PID khi dừng
  for (int i = 0; i < 4; i++) {
    wpid[i].integral      = 0;
    wpid[i].lastError     = 0;
    wpid[i].filteredDeriv = 0;
    wpid[i].pwmOutput     = 0;
    wpid[i].prevPwmOutput = 0;
  }

  for (int i = 0; i < 4; i++) wHealth[i] = {false, false, 0};
}

void brake() {
  setGroupTargets(0, 0);
}

void emergencyStop() {
  for (int i = 0; i < 4; i++) {
    slew[i].target  = 0;
    slew[i].current = 0.0f;
  }
  writeAllDrives(0, 0, 0, 0);
  manualDriveActive = false;
  currentDirection  = "STOP";
  isMoving          = false;

  for (int i = 0; i < 4; i++) wHealth[i] = {false, false, 0};
}

// ============================================================
//  XỬ LÝ LỆNH ĐIỀU KHIỂN
// ============================================================
void escapeObstacle() {
  stopMotor();
  delay(100);
  applyMotorDirection("FORWARD");
  writeSpeed(255);
  delay(200);
  stopMotor();
  delay(300);
  applyMotorDirection("FORWARD");
  writeSpeed(255);
  delay(1000);
  stopMotor();
}

void handleCommand(String command) {
  command.trim();
  command.toUpperCase();

  // Chuẩn hóa lệnh đa ngôn ngữ & phím tắt
  if (command == "FORWARD" || command == "TIEN" || command == "F" || command == "W" || command == "UP") {
    command = "FORWARD";
  } else if (command == "BACKWARD" || command == "LUI" || command == "B" || command == "S" || command == "DOWN") {
    command = "BACKWARD";
  } else if (command == "LEFT" || command == "TRAI" || command == "RE_TRAI" || command == "L" || command == "A") {
    command = "LEFT";
  } else if (command == "RIGHT" || command == "PHAI" || command == "RE_PHAI" || command == "R" || command == "D") {
    command = "RIGHT";
  } else if (command == "STOP" || command == "DUNG" || command == "BRAKE" || command == "X" || command == "SPACE") {
    command = "STOP";
  } else if (command == "EMERGENCY_STOP" || command == "ESTOP") {
    command = "EMERGENCY_STOP";
  }

  if (isCalibrating && command != "STOP" && command != "EMERGENCY_STOP") {
    Serial.println("Dang calibrate, chi cho phep STOP!");
    return;
  }

  if (command == "FORWARD" || command == "BACKWARD" ||
      command == "LEFT"    || command == "RIGHT") {
    isMoving          = true;
    manualDriveActive = true;
    applyMotorDirection(command);

    if (pidGlobalEnabled) {
      // Đặt target RPM đồng tốc cho tất cả 4 bánh
      for (int i = 0; i < 4; i++) {
        wpid[i].targetRPM = globalTargetRPM;
      }
      Serial.printf("PID move: %s | 4 banh dong toc Target = %.1f RPM\n",
                    command.c_str(), globalTargetRPM);
    } else {
      writeSpeed(currentSpeed);
      Serial.println("Di chuyen: " + command + " | Speed PWM: " + String(currentSpeed));
    }

  } else if (command == "STOP") {
    isCalibrating    = false;
    isMoving         = false;
    currentDirection = "STOP";
    setGroupTargets(0, 0);  // Slew giảm tốc từ từ

  } else if (command == "EMERGENCY_STOP") {
    isCalibrating = false;
    emergencyStop();

  } else if (command == "ESCAPE_OBSTACLE") {
    escapeObstacle();

  } else {
    Serial.println("Lenh khong hop le: " + command);
  }
}

// ============================================================
//  CALIBRATION
// ============================================================
void updateCalibration() {
  if (!isCalibrating) return;

  unsigned long elapsed = millis() - calibStartTime;
  if (elapsed < calibDuration) return;

  setGroupTargets(0, 0);
  isCalibrating = false;
  isMoving      = false;

  float totalTime_s = calibDuration / 1000.0f;
  Serial.println("===== CALIBRATION COMPLETE =====");
  Serial.printf("PWM: %d | Duration: %.1fs\n", calibPWM, totalTime_s);

  for (int i = 0; i < 4; i++) {
    noInterrupts();
    long cnt = enc[i].count;
    interrupts();

    float pulses = cnt - calibStartCount[i];
    float avgRPM = (pulses / ENCODER_PPR) / totalTime_s * 60.0f;
    float avgSpd = avgRPM * WHEEL_CIRCUMFERENCE / 60.0f;
    Serial.printf("  Banh %d: %.1f RPM (%.3f m/s)\n", i + 1, avgRPM, avgSpd);
  }
  Serial.println("================================");
}

// ============================================================
//  DEBUG SERIAL (mỗi 500ms)
// ============================================================
void printDebugInfo() {
  unsigned long now = millis();
  if (now - lastDebugPrintTime < DEBUG_PRINT_INTERVAL_MS) return;
  lastDebugPrintTime = now;

  bool anyActive = isMoving;
  for (int i = 0; i < 4; i++) if (enc[i].rpm != 0) anyActive = true;
  if (!anyActive) return;

  Serial.println("----");
  for (int i = 0; i < 4; i++) {
    Serial.printf("[W%d] RPM: %6.1f | Speed: %.3f m/s | Dist: %.3f m | PWM_tgt: %4d",
                  i + 1, enc[i].rpm, enc[i].speed_ms, enc[i].distance_m, slew[i].target);
    if (pidGlobalEnabled && wpid[i].enabled) {
      Serial.printf(" | PID_tgt: %.0f RPM | PID_pwm: %d",
                    wpid[i].targetRPM, wpid[i].pwmOutput);
    }
#if WHEEL3_ENCODER_FAULT
    if (i == 3) Serial.print(" [MIRROR W3]");
#endif
    if (wHealth[i].isStalled)  Serial.print(" [KHOA]");
    if (wHealth[i].isSpinning) Serial.print(" [TREO]");
    Serial.println();
  }

  if (currentDirection == "LEFT" || currentDirection == "RIGHT") {
    Serial.printf("[TURN] %s | inner_ratio: %.0f%%\n",
                  currentDirection.c_str(), turnInnerRatio * 100);
  }
}

// ============================================================
//  HÀM TẮT LED RGB (WS2812) TRÊN CHÂN 48 (ESP32-S3)
// ============================================================
void turnOffRGB() {
#ifdef RGB_BUILTIN
  neopixelWrite(RGB_BUILTIN, 0, 0, 0);
#endif
#if defined(RGB_LED_PIN)
  neopixelWrite(RGB_LED_PIN, 0, 0, 0);
#endif
#if defined(ESP_ARDUINO_VERSION) && (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0))
  rgbLedWrite(RGB_LED_PIN, 0, 0, 0);
#endif
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  // 1. NGAY LẬP TỨC ÉP CỨNG TẤT CẢ CHÂN DRIVER XUỐNG 0V ĐỂ CHỐNG GIẬT MOTOR
  lockAllDriverPins();

  Serial.begin(115200);

  // Tắt LED RGB chân 48 trên board ESP32-S3
  turnOffRGB();

  // Khởi tạo 4 driver với PWM 0
  pwmSetup(DRV1_RPWM, CH_DRV1_F); pwmSetup(DRV1_LPWM, CH_DRV1_R);
  pwmSetup(DRV2_RPWM, CH_DRV2_F); pwmSetup(DRV2_LPWM, CH_DRV2_R);
  pwmSetup(DRV3_RPWM, CH_DRV3_F); pwmSetup(DRV3_LPWM, CH_DRV3_R);
  pwmSetup(DRV4_RPWM, CH_DRV4_F); pwmSetup(DRV4_LPWM, CH_DRV4_R);
  writeAllDrives(0, 0, 0, 0);

  // Đảm bảo các kênh Slew ở mức 0 tuyệt đối
  for (int i = 0; i < 4; i++) {
    slew[i].target  = 0;
    slew[i].current = 0.0f;
  }

  // Khởi tạo Encoder
  void (*isrFn[4])() = {isr_enc0, isr_enc1, isr_enc2, isr_enc3};
  for (int i = 0; i < 4; i++) {
    pinMode(enc[i].pinA, INPUT_PULLUP);
    pinMode(enc[i].pinB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(enc[i].pinA), isrFn[i], RISING);
  }

  lastSpeedCalcTime  = millis();
  lastDebugPrintTime = millis();

  Serial.println("=== Robot Controller (4-wheel independent PID) ===");
  Serial.printf("Encoder: %d PPR | Wheel: %.0fmm | Circ: %.4fm\n",
                ENCODER_PPR, WHEEL_DIAMETER_M * 1000, WHEEL_CIRCUMFERENCE);
  Serial.printf("MIN_PWM: %d | Turn ratio: %.0f%% | PID rate limit: %d/cycle\n",
                MIN_PWM, turnInnerRatio * 100, MAX_PWM_CHANGE);
#if WHEEL3_ENCODER_FAULT
  Serial.println("[MODE] Encoder 3 FAULT TOLERANCE ON: Banh 3 chay dong bo (Mirror) theo Banh 2");
#endif

  // ==== WiFi ====
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Dang ket noi WiFi");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    writeAllDrives(0, 0, 0, 0); // Đảm bảo motor luôn dừng trong lúc đợi WiFi
    delay(500);
    Serial.print(".");
    attempts++;
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\n[CANH BAO] Khong the ket noi WiFi! Xe van san sang che do Serial/Manual.");
  } else {
    Serial.println("\nDa ket noi! IP: " + WiFi.localIP().toString());
  }

  writeAllDrives(0, 0, 0, 0);

  server.enableCORS(true);

  // ---- /ping ----
  server.on("/ping", HTTP_GET, []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "text/plain",
                "ESP32 4-Wheel PID Controller - IP: " + WiFi.localIP().toString());
  });

  // ---- /rgb_off ----
  server.on("/rgb_off", HTTP_GET, []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    turnOffRGB();
    server.send(200, "application/json", "{\"status\":\"rgb_off\",\"pin\":48}");
  });

  // ---- /encoders ----
  server.on("/encoders", HTTP_GET, []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    float dist[4];
    for (int i = 0; i < 4; i++) dist[i] = enc[i].distance_m;
    float robot_dist = (dist[0] + dist[1] + dist[2] + dist[3]) / 4.0f;

    String json = "{";
    for (int i = 0; i < 4; i++) {
      json += "\"enc" + String(i + 1) + "\":{";
      json += "\"count\":"      + String(enc[i].count)        + ",";
      json += "\"rpm\":"        + String(enc[i].rpm,       1) + ",";
      json += "\"speed_ms\":"   + String(enc[i].speed_ms,  3) + ",";
      json += "\"distance_m\":" + String(enc[i].distance_m,3) + "}";
      if (i < 3) json += ",";
    }
    json += ",\"robot_distance_m\":"  + String(robot_dist, 4);
    json += ",\"turn_ratio\":"        + String(turnInnerRatio, 2);
    json += ",\"wheel_stalled\":[";
    for (int i = 0; i < 4; i++) {
      json += String(wHealth[i].isStalled ? "true" : "false");
      if (i < 3) json += ",";
    }
    json += "],\"wheel_spinning\":[";
    for (int i = 0; i < 4; i++) {
      json += String(wHealth[i].isSpinning ? "true" : "false");
      if (i < 3) json += ",";
    }
    json += "]}";
    server.send(200, "application/json", json);
  });

  // ---- /odometry ----
  server.on("/odometry", HTTP_GET, []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    float rpmL   = (enc[0].rpm   + enc[1].rpm)   / 2.0f;
    float rpmR   = (enc[2].rpm   + enc[3].rpm)   / 2.0f;
    float spdL   = (enc[0].speed_ms + enc[1].speed_ms) / 2.0f;
    float spdR   = (enc[2].speed_ms + enc[3].speed_ms) / 2.0f;
    float distL  = (enc[0].distance_m + enc[1].distance_m) / 2.0f;
    float distR  = (enc[2].distance_m + enc[3].distance_m) / 2.0f;

    String json = "{";
    json += "\"wheel_left_rpm\":"      + String(rpmL,  2) + ",";
    json += "\"wheel_right_rpm\":"     + String(rpmR,  2) + ",";
    json += "\"wheel_left_speed\":"    + String(spdL,  4) + ",";
    json += "\"wheel_right_speed\":"   + String(spdR,  4) + ",";
    json += "\"wheel_left_distance\":" + String(distL, 4) + ",";
    json += "\"wheel_right_distance\":" + String(distR, 4) + ",";
    json += "\"pid_enabled\":"         + String(pidGlobalEnabled ? "true" : "false");
    json += "}";
    server.send(200, "application/json", json);
  });

  // ---- /reset_odometry ----
  server.on("/reset_odometry", HTTP_GET, []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    noInterrupts();
    for (int i = 0; i < 4; i++) enc[i].count = 0;
    interrupts();
    for (int i = 0; i < 4; i++) {
      enc[i].lastSpeedCount = 0;
      enc[i].distance_m     = 0;
      enc[i].rpm            = 0;
      enc[i].speed_ms       = 0;
      enc[i].bufferIndex    = 0;
      for (int j = 0; j < MOVING_AVG_SIZE; j++) enc[i].rpmBuffer[j] = 0;
    }
    Serial.println("Odometry RESET");
    server.send(200, "application/json", "{\"status\":\"odometry_reset\"}");
  });

  // ---- /pid  (GET đọc / GET+params cài đặt) ----
  server.on("/pid", []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");

    int targetWheel = -1;  // -1 = áp cho tất cả
    if (server.hasArg("wheel")) {
      targetWheel = constrain(server.arg("wheel").toInt(), 0, 3);
    }

    auto applyPID = [&](int i) {
      if (server.hasArg("kp"))  wpid[i].kp = server.arg("kp").toFloat();
      if (server.hasArg("ki"))  wpid[i].ki = server.arg("ki").toFloat();
      if (server.hasArg("kd"))  wpid[i].kd = server.arg("kd").toFloat();
      if (server.hasArg("target_rpm")) {
        wpid[i].targetRPM = server.arg("target_rpm").toFloat();
      }
    };

    if (targetWheel >= 0) {
      applyPID(targetWheel);
    } else {
      for (int i = 0; i < 4; i++) applyPID(i);
    }

    // Đặt target RPM nhóm trái/phải hoặc toàn bộ
    if (server.hasArg("target_rpm_l")) {
      targetRPM_Left = server.arg("target_rpm_l").toFloat();
      wpid[0].targetRPM = targetRPM_Left;
      wpid[1].targetRPM = targetRPM_Left;
    }
    if (server.hasArg("target_rpm_r")) {
      targetRPM_Right = server.arg("target_rpm_r").toFloat();
      wpid[2].targetRPM = targetRPM_Right;
      wpid[3].targetRPM = targetRPM_Right;
    }
    if (server.hasArg("target_rpm")) {
      float t = server.arg("target_rpm").toFloat();
      globalTargetRPM = targetRPM_Left = targetRPM_Right = t;
      for (int i = 0; i < 4; i++) wpid[i].targetRPM = t;
    }

    // Bật/tắt PID
    if (server.hasArg("enabled")) {
      String en = server.arg("enabled");
      pidGlobalEnabled = (en == "1" || en == "true");
      for (int i = 0; i < 4; i++) {
        wpid[i].enabled = pidGlobalEnabled;
        if (pidGlobalEnabled) {
          wpid[i].integral      = 0;
          wpid[i].lastError     = 0;
          wpid[i].filteredDeriv = 0;
          wpid[i].pwmOutput     = currentSpeed;
          wpid[i].prevPwmOutput = currentSpeed;
        }
      }
      Serial.println(pidGlobalEnabled ? "PID ENABLED (4 wheels)" : "PID DISABLED");
    }

    // Trả về JSON trạng thái đầy đủ
    String json = "{\"enabled\":" + String(pidGlobalEnabled ? "true" : "false") + ",";
    json += "\"target_rpm_global\":" + String(globalTargetRPM, 1) + ",";
    json += "\"target_rpm_l\":" + String(targetRPM_Left,  1) + ",";
    json += "\"target_rpm_r\":" + String(targetRPM_Right, 1) + ",";
    json += "\"wheels\":[";
    for (int i = 0; i < 4; i++) {
      json += "{\"id\":"         + String(i)                          + ",";
      json += "\"kp\":"          + String(wpid[i].kp,          3)     + ",";
      json += "\"ki\":"          + String(wpid[i].ki,          3)     + ",";
      json += "\"kd\":"          + String(wpid[i].kd,          3)     + ",";
      json += "\"target_rpm\":"  + String(wpid[i].targetRPM,   1)     + ",";
      json += "\"actual_rpm\":"  + String(abs(enc[i].rpm),     1)     + ",";
      json += "\"pwm_output\":"  + String(wpid[i].pwmOutput)          + ",";
      json += "\"slew_current\":" + String((int)slew[i].current)      + "}";
      if (i < 3) json += ",";
    }
    json += "]}";
    server.send(200, "application/json", json);
  });

  // ---- /calibrate ----
  server.on("/calibrate", []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    if (isCalibrating) {
      server.send(409, "application/json", "{\"error\":\"calibration_in_progress\"}");
      return;
    }
    if (server.hasArg("pwm"))      calibPWM      = constrain(server.arg("pwm").toInt(), 50, 255);
    if (server.hasArg("duration")) calibDuration = constrain(server.arg("duration").toInt(), 1000, 10000);

    isCalibrating = true;
    calibStartTime = millis();
    noInterrupts();
    for (int i = 0; i < 4; i++) calibStartCount[i] = enc[i].count;
    interrupts();

    isMoving = true;
    currentDirection = "FORWARD";
    setGroupTargets(calibPWM, calibPWM);

    Serial.printf("=== CALIBRATION START === PWM:%d | %lums\n", calibPWM, calibDuration);
    server.send(200, "application/json",
      "{\"status\":\"calibration_started\",\"pwm\":" + String(calibPWM) +
      ",\"duration_ms\":" + String(calibDuration) + "}");
  });

  // ---- /status ----
  server.on("/status", HTTP_GET, []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    float rpmL = (enc[0].rpm + enc[1].rpm) / 2.0f;
    float rpmR = (enc[2].rpm + enc[3].rpm) / 2.0f;
    bool  accel = false;
    for (int i = 0; i < 4; i++) {
      if (abs(slew[i].current - slew[i].target) > 1.0f) { accel = true; break; }
    }
    String json = "{";
    json += "\"is_moving\":"      + String(isMoving      ? "true" : "false") + ",";
    json += "\"direction\":\""    + currentDirection + "\",";
    json += "\"current_speed\":"  + String(currentSpeed) + ",";
    json += "\"is_accelerating\":" + String(accel         ? "true" : "false") + ",";
    json += "\"is_calibrating\":" + String(isCalibrating  ? "true" : "false") + ",";
    json += "\"pid_enabled\":"    + String(pidGlobalEnabled ? "true" : "false") + ",";
    json += "\"rpm_left\":"       + String(rpmL, 1) + ",";
    json += "\"rpm_right\":"      + String(rpmR, 1) + ",";
    json += "\"turn_ratio\":"     + String(turnInnerRatio, 2);
    json += "}";
    server.send(200, "application/json", json);
  });

  // ---- /control ----
  server.on("/control", []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Access-Control-Allow-Methods", "GET,POST");
    if (server.hasArg("cmd")) {
      String cmd = server.arg("cmd");
      if (server.hasArg("speed")) {
        currentSpeed = constrain(server.arg("speed").toInt(), 0, 255);
        globalTargetRPM = (currentSpeed / 255.0f) * 220.0f;
        for (int i = 0; i < 4; i++) wpid[i].targetRPM = globalTargetRPM;
        Serial.printf("Speed cap nhat: %d (Target RPM: %.1f)\n", currentSpeed, globalTargetRPM);
        if (isMoving && !pidGlobalEnabled) writeSpeed(currentSpeed);
      }
      handleCommand(cmd);
      server.send(200, "text/plain", "OK: " + cmd);
    } else {
      server.send(400, "text/plain", "Thieu tham so cmd");
    }
  });

  // ---- /speed ----
  server.on("/speed", []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    if (server.hasArg("value")) {
      currentSpeed = constrain(server.arg("value").toInt(), 0, 255);
      globalTargetRPM = (currentSpeed / 255.0f) * 220.0f;
      for (int i = 0; i < 4; i++) wpid[i].targetRPM = globalTargetRPM;
      Serial.printf("Speed cap nhat: %d (Target RPM: %.1f)\n", currentSpeed, globalTargetRPM);
      if (isMoving && !pidGlobalEnabled) writeSpeed(currentSpeed);
      server.send(200, "text/plain", "Speed: " + String(currentSpeed) + " (Target RPM: " + String(globalTargetRPM, 1) + ")");
    } else {
      server.send(400, "text/plain", "Thieu tham so value");
    }
  });

  // ---- /turn_ratio ----
  server.on("/turn_ratio", []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    if (server.hasArg("value")) {
      float val = server.arg("value").toFloat();
      turnInnerRatio = constrain(val, 0.0f, 1.0f);
      Serial.printf("Turn ratio: %.0f%%\n", turnInnerRatio * 100);
      if (isMoving && !pidGlobalEnabled &&
          (currentDirection == "LEFT" || currentDirection == "RIGHT")) {
        writeSpeed(currentSpeed);
      }
    }
    server.send(200, "application/json",
                "{\"turn_inner_ratio\":" + String(turnInnerRatio, 2) + "}");
  });

  // ---- /health ----
  server.on("/health", HTTP_GET, []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    String json = "{\"wheels\":[";
    for (int i = 0; i < 4; i++) {
      json += "{\"id\":"       + String(i) + ",";
      json += "\"stalled\":"   + String(wHealth[i].isStalled  ? "true" : "false") + ",";
      json += "\"spinning\":"  + String(wHealth[i].isSpinning ? "true" : "false") + "}";
      if (i < 3) json += ",";
    }
    json += "],\"turn_ratio\":" + String(turnInnerRatio, 2);
    json += ",\"min_pwm\":"     + String(MIN_PWM) + "}";
    server.send(200, "application/json", json);
  });

  // ---- /imu (placeholder) ----
  server.on("/imu", HTTP_GET, []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json",
                "{\"active\":false,\"yaw\":0.0,\"pitch\":0.0,\"roll\":0.0,\"mag_heading\":0.0}");
  });

  // ---- /wheel ----
  server.on("/wheel", HTTP_GET, []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");

    if (!server.hasArg("id")) {
      String json = "{\"wheels\":[";
      const char* pos[4] = {"trai_truoc","trai_sau","phai_truoc","phai_sau"};
      for (int i = 0; i < 4; i++) {
        json += "{";
        json += "\"id\":"          + String(i)                        + ",";
        json += "\"position\":\""  + String(pos[i])                   + "\",";
        json += "\"kp\":"          + String(wpid[i].kp,          4)   + ",";
        json += "\"ki\":"          + String(wpid[i].ki,          4)   + ",";
        json += "\"kd\":"          + String(wpid[i].kd,          4)   + ",";
        json += "\"target_rpm\":"  + String(wpid[i].targetRPM,   1)   + ",";
        json += "\"actual_rpm\":"  + String(abs(enc[i].rpm),     1)   + ",";
        json += "\"error_rpm\":"   + String(wpid[i].targetRPM - abs(enc[i].rpm), 1) + ",";
        json += "\"integral\":"    + String(wpid[i].integral,    2)   + ",";
        json += "\"pid_pwm\":"     + String(wpid[i].pwmOutput)        + ",";
        json += "\"slew_pwm\":"    + String((int)slew[i].current)     + ",";
        json += "\"slew_target\":" + String(slew[i].target)           + ",";
        json += "\"distance_m\":"  + String(enc[i].distance_m,  3)   + ",";
        json += "\"stalled\":"     + String(wHealth[i].isStalled  ? "true":"false") + ",";
        json += "\"spinning\":"    + String(wHealth[i].isSpinning ? "true":"false") + ",";
        json += "\"pid_enabled\":" + String(wpid[i].enabled ? "true":"false");
        json += "}";
        if (i < 3) json += ",";
      }
      json += "]}";
      server.send(200, "application/json", json);
      return;
    }

    int id = constrain(server.arg("id").toInt(), 0, 3);
    const char* pos[4] = {"trai_truoc","trai_sau","phai_truoc","phai_sau"};

    bool changed = false;
    if (server.hasArg("kp"))  { wpid[id].kp = server.arg("kp").toFloat(); changed = true; }
    if (server.hasArg("ki"))  { wpid[id].ki = server.arg("ki").toFloat(); changed = true; }
    if (server.hasArg("kd"))  { wpid[id].kd = server.arg("kd").toFloat(); changed = true; }
    if (server.hasArg("target_rpm")) {
      wpid[id].targetRPM = server.arg("target_rpm").toFloat(); changed = true;
    }
    if (changed) {
      wpid[id].integral      = 0;
      wpid[id].lastError     = 0;
      wpid[id].filteredDeriv = 0;
      Serial.printf("[WHEEL%d] Kp=%.4f Ki=%.4f Kd=%.4f target=%.0f\n",
                    id, wpid[id].kp, wpid[id].ki, wpid[id].kd, wpid[id].targetRPM);
    }

    if (server.hasArg("pwm")) {
      int testPwm = constrain(server.arg("pwm").toInt(), -255, 255);
      wpid[id].enabled = false;
      slew[id].target = testPwm;
      isMoving = true;
      Serial.printf("[WHEEL%d] Test PWM: %d\n", id, testPwm);
    }

    if (server.hasArg("stop") && server.arg("stop") == "1") {
      slew[id].target  = 0;
      slew[id].current = 0;
      wpid[id].integral      = 0;
      wpid[id].lastError     = 0;
      wpid[id].filteredDeriv = 0;
      wpid[id].pwmOutput     = 0;
      wpid[id].prevPwmOutput = 0;
      Serial.printf("[WHEEL%d] Stopped\n", id);
    }

    if (server.hasArg("pid")) {
      String en = server.arg("pid");
      wpid[id].enabled = (en == "1" || en == "true");
      if (wpid[id].enabled) {
        wpid[id].integral      = 0;
        wpid[id].lastError     = 0;
        wpid[id].filteredDeriv = 0;
      }
      Serial.printf("[WHEEL%d] PID %s\n", id, wpid[id].enabled ? "ON" : "OFF");
    }

    String json = "{";
    json += "\"id\":"            + String(id)                          + ",";
    json += "\"position\":\""    + String(pos[id])                     + "\",";
    json += "\"kp\":"            + String(wpid[id].kp,           4)    + ",";
    json += "\"ki\":"            + String(wpid[id].ki,           4)    + ",";
    json += "\"kd\":"            + String(wpid[id].kd,           4)    + ",";
    json += "\"target_rpm\":"    + String(wpid[id].targetRPM,    1)    + ",";
    json += "\"actual_rpm\":"    + String(abs(enc[id].rpm),      1)    + ",";
    json += "\"error_rpm\":"     + String(wpid[id].targetRPM - abs(enc[id].rpm), 1) + ",";
    json += "\"integral\":"      + String(wpid[id].integral,     2)    + ",";
    json += "\"filtered_deriv\":" + String(wpid[id].filteredDeriv, 3)  + ",";
    json += "\"pid_pwm\":"       + String(wpid[id].pwmOutput)          + ",";
    json += "\"slew_pwm\":"      + String((int)slew[id].current)       + ",";
    json += "\"slew_target\":"   + String(slew[id].target)             + ",";
    json += "\"speed_ms\":"      + String(enc[id].speed_ms,      3)    + ",";
    json += "\"distance_m\":"    + String(enc[id].distance_m,    3)    + ",";
    json += "\"enc_count\":"     + String(enc[id].count)               + ",";
    json += "\"stalled\":"       + String(wHealth[id].isStalled  ? "true":"false") + ",";
    json += "\"spinning\":"      + String(wHealth[id].isSpinning ? "true":"false") + ",";
    json += "\"pid_enabled\":"   + String(wpid[id].enabled ? "true":"false");
    json += "}";
    server.send(200, "application/json", json);
  });

  server.begin();
  Serial.println("Server da khoi dong");
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
  server.handleClient();

  updateSpeedRamp();

  if (manualDriveActive && currentDirection != "STOP" && !pidGlobalEnabled) {
    writeSpeed(currentSpeed);
  }

  calculateSpeed();
  updateWheelHealth();
  updatePID();
  updateCalibration();
  printDebugInfo();

  delay(1);
}
