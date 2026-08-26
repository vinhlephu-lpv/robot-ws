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

// Ngưỡng Deadzone & Giới hạn gia tốc PWM
#define MIN_PWM                 75        // Deadzone motor 775 24V có hộp số
#define MAX_PWM_CHANGE          10        // Giới hạn đổi PWM/chu kỳ PID
#define RAMP_STEP_MAX           2.0f      // Bước tăng tốc tối đa (PWM/25ms)
#define RAMP_STEP_MIN           0.3f      // Bước tăng tốc tối thiểu
#define RAMP_STEP_STOP_MAX      1.5f      // Bước giảm tốc bảo vệ hộp số

// Lọc nhiễu Encoder
#define MIN_ENC_INTERVAL_US     200       // Lọc xung nhiễu điện từ < 200us
#define MAX_PLAUSIBLE_RPM       350.0f    // Giới hạn vật lý lọc đột biến RPM
#define MOVING_AVG_SIZE         4         // Số mẫu trung bình trượt RPM
#define DERIVATIVE_FILTER       0.6f      // Hệ số lọc thông thấp vi phân PID

// Phát hiện kẹt bánh (Stall Detection)
#define STALL_DETECT_MS         600
#define STALL_PWM_THRESHOLD     90
#define STALL_RPM_THRESHOLD     8.0f

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
#define DRV4_RPWM   20   // Bánh 3: Phải sau
#define DRV4_LPWM   21

// Cờ đảo chiều driver (+1: bình thường, -1: đảo chiều nếu đấu ngược dây)
#define INV_DRV1    1
#define INV_DRV2    1
#define INV_DRV3    1
#define INV_DRV4    1

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
  float speed_ms;
  float distance_m;
  float rpmBuffer[MOVING_AVG_SIZE];
  int   bufferIndex;
};

// Sơ đồ 4 Encoder: Trái trước (16,17), Trái sau (38,39), Phải trước (10,11), Phải sau (40,41)
EncoderData enc[4] = {
  {16, 17, 1, 0, 0, 0, 0, 0, {0}, 0}, // enc[0]: Bánh trái trước
  {38, 39, 1, 0, 0, 0, 0, 0, {0}, 0}, // enc[1]: Bánh trái sau
  {10, 11, 1, 0, 0, 0, 0, 0, {0}, 0}, // enc[2]: Bánh phải trước
  {40, 41, 1, 0, 0, 0, 0, 0, {0}, 0}  // enc[3]: Bánh phải sau
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

// Hệ số PID từng bánh + Bù đồng tốc chéo
#define PID0_KP 0.650f, PID0_KI 0.850f, PID0_KD 0.040f // Trái trước
#define PID1_KP 0.750f, PID1_KI 1.050f, PID1_KD 0.050f // Trái sau
#define PID2_KP 0.650f, PID2_KI 0.850f, PID2_KD 0.040f // Phải trước
#define PID3_KP 0.750f, PID3_KI 1.050f, PID3_KD 0.050f // Phải sau
#define K_SYNC_CROSS_WHEEL 0.35f                       // Hệ số bù lệch tốc chéo

WheelPID wpid[4] = {
  {0.650f, 0.850f, 0.040f, 0, 0, 0, 0, 0, 0, false},
  {0.750f, 1.050f, 0.050f, 0, 0, 0, 0, 0, 0, false},
  {0.650f, 0.850f, 0.040f, 0, 0, 0, 0, 0, 0, false},
  {0.750f, 1.050f, 0.050f, 0, 0, 0, 0, 0, 0, false}
};

struct SlewChannel {
  int   target;                     // Tốc độ mục tiêu (-255 đến +255)
  float current;                    // Tốc độ hiện tại mượt mà
  float step;                       // Bước ramp động
};

SlewChannel slew[4] = {};

struct WheelHealth {
  bool isStalled;
  unsigned long stallStartTime;
};
WheelHealth wHealth[4] = {};

// Biến điều khiển xe
int           currentSpeed      = 255;
bool          isMoving          = false;
bool          manualDriveActive = false;
bool          pidGlobalEnabled  = false;
float         globalTargetRPM   = 100.0f;
String        currentDirection  = "STOP";
unsigned long lastSpeedCalcTime  = 0;
unsigned long lastRampTime       = 0;
unsigned long lastDebugPrintTime = 0;
volatile unsigned long lastEncTime[4] = {0, 0, 0, 0};

// ============================================================
//  5. CÁC HÀM NGẮT ENCODER (ISR CÓ LỌC GAI NHIỄU IRAM)
// ============================================================
void IRAM_ATTR isr_enc0() {
  unsigned long now = micros();
  if (now - lastEncTime[0] < MIN_ENC_INTERVAL_US) return;
  lastEncTime[0] = now;
  enc[0].count += (digitalRead(enc[0].pinB) > 0) ? enc[0].sign : -enc[0].sign;
}

void IRAM_ATTR isr_enc1() {
  unsigned long now = micros();
  if (now - lastEncTime[1] < MIN_ENC_INTERVAL_US) return;
  lastEncTime[1] = now;
  enc[1].count += (digitalRead(enc[1].pinB) > 0) ? enc[1].sign : -enc[1].sign;
}

void IRAM_ATTR isr_enc2() {
  unsigned long now = micros();
  if (now - lastEncTime[2] < MIN_ENC_INTERVAL_US) return;
  lastEncTime[2] = now;
  enc[2].count += (digitalRead(enc[2].pinB) > 0) ? enc[2].sign : -enc[2].sign;
}

void IRAM_ATTR isr_enc3() {
  unsigned long now = micros();
  if (now - lastEncTime[3] < MIN_ENC_INTERVAL_US) return;
  lastEncTime[3] = now;
  enc[3].count += (digitalRead(enc[3].pinB) > 0) ? enc[3].sign : -enc[3].sign;
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

// ============================================================
//  7. TÍNH TOÁN VẬN TỐC & SỨC KHỎE BÁNH XE
// ============================================================
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

    float rawRPM = (pulses * 60.0f) / (ENCODER_PPR * dt);
    if (abs(rawRPM) > MAX_PLAUSIBLE_RPM) rawRPM = 0;

    enc[i].rpmBuffer[enc[i].bufferIndex] = rawRPM;
    enc[i].bufferIndex = (enc[i].bufferIndex + 1) % MOVING_AVG_SIZE;
    enc[i].rpm = calculateMovingAverage(enc[i].rpmBuffer, MOVING_AVG_SIZE);

    enc[i].speed_ms   = enc[i].rpm * WHEEL_CIRCUMFERENCE / 60.0f;
    enc[i].distance_m += (abs(pulses) * WHEEL_CIRCUMFERENCE) / ENCODER_PPR;
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

  if (slew[0].target < 0) v_left  = -abs(v_left);
  else if (slew[0].target > 0) v_left = abs(v_left);
  else if (abs(enc[0].rpm) < 1.0f && abs(enc[1].rpm) < 1.0f) v_left = 0.0f;

  if (slew[2].target < 0) v_right = -abs(v_right);
  else if (slew[2].target > 0) v_right = abs(v_right);
  else if (abs(enc[2].rpm) < 1.0f && abs(enc[3].rpm) < 1.0f) v_right = 0.0f;

  Serial.printf("ODOM %.3f %.3f\n", v_left, v_right);
}

void updateWheelHealth() {
  if (!isMoving) {
    for (int i = 0; i < 4; i++) wHealth[i] = {false, 0};
    return;
  }
  unsigned long now = millis();
  for (int i = 0; i < 4; i++) {
#if WHEEL3_ENCODER_FAULT
    if (i == 3) { wHealth[3] = wHealth[2]; continue; }
#endif
    float rpm = abs(enc[i].rpm);
    int   pwm = abs(slew[i].target);

    if (pwm > STALL_PWM_THRESHOLD && rpm < STALL_RPM_THRESHOLD) {
      if (wHealth[i].stallStartTime == 0) wHealth[i].stallStartTime = now;
      if (now - wHealth[i].stallStartTime > STALL_DETECT_MS && !wHealth[i].isStalled) {
        wHealth[i].isStalled = true;
        Serial.printf("[HEALTH] CANH BAO: Banh %d bi KHOA!\n", i + 1);
      }
    } else {
      wHealth[i].stallStartTime = 0;
      wHealth[i].isStalled = false;
    }
  }
}

// ============================================================
//  8. THUẬT TOÁN PID + FEEDFORWARD + CROSS-COUPLING SYNC
// ============================================================
void updatePID() {
  if (!pidGlobalEnabled || !isMoving) return;

  float dt = SPEED_CALC_INTERVAL_MS / 1000.0f;
  float r0 = abs(enc[0].rpm), r1 = abs(enc[1].rpm);
  float r2 = abs(enc[2].rpm), r3 = abs(enc[3].rpm);

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
      wpid[i].pwmOutput = 0; wpid[i].integral = 0; slew[i].target = 0;
      continue;
    }

    // Feedforward PWM cơ sở
    float ff_pwm = (target / 220.0f) * (255.0f - MIN_PWM) + MIN_PWM;

    // Positional PID
    float error = target - actualRPM;
    wpid[i].integral = constrain(wpid[i].integral + error * dt, -35.0f, 35.0f);
    float rawDeriv = (error - wpid[i].lastError) / dt;
    wpid[i].filteredDeriv = DERIVATIVE_FILTER * rawDeriv + (1.0f - DERIVATIVE_FILTER) * wpid[i].filteredDeriv;
    wpid[i].lastError = error;

    float pid_corr = (wpid[i].kp * error) + (wpid[i].ki * wpid[i].integral) + (wpid[i].kd * wpid[i].filteredDeriv);

    // Cross-Coupling Đồng tốc
    float sync_avg  = (currentDirection == "FORWARD" || currentDirection == "BACKWARD") ? avgTotal : (isLeft ? avgLeft : avgRight);
    float sync_corr = (sync_avg - actualRPM) * K_SYNC_CROSS_WHEEL;

    int desired = constrain((int)(ff_pwm + pid_corr + sync_corr), 0, 255);
    int delta   = constrain(desired - wpid[i].prevPwmOutput, -MAX_PWM_CHANGE, MAX_PWM_CHANGE);
    wpid[i].pwmOutput     = constrain(wpid[i].prevPwmOutput + delta, 0, 255);
    wpid[i].prevPwmOutput = wpid[i].pwmOutput;

    if (wpid[i].pwmOutput > 0 && wpid[i].pwmOutput < MIN_PWM) wpid[i].pwmOutput = MIN_PWM;
    if (wHealth[i].isStalled) { wpid[i].pwmOutput = max(0, wpid[i].pwmOutput - 30); wpid[i].integral = 0; }

    // Quy tắc chiều quay 4 bánh (Tank Drive)
    int sign = 0;
    if (currentDirection == "FORWARD")       sign = 1;               // Trái (+), Phải (+)
    else if (currentDirection == "BACKWARD") sign = -1;              // Trái (-), Phải (-)
    else if (currentDirection == "LEFT")     sign = isLeft ? -1 : 1; // Trái (-), Phải (+) -> Xoay trái
    else if (currentDirection == "RIGHT")    sign = isLeft ? 1 : -1; // Trái (+), Phải (-) -> Xoay phải

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
  if (currentDirection == "FORWARD")       setGroupTargets(speed, speed);
  else if (currentDirection == "BACKWARD") setGroupTargets(-speed, -speed);
  else if (currentDirection == "LEFT")     setGroupTargets(-speed, speed);
  else if (currentDirection == "RIGHT")    setGroupTargets(speed, -speed);
  else                                     setGroupTargets(0, 0);
}

void stopMotor() {
  manualDriveActive = false;
  currentDirection  = "STOP";
  for (int i = 0; i < 4; i++) {
    slew[i].target = 0; slew[i].current = 0.0f;
    wpid[i].integral = 0; wpid[i].lastError = 0; wpid[i].filteredDeriv = 0;
    wpid[i].pwmOutput = 0; wpid[i].prevPwmOutput = 0;
    wHealth[i] = {false, 0};
  }
  writeAllDrives(0, 0, 0, 0);
  isMoving = false;
}

void handleCommand(String command) {
  command.trim();
  if (command.length() == 0) return;

  // Lệnh vận tốc ROS 2: "V <rpm_L> <rpm_R>" hoặc "v <rpm_L> <rpm_R>"
  if (command.startsWith("V ") || command.startsWith("v ") || command.startsWith("V\t") || command.startsWith("v\t")) {
    float r_l = 0.0f, r_r = 0.0f;
    int parsed = sscanf(command.c_str() + 2, "%f %f", &r_l, &r_r);
    if (parsed == 2) {
      if (abs(r_l) < 0.1f && abs(r_r) < 0.1f) {
        isMoving = false;
        currentDirection = "STOP";
        setGroupTargets(0, 0);
        for (int i = 0; i < 4; i++) wpid[i].targetRPM = 0.0f;
      } else {
        isMoving = true;
        manualDriveActive = true;
        currentDirection = "ROS";
        // Chuyển đổi RPM -> PWM (-255 đến 255) theo tỉ lệ motor 220 RPM
        int pwmL = (int)constrain((r_l / 220.0f) * 255.0f, -255.0f, 255.0f);
        int pwmR = (int)constrain((r_r / 220.0f) * 255.0f, -255.0f, 255.0f);
        setGroupTargets(pwmL, pwmR);

        wpid[0].targetRPM = abs(r_l);
        wpid[1].targetRPM = abs(r_l);
        wpid[2].targetRPM = abs(r_r);
        wpid[3].targetRPM = abs(r_r);
      }
      return;
    }
  }

  command.toUpperCase();

  // Chuẩn hóa tên lệnh
  if (command == "FORWARD" || command == "TIEN" || command == "F" || command == "W")      command = "FORWARD";
  else if (command == "BACKWARD" || command == "LUI" || command == "B" || command == "S")  command = "BACKWARD";
  else if (command == "LEFT" || command == "TRAI" || command == "L" || command == "A")    command = "LEFT";
  else if (command == "RIGHT" || command == "PHAI" || command == "R" || command == "D")   command = "RIGHT";
  else if (command == "STOP" || command == "DUNG" || command == "X" || command == "SPACE") command = "STOP";
  else if (command == "EMERGENCY_STOP" || command == "ESTOP")                              command = "EMERGENCY_STOP";

  if (command == "FORWARD" || command == "BACKWARD" || command == "LEFT" || command == "RIGHT") {
    isMoving          = true;
    manualDriveActive = true;
    currentDirection  = command;

    if (pidGlobalEnabled) {
      for (int i = 0; i < 4; i++) wpid[i].targetRPM = globalTargetRPM;
      Serial.printf("PID move: %s | Target: %.1f RPM\n", command.c_str(), globalTargetRPM);
    } else {
      writeSpeed(currentSpeed);
      Serial.println("Chay: " + command + " | PWM: " + String(currentSpeed));
    }
  } else if (command == "STOP") {
    isMoving = false;
    currentDirection = "STOP";
    setGroupTargets(0, 0);
  } else if (command == "EMERGENCY_STOP") {
    stopMotor();
  }
}

// ============================================================
//  11. IN DEBUG SERIAL MONITOR (500ms)
// ============================================================
void printDebugInfo() {
  unsigned long now = millis();
  if (now - lastDebugPrintTime < DEBUG_PRINT_INTERVAL_MS) return;
  lastDebugPrintTime = now;

  bool active = isMoving;
  for (int i = 0; i < 4; i++) if (enc[i].rpm != 0) active = true;
  if (!active) return;

  Serial.println("----");
  for (int i = 0; i < 4; i++) {
    Serial.printf("[W%d] RPM:%6.1f | Spd:%.3fm/s | Dist:%.3fm | PWM:%4d",
                  i + 1, enc[i].rpm, enc[i].speed_ms, enc[i].distance_m, slew[i].target);
    if (pidGlobalEnabled && wpid[i].enabled) {
      Serial.printf(" | PID_tgt:%.0f | PID_pwm:%d", wpid[i].targetRPM, wpid[i].pwmOutput);
    }
    if (wHealth[i].isStalled) Serial.print(" [KHOA]");
    Serial.println();
  }
}

// ============================================================
//  12. SETUP & WEBSERVER CỐT LÕI
// ============================================================
void setup() {
  // 1. Ép cứng mức 0V ngay dòng đầu tiên
  lockAllDriverPins();

  Serial.begin(115200);
  Serial.setTimeout(10);

  // Tắt LED RGB chân 48 ESP32-S3
#if defined(RGB_LED_PIN)
  neopixelWrite(RGB_LED_PIN, 0, 0, 0);
#endif

  // Khởi tạo PWM 4 driver
  pwmSetup(DRV1_RPWM, CH_DRV1_F); pwmSetup(DRV1_LPWM, CH_DRV1_R);
  pwmSetup(DRV2_RPWM, CH_DRV2_F); pwmSetup(DRV2_LPWM, CH_DRV2_R);
  pwmSetup(DRV3_RPWM, CH_DRV3_F); pwmSetup(DRV3_LPWM, CH_DRV3_R);
  pwmSetup(DRV4_RPWM, CH_DRV4_F); pwmSetup(DRV4_LPWM, CH_DRV4_R);
  writeAllDrives(0, 0, 0, 0);

  // Khởi tạo ngắt 4 Encoder
  void (*isrFn[4])() = {isr_enc0, isr_enc1, isr_enc2, isr_enc3};
  for (int i = 0; i < 4; i++) {
    pinMode(enc[i].pinA, INPUT_PULLUP);
    pinMode(enc[i].pinB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(enc[i].pinA), isrFn[i], RISING);
  }

  Serial.println("\n=== XE TU HANH 4 BANH PID (ROBOT CONTROLLER) ===");
  Serial.printf("Encoder: %d PPR | Banh: %.0fmm | MIN_PWM: %d\n",
                ENCODER_PPR, WHEEL_DIAMETER_M * 1000, MIN_PWM);

  // Khởi động WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Dang ket noi WiFi");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 25) {
    writeAllDrives(0, 0, 0, 0);
    delay(500); Serial.print("."); attempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi OK! IP: " + WiFi.localIP().toString());
  } else {
    Serial.println("\nKhong co WiFi. Xe san sang qua Serial!");
  }

  // --- API WebServer ---
  server.enableCORS(true);

  server.on("/ping", HTTP_GET, []() {
    server.send(200, "text/plain", "ESP32 4-Wheel PID - IP: " + WiFi.localIP().toString());
  });

  server.on("/control", []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    if (server.hasArg("cmd")) {
      if (server.hasArg("speed")) {
        currentSpeed = constrain(server.arg("speed").toInt(), 0, 255);
        globalTargetRPM = (currentSpeed / 255.0f) * 220.0f;
        for (int i = 0; i < 4; i++) wpid[i].targetRPM = globalTargetRPM;
      }
      handleCommand(server.arg("cmd"));
      server.send(200, "text/plain", "OK: " + server.arg("cmd"));
    } else {
      server.send(400, "text/plain", "Missing cmd");
    }
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

  // Đọc lệnh trực tiếp từ Serial Monitor
  if (Serial.available()) {
    String serialCmd = Serial.readStringUntil('\n');
    handleCommand(serialCmd);
  }

  updateSpeedRamp();

  if (manualDriveActive && currentDirection != "STOP" && currentDirection != "ROS" && !pidGlobalEnabled) {
    writeSpeed(currentSpeed);
  }

  calculateSpeed();
  updateWheelHealth();
  updatePID();
  printDebugInfo();

  delay(1);
}
