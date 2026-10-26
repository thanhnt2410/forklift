// esp32_hardware_test.ino — TEST PID & TĂNG GIẢM TỐC ĐỘ 4 BÁNH XE MECANUM
//
// ĐỘC LẬP 100%: không include file ngoài, chỉ dùng core Arduino ESP32.
// Nạp trực tiếp bằng Arduino IDE (board: ESP32 Dev Module, core 2.x hoặc 3.x).
//
// THÔNG SỐ ĐÃ TỐI ƯU:
//   - PID êm mượt: Kp = 0.04 | Ki = 0.08 | Kd = 0.00
//   - Chu kỳ PID: 10 ms (100 Hz), CPR: 937.2, Max duty: 50%, Trần: 10 rad/s.
//   - Bánh FL: PWM {25, 26} | ENC {34, 35} | enc_sign =  1 | mot_sign =  1
//   - Bánh FR: PWM {18, 19} | ENC { 2, 14} | enc_sign =  1 | mot_sign = -1
//   - Bánh RL: PWM {23, 22} | ENC { 5, 17} | enc_sign =  1 | mot_sign =  1
//   - Bánh RR: PWM {21, 27} | ENC {16,  4} | enc_sign = -1 | mot_sign = -1
//
// LỆNH ĐIỀU KHIỂN (115200 BAUD):
//   - FL / FR / RL / RR <rad/s> : Quay riêng từng bánh (vd: FR 2.5)
//   - ALL <rad/s>               : Quay cùng lúc cả 4 bánh
//   - +                         : Tăng tốc độ thêm +0.5 rad/s
//   - -                         : Giảm tốc độ đi -0.5 rad/s
//   - <số>                      : Nhập trực tiếp tốc độ mới (vd: gõ 3.0 hoặc -2.0)
//   - STOP                      : Dừng khẩn cấp toàn bộ bánh
//   - PID <p> <i> <d>           : Chỉnh trực tiếp thông số PID
//   - ?                         : Xem menu trợ giúp

#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

// ============================ CẤU HÌNH CHÂN ============================
static const int8_t  PIN_RPWM[4]  = {25, 18, 23, 21};   // RPWM BTS7960
static const int8_t  PIN_LPWM[4]  = {26, 19, 22, 27};   // LPWM BTS7960
static const int8_t  PIN_ENC_A[4] = {34,  2,  5, 16};
static const int8_t  PIN_ENC_B[4] = {35, 14, 17,  4};
static const int8_t  ENC_SIGN[4]  = { 1,  1,  1, -1};   // Dấu encoder
static const int8_t  MOT_SIGN[4]  = { 1, -1,  1, -1};   // Dấu động cơ
static const char *const WHEEL_NAME[4] = {"FL", "FR", "RL", "RR"};

// ========================== THÔNG SỐ ĐIỀU KHIỂN =========================
static const float   WHEEL_CPR       = 937.2f;          // CPR danh nghĩa GA25-370
static const float   TAU_F           = 6.28318530718f;  // 2*PI
static const float   MAX_RAD_S       = 10.0f;           // Giới hạn tốc độ góc tối đa
static const float   MAX_DUTY        = 0.50f;           // Giới hạn 50% công suất để an toàn

// Bộ thông số chuẩn đã test mượt mà không rung:
static const float   KP_INIT = 0.032f, KI_INIT = 0.16f, KD_INIT = 0.f;

static const uint32_t PID_PERIOD_US   = 10000;          // 100 Hz (10ms)
static const uint32_t PRINT_PERIOD_US = 50000;          // 20 Hz (50ms)

static const uint32_t PWM_FREQ   = 20000;               // 20 kHz
static const uint8_t  PWM_RES    = 8;                   // 8-bit
static const uint32_t PWM_MAXVAL = 255;

// ========================== TRẠNG THÁI BÁNH XE ==========================
static volatile int32_t encTicks[4]     = {0, 0, 0, 0};
static uint8_t          prevEncState[4] = {0, 0, 0, 0};

static float    wheelSetpoint[4] = {0, 0, 0, 0};
static float    actualVel[4]     = {0, 0, 0, 0};        // Vận tốc góc đo được (rad/s)
static int32_t  lastPidTicks[4]  = {0, 0, 0, 0};
static float    prevMeasured[4]  = {0, 0, 0, 0};
static float    integTerm[4]     = {0, 0, 0, 0};
static float    dutyOut[4]       = {0, 0, 0, 0};

static float    kp = KP_INIT, ki = KI_INIT, kd = KD_INIT;
static int8_t   activeWheel = -1;                       // -1: tắt hết, 0..3: đơn bánh, 4: ALL

// ============================== TIỆN ÍCH ===============================
static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline float fabsf_(float v) { return v < 0.0f ? -v : v; }

static void setWheelDuty(int i, float dutySigned) {
  float d = clampf(fabsf_(dutySigned), 0.0f, 1.0f);
  uint32_t pwmVal = (uint32_t)(d * PWM_MAXVAL);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  if (dutySigned > 0.0001f) {
    ledcWrite(PIN_RPWM[i], pwmVal);
    ledcWrite(PIN_LPWM[i], 0);
  } else if (dutySigned < -0.0001f) {
    ledcWrite(PIN_RPWM[i], 0);
    ledcWrite(PIN_LPWM[i], pwmVal);
  } else {
    ledcWrite(PIN_RPWM[i], 0);
    ledcWrite(PIN_LPWM[i], 0);
  }
#else
  int chR = i * 2;
  int chL = i * 2 + 1;
  if (dutySigned > 0.0001f) {
    ledcWrite(chR, pwmVal);
    ledcWrite(chL, 0);
  } else if (dutySigned < -0.0001f) {
    ledcWrite(chR, 0);
    ledcWrite(chL, pwmVal);
  } else {
    ledcWrite(chR, 0);
    ledcWrite(chL, 0);
  }
#endif
  dutyOut[i] = dutySigned;
}

// ============================ ENCODER (ISR) ============================
static const int8_t QUAD_LUT[16] = {
  0, -1, +1,  0,
 +1,  0,  0, -1,
 -1,  0,  0, +1,
  0, +1, -1,  0
};

static inline void IRAM_ATTR quadDecode(int i) {
  uint8_t st = (uint8_t)((digitalRead(PIN_ENC_A[i]) ? 1u : 0u) |
                         (digitalRead(PIN_ENC_B[i]) ? 2u : 0u));
  uint8_t prev = prevEncState[i];
  if (st != prev) {
    encTicks[i] += (int32_t)QUAD_LUT[(prev << 2) | st] * (int32_t)ENC_SIGN[i];
    prevEncState[i] = st;
  }
}

static void IRAM_ATTR isrFL_A() { quadDecode(0); }
static void IRAM_ATTR isrFL_B() { quadDecode(0); }
static void IRAM_ATTR isrFR_A() { quadDecode(1); }
static void IRAM_ATTR isrFR_B() { quadDecode(1); }
static void IRAM_ATTR isrRL_A() { quadDecode(2); }
static void IRAM_ATTR isrRL_B() { quadDecode(2); }
static void IRAM_ATTR isrRR_A() { quadDecode(3); }
static void IRAM_ATTR isrRR_B() { quadDecode(3); }

// ======================== PID VẬN TỐC BÁNH XE ==========================
static void pidStepWheel(int i, float dt) {
  if (activeWheel != 4 && i != activeWheel) {
    integTerm[i]    = 0.0f;
    prevMeasured[i] = 0.0f;
    actualVel[i]    = 0.0f;
    lastPidTicks[i] = encTicks[i];
    setWheelDuty(i, 0.0f);
    return;
  }

  int32_t currentTicks = encTicks[i];
  int32_t deltaTicks   = currentTicks - lastPidTicks[i];
  lastPidTicks[i]      = currentTicks;

  float rawMeas = ((float)deltaTicks / WHEEL_CPR) * TAU_F / dt;
  actualVel[i] += (rawMeas - actualVel[i]) * dt / (0.03f + dt);
  float meas = actualVel[i];

  float sp    = clampf(wheelSetpoint[i], -MAX_RAD_S, MAX_RAD_S);
  float e     = sp - meas;
  float dMeas = (meas - prevMeasured[i]) / dt;
  prevMeasured[i] = meas;

  float u        = kp * e + integTerm[i] - kd * dMeas;
  float uClamped = clampf(u, -MAX_DUTY, MAX_DUTY);

  bool satHi = (u > uClamped);
  bool satLo = (u < uClamped);
  bool helps = (satHi && e < 0.0f) || (satLo && e > 0.0f);
  if (!satHi && !satLo) integTerm[i] += ki * e * dt;
  else if (helps)       integTerm[i] += ki * e * dt;
  integTerm[i] = clampf(integTerm[i], -MAX_DUTY, MAX_DUTY);

  setWheelDuty(i, clampf(uClamped * MOT_SIGN[i], -MAX_DUTY, MAX_DUTY));
}

// ============================== LỆNH SERIAL ============================
static void printMenu() {
  Serial.println(F("\n# ---------------- MENU ĐIỀU KHIỂN & CHỈNH TỐC ĐỘ ----------------"));
  Serial.println(F("# FL / FR / RL / RR <v> : Chọn bánh & tốc độ (vd: FR 2.5)"));
  Serial.println(F("# ALL <v>               : Chạy cùng lúc cả 4 bánh"));
  Serial.println(F("# +                     : Tăng tốc bánh hiện tại thêm +0.5 rad/s"));
  Serial.println(F("# -                     : Giảm tốc bánh hiện tại đi -0.5 rad/s"));
  Serial.println(F("# <con_số>              : Nhập trực tiếp tốc độ mới (vd: gõ 3.0 hoặc -2.0)"));
  Serial.println(F("# STOP                  : Dừng khẩn cấp toàn bộ bánh"));
  Serial.println(F("# PID p i d             : Đổi thông số PID (vd:   )"));
  Serial.println(F("# ?                     : Xem lại menu này"));
  Serial.println(F("# ----------------------------------------------------------------\n"));
}

static void selectSingleWheel(int idx, float rads) {
  for (int i = 0; i < 4; i++) {
    wheelSetpoint[i] = 0.0f;
    integTerm[i]     = 0.0f;
    prevMeasured[i]  = 0.0f;
    setWheelDuty(i, 0.0f);
    lastPidTicks[i]  = encTicks[i];
  }
  wheelSetpoint[idx] = rads;
  activeWheel = (int8_t)idx;
}

static void selectAllWheels(float rads) {
  for (int i = 0; i < 4; i++) {
    wheelSetpoint[i] = rads;
    integTerm[i]     = 0.0f;
    prevMeasured[i]  = 0.0f;
    lastPidTicks[i]  = encTicks[i];
  }
  activeWheel = 4;
}

static void allWheelsOff() {
  for (int i = 0; i < 4; i++) {
    wheelSetpoint[i] = 0.0f;
    integTerm[i]     = 0.0f;
    prevMeasured[i]  = 0.0f;
    setWheelDuty(i, 0.0f);
  }
  activeWheel = -1;
}

// Tăng / giảm tốc độ bánh đang chạy
static void adjustActiveSpeed(float delta) {
  if (activeWheel == -1) {
    Serial.println(F("# [!] Chưa chọn bánh nào! Hãy chọn trước (vd: FR 2.5 hoặc ALL 2.5)"));
    return;
  }
  if (activeWheel == 4) { // ALL
    float newV = clampf(wheelSetpoint[0] + delta, -MAX_RAD_S, MAX_RAD_S);
    for (int i = 0; i < 4; i++) wheelSetpoint[i] = newV;
    Serial.printf("# [ALL] Đã chỉnh tốc độ -> %.2f rad/s\n", newV);
  } else {
    float newV = clampf(wheelSetpoint[activeWheel] + delta, -MAX_RAD_S, MAX_RAD_S);
    wheelSetpoint[activeWheel] = newV;
    Serial.printf("# [%s] Đã chỉnh tốc độ -> %.2f rad/s\n", WHEEL_NAME[activeWheel], newV);
  }
}

// Gán trực tiếp tốc độ mới cho bánh đang chạy
static void setActiveSpeed(float newV) {
  if (activeWheel == -1) {
    Serial.println(F("# [!] Chưa chọn bánh nào! Hãy chọn trước (vd: FR 2.5 hoặc ALL 2.5)"));
    return;
  }
  newV = clampf(newV, -MAX_RAD_S, MAX_RAD_S);
  if (activeWheel == 4) {
    for (int i = 0; i < 4; i++) wheelSetpoint[i] = newV;
    Serial.printf("# [ALL] Tốc độ mới: %.2f rad/s\n", newV);
  } else {
    wheelSetpoint[activeWheel] = newV;
    Serial.printf("# [%s] Tốc độ mới: %.2f rad/s\n", WHEEL_NAME[activeWheel], newV);
  }
}

static void handleLine(char *s) {
  char *tok = strtok(s, " ,\t\r\n");
  if (!tok) return;

  if (!strcasecmp(tok, "?") || !strcasecmp(tok, "HELP")) {
    printMenu();
    return;
  }
  if (!strcasecmp(tok, "STOP")) {
    allWheelsOff();
    Serial.println(F("# [STOP] ĐÃ DỪNG TOÀN BỘ BÁNH XE."));
    return;
  }

  // Phím tắt tăng / giảm tốc
  if (!strcmp(tok, "+") || !strcasecmp(tok, "UP")) {
    adjustActiveSpeed(0.5f);
    return;
  }
  if (!strcmp(tok, "-") || !strcasecmp(tok, "DN") || !strcasecmp(tok, "DOWN")) {
    adjustActiveSpeed(-0.5f);
    return;
  }

  // Đổi PID
  if (!strcasecmp(tok, "PID")) {
    char *a1 = strtok(nullptr, " ,\t\r\n");
    char *a2 = strtok(nullptr, " ,\t\r\n");
    char *a3 = strtok(nullptr, " ,\t\r\n");
    if (!a1 || !a2 || !a3) { Serial.println(F("# Lỗi cú pháp: PID <p> <i> <d>")); return; }
    kp = (float)atof(a1); ki = (float)atof(a2); kd = (float)atof(a3);
    for (int i = 0; i < 4; i++) { integTerm[i] = 0.0f; prevMeasured[i] = 0.0f; }
    Serial.printf("# PID mới: Kp=%.4f Ki=%.4f Kd=%.4f\n", kp, ki, kd);
    return;
  }

  // Chạy cả 4 bánh
  if (!strcasecmp(tok, "ALL")) {
    char *a1 = strtok(nullptr, " ,\t\r\n");
    if (!a1) { Serial.println(F("# Lỗi: ALL <rad/s>")); return; }
    float v = clampf((float)atof(a1), -MAX_RAD_S, MAX_RAD_S);
    selectAllWheels(v);
    Serial.printf("# CHẠY CẢ 4 BÁNH: Setpoint = %.2f rad/s\n", v);
    return;
  }

  // Kiểm tra tên bánh (FL, FR, RL, RR)
  int idx = -1;
  for (int i = 0; i < 4; i++) if (!strcasecmp(tok, WHEEL_NAME[i])) idx = i;
  if (idx >= 0) {
    char *a1 = strtok(nullptr, " ,\t\r\n");
    if (!a1) { Serial.printf("# Lỗi: %s <rad/s>\n", WHEEL_NAME[idx]); return; }
    float v = clampf((float)atof(a1), -MAX_RAD_S, MAX_RAD_S);
    selectSingleWheel(idx, v);
    Serial.printf("# FOCUS [%s]: Setpoint = %.2f rad/s (các bánh khác đã tắt)\n", WHEEL_NAME[idx], v);
    return;
  }

  // Kiểm tra nếu người dùng gõ trực tiếp một con số (vd: "3.2" hoặc "-1.5")
  char *endptr = nullptr;
  float directVal = strtof(tok, &endptr);
  if (endptr != tok && *endptr == '\0') {
    setActiveSpeed(directVal);
    return;
  }

  Serial.printf("# Lệnh không đúng: %s (Gõ ? để xem menu)\n", tok);
}

// ================================ SETUP ===============================
void setup() {
  Serial.begin(115200);
  delay(300);

  // 1. Cấu hình PWM
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  for (int i = 0; i < 4; i++) {
    ledcAttach(PIN_RPWM[i], PWM_FREQ, PWM_RES);
    ledcAttach(PIN_LPWM[i], PWM_FREQ, PWM_RES);
    ledcWrite(PIN_RPWM[i], 0);
    ledcWrite(PIN_LPWM[i], 0);
  }
#else
  int ch = 0;
  for (int i = 0; i < 4; i++) {
    ledcSetup(ch, PWM_FREQ, PWM_RES); ledcAttachPin(PIN_RPWM[i], ch);
    ledcSetup(ch + 1, PWM_FREQ, PWM_RES); ledcAttachPin(PIN_LPWM[i], ch + 1);
    ledcWrite(ch, 0); ledcWrite(ch + 1, 0);
    ch += 2;
  }
#endif

  // 2. Cấu hình Encoder
  for (int i = 0; i < 4; i++) {
    pinMode(PIN_ENC_A[i], (PIN_ENC_A[i] >= 34) ? INPUT : INPUT_PULLUP);
    pinMode(PIN_ENC_B[i], (PIN_ENC_B[i] >= 34) ? INPUT : INPUT_PULLUP);
    prevEncState[i] = (uint8_t)((digitalRead(PIN_ENC_A[i]) ? 1u : 0u) |
                                (digitalRead(PIN_ENC_B[i]) ? 2u : 0u));
    lastPidTicks[i] = 0;
  }

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A[0]), isrFL_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B[0]), isrFL_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A[1]), isrFR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B[1]), isrFR_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A[2]), isrRL_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B[2]), isrRL_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A[3]), isrRR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B[3]), isrRR_B, CHANGE);

  Serial.println(F("\n========================================================"));
  Serial.println(F("# ESP32 WHEEL PID TEST TOOL SẴN SÀNG"));
  Serial.printf("# Đã nạp thông số mượt: Kp=%.3f Ki=%.3f Kd=%.3f\n", kp, ki, kd);
  Serial.println(F("========================================================"));
  printMenu();
}

// ================================ LOOP ================================
static char     lineBuf[80];
static uint8_t  lineLen = 0;
static uint32_t lastPidUs   = 0;
static uint32_t lastPrintUs = 0;

void loop() {
  uint32_t now = micros();

  // 1. Nhận lệnh Serial (Non-blocking)
  while (Serial.available() > 0) {
    int c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        handleLine(lineBuf);
        lineLen = 0;
      }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = (char)c;
    } else {
      lineLen = 0;
    }
  }

  // 2. Vòng lặp PID 100 Hz (10ms)
  if (now - lastPidUs >= PID_PERIOD_US) {
    float dt = (float)(now - lastPidUs) * 1e-6f;
    lastPidUs = now;
    if (dt > 0.05f) dt = 0.01f;

    for (int i = 0; i < 4; i++) {
      pidStepWheel(i, dt);
    }
  }

  // 3. In dữ liệu Serial Plotter: 20 Hz (50ms)
  if (now - lastPrintUs >= PRINT_PERIOD_US) {
    lastPrintUs = now;
    int plotIdx = (activeWheel >= 0 && activeWheel < 4) ? activeWheel : 0;

    Serial.printf("Setpoint:%.2f Actual:%.2f Duty:%.2f\n",
                  wheelSetpoint[plotIdx], actualVel[plotIdx], dutyOut[plotIdx]);
  }
}