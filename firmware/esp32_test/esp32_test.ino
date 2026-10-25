// SPDX-License-Identifier: Apache-2.0
#include <Arduino.h>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include "motor_config.h"

#if !CONFIG_IDF_TARGET_ESP32
#error This sketch uses the pin map of a classic ESP32-WROOM.
#endif

// Firmware test doc lap: PWM truc tiep, khong PID, khong arm, khong timeout Serial.
// FL,25          -> chi banh truoc trai, PWM +25%; cac banh khac dung.
// RL,-25         -> chi banh sau trai, PWM -25%; cac banh khac dung.
// ALL,25         -> ca bon banh PWM +25%.
// C,25,0,-25,0   -> dat rieng PWM FL,FR,RL,RR.
// D              -> dung tat ca.
// Z              -> dua so dem encoder ve 0, KHONG dung motor.
// ?              -> hien huong dan.

int motorPWM[MOTOR_COUNT] = {0, 0, 0, 0};
bool pwmAttached[MOTOR_COUNT][2] = {};
bool motorsReady = false;

volatile int64_t encoderCount[MOTOR_COUNT] = {};
volatile uint8_t encoderState[MOTOR_COUNT] = {};
portMUX_TYPE encoderLock = portMUX_INITIALIZER_UNLOCKED;

// Bang chuyen trang thai A/B, cung quy uoc dau x4 voi firmware chinh.
// Vi du: 00 -> 01 -> 11 -> 10 -> 00 dem +4.
DRAM_ATTR const int8_t ENCODER_TRANSITION[16] = {
   0,  1, -1,  0,
  -1,  0,  0,  1,
   1,  0,  0, -1,
   0, -1,  1,  0
};

int64_t previousCount[MOTOR_COUNT] = {};
unsigned long lastStatusMs = 0;
char serialLine[96];
size_t serialLength = 0;
bool serialOverflow = false;

void ARDUINO_ISR_ATTR updateEncoder(int motor) {
  portENTER_CRITICAL_ISR(&encoderLock);
  const uint8_t state = (digitalRead(ENCODER_A_PIN[motor]) << 1) |
                        digitalRead(ENCODER_B_PIN[motor]);
  const uint8_t transition = (encoderState[motor] << 2) | state;
  encoderCount[motor] += ENCODER_TRANSITION[transition] * ENCODER_SIGN[motor];
  encoderState[motor] = state;
  portEXIT_CRITICAL_ISR(&encoderLock);
}

void ARDUINO_ISR_ATTR encoderFL() { updateEncoder(0); }
void ARDUINO_ISR_ATTR encoderFR() { updateEncoder(1); }
void ARDUINO_ISR_ATTR encoderRL() { updateEncoder(2); }
void ARDUINO_ISR_ATTR encoderRR() { updateEncoder(3); }

void setupEncoders() {
  void (*interruptHandler[MOTOR_COUNT])() = {encoderFL, encoderFR, encoderRL, encoderRR};

  for (int motor = 0; motor < MOTOR_COUNT; motor++) {
    const int pinA = ENCODER_A_PIN[motor];
    const int pinB = ENCODER_B_PIN[motor];
    // GPIO34/35 khong co dien tro keo noi; dung INPUT.
    pinMode(pinA, pinA >= 34 ? INPUT : INPUT_PULLUP);
    pinMode(pinB, pinB >= 34 ? INPUT : INPUT_PULLUP);
    encoderState[motor] = (digitalRead(pinA) << 1) | digitalRead(pinB);
  }

  for (int motor = 0; motor < MOTOR_COUNT; motor++) {
    attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN[motor]), interruptHandler[motor], CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN[motor]), interruptHandler[motor], CHANGE);
  }
}

void stopAllMotors() {
  for (int motor = 0; motor < MOTOR_COUNT; motor++) {
    if (pwmAttached[motor][0]) ledcWrite(RPWM_PIN[motor], 0);
    if (pwmAttached[motor][1]) ledcWrite(LPWM_PIN[motor], 0);
    motorPWM[motor] = 0;
  }
}

bool setupMotors() {
  // Tat het chan PWM truoc khi gan LEDC.
  for (int motor = 0; motor < MOTOR_COUNT; motor++) {
    pinMode(RPWM_PIN[motor], OUTPUT);
    pinMode(LPWM_PIN[motor], OUTPUT);
    digitalWrite(RPWM_PIN[motor], LOW);
    digitalWrite(LPWM_PIN[motor], LOW);
  }

  for (int motor = 0; motor < MOTOR_COUNT; motor++) {
    pwmAttached[motor][0] = ledcAttachChannel(
      RPWM_PIN[motor], PWM_FREQUENCY, PWM_RESOLUTION, motor * 2);
    pwmAttached[motor][1] = ledcAttachChannel(
      LPWM_PIN[motor], PWM_FREQUENCY, PWM_RESOLUTION, motor * 2 + 1);

    if (!pwmAttached[motor][0] || !pwmAttached[motor][1]) {
      stopAllMotors();
      return false;
    }
  }

  stopAllMotors();
  return true;
}

void setMotorPWM(int motor, int percent) {
  const int signedPercent = percent * MOTOR_SIGN[motor];
  const int duty = abs(signedPercent) * PWM_MAX / 100;

  // Tat ca hai ngo vao truoc khi cap PWM theo chieu moi.
  ledcWrite(RPWM_PIN[motor], 0);
  ledcWrite(LPWM_PIN[motor], 0);
  if (signedPercent > 0) ledcWrite(RPWM_PIN[motor], duty);
  if (signedPercent < 0) ledcWrite(LPWM_PIN[motor], duty);
  motorPWM[motor] = percent;
}

bool parsePWM(const char * text, int & percent) {
  if (*text == '\0') return false;
  char * end;
  errno = 0;
  const long value = strtol(text, &end, 10);
  if (errno != 0 || *end != '\0' || value < -100 || value > 100) return false;
  percent = int(value);
  return true;
}

void printHelp() {
  Serial.println("ESP32 motor test | 115200 baud | Newline / CRLF");
  Serial.println("FL,25 / FR,25 / RL,25 / RR,25 : one motor, others stop");
  Serial.println("ALL,25 : all motors; C,25,0,-25,0 : PWM FL,FR,RL,RR");
  Serial.println("PWM: integer -100..100 percent; negative = reverse; 0 = stop");
  Serial.println("D = stop all; Z = zero encoders; ? = help");
  Serial.println("One command keeps running until a new command or D.");
  Serial.println("Encoder RPM uses nominal 937.2 counts/wheel revolution.");
}

void zeroEncoders() {
  portENTER_CRITICAL(&encoderLock);
  for (int motor = 0; motor < MOTOR_COUNT; motor++) encoderCount[motor] = 0;
  portEXIT_CRITICAL(&encoderLock);

  for (int motor = 0; motor < MOTOR_COUNT; motor++) previousCount[motor] = 0;
  lastStatusMs = millis();
  Serial.println("OK encoders zeroed (PWM unchanged)");
}

void processCommand(char * line) {
  if (strcmp(line, "D") == 0) {
    stopAllMotors();
    Serial.println("OK stopped");
    return;
  }
  if (strcmp(line, "Z") == 0) {
    zeroEncoders();
    return;
  }
  if (strcmp(line, "?") == 0) {
    printHelp();
    return;
  }

  // Tach CSV, toi da 5 truong, khong bo qua truong rong.
  char * fields[5] = {line};
  int fieldCount = 1;
  for (char * p = line; *p; p++) {
    if (*p != ',') continue;
    if (fieldCount == 5) {
      Serial.println("ERR too many fields; use ? for help");
      return;
    }
    *p = '\0';
    fields[fieldCount++] = p + 1;
  }

  // Kiem tra toan bo lenh truoc khi thay doi PWM.
  int requestedPWM[MOTOR_COUNT] = {0, 0, 0, 0};
  bool valid = false;
  if (fieldCount == 5 && strcmp(fields[0], "C") == 0) {
    valid = true;
    for (int motor = 0; motor < MOTOR_COUNT; motor++) {
      if (!parsePWM(fields[motor + 1], requestedPWM[motor])) valid = false;
    }
  } else if (fieldCount == 2) {
    int percent;
    if (parsePWM(fields[1], percent)) {
      for (int motor = 0; motor < MOTOR_COUNT; motor++) {
        if (strcmp(fields[0], MOTOR_NAME[motor]) == 0 || strcmp(fields[0], "ALL") == 0) {
          requestedPWM[motor] = percent;
          valid = true;
        }
      }
    }
  }

  if (!valid) {
    Serial.println("ERR use FL,25 or C,25,0,0,0; PWM must be integer -100..100");
    return;
  }
  if (!motorsReady) {
    Serial.println("ERR PWM initialization failed; reset board after checking configuration");
    return;
  }

  for (int motor = 0; motor < MOTOR_COUNT; motor++) setMotorPWM(motor, requestedPWM[motor]);
  Serial.printf("OK PWM FL=%d FR=%d RL=%d RR=%d %%\n",
                motorPWM[0], motorPWM[1], motorPWM[2], motorPWM[3]);
}

void readSerial() {
  // Khong dung readStringUntil: tranh cho timeout khi chua nhan du dong.
  for (int bytesRead = 0; bytesRead < 128 && Serial.available(); bytesRead++) {
    const char byte = Serial.read();
    if (byte == '\n') {
      if (serialOverflow) {
        Serial.println("ERR invalid or too long line; command ignored");
      } else {
        // Chi bo CR o cuoi dong de chap nhan ca CRLF.
        if (serialLength > 0 && serialLine[serialLength - 1] == '\r') serialLength--;
        serialLine[serialLength] = '\0';
        if (serialLength > 0) processCommand(serialLine);
      }
      serialLength = 0;
      serialOverflow = false;
    } else if (!serialOverflow) {
      if (byte == '\0' || serialLength >= sizeof(serialLine) - 1) {
        serialOverflow = true;
      } else {
        serialLine[serialLength++] = byte;
      }
    }
  }
}

void printStatus(unsigned long now) {
  const unsigned long elapsed = now - lastStatusMs;
  if (elapsed < STATUS_INTERVAL_MS) return;

  int64_t counts[MOTOR_COUNT];
  portENTER_CRITICAL(&encoderLock);
  for (int motor = 0; motor < MOTOR_COUNT; motor++) counts[motor] = encoderCount[motor];
  portEXIT_CRITICAL(&encoderLock);

  Serial.printf("T=%lu ms", now);
  for (int motor = 0; motor < MOTOR_COUNT; motor++) {
    const int64_t delta = counts[motor] - previousCount[motor];
    const float rpm = float(delta) * 60000.0f / (COUNTS_PER_WHEEL_REV * elapsed);
    previousCount[motor] = counts[motor];
    Serial.printf(" | %s pwm=%d%% enc=%lld rpm=%.1f",
                  MOTOR_NAME[motor], motorPWM[motor], (long long)counts[motor], rpm);
  }
  Serial.println();
  lastStatusMs = now;
}

void setup() {
  Serial.begin(SERIAL_BAUD);

  // Khong test nang: giu STEP/DIR TB6600 o LOW.
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  pinMode(33, OUTPUT);
  digitalWrite(33, LOW);

  motorsReady = setupMotors();
  setupEncoders();
  lastStatusMs = millis();
  printHelp();
  Serial.println(motorsReady ? "READY motors stopped" : "ERR PWM initialization failed");
}

void loop() {
  readSerial();
  printStatus(millis());
  delay(1);
}
