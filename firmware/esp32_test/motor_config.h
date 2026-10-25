// SPDX-License-Identifier: Apache-2.0
#pragma once

// ESP32-WROOM DevKit 38 pin, Arduino ESP32 core 3.x.
// Thu tu: FL = truoc trai, FR = truoc phai, RL = sau trai, RR = sau phai.
const int MOTOR_COUNT = 4;
const char * const MOTOR_NAME[MOTOR_COUNT] = {"FL", "FR", "RL", "RR"};

// BTS7960: R_EN va L_EN duoc keo HIGH ben ngoai, khong dung chan EN ESP32.
// Da doi FL/RL theo thu nghiem: GPIO25/26 = truoc trai, 23/22 = sau trai.
const int RPWM_PIN[MOTOR_COUNT] = {25, 18, 23, 21};
const int LPWM_PIN[MOTOR_COUNT] = {26, 19, 22, 27};
const int ENCODER_A_PIN[MOTOR_COUNT] = {34, 2, 5, 16};
const int ENCODER_B_PIN[MOTOR_COUNT] = {35, 14, 17, 4};

// Lenh PWM duong = quay tien, theo ket qua test tung banh.
const int MOTOR_SIGN[MOTOR_COUNT] = {1, -1, 1, -1};
// RR truoc day lui nhung dem duong: dao ca dau motor va encoder.
// FR chi dao dong 0/1: giu dau tam thoi, kiem tra day A/B roi test lai.
const int ENCODER_SIGN[MOTOR_COUNT] = {1, 1, 1, -1};

const unsigned long SERIAL_BAUD = 115200;
const unsigned long STATUS_INTERVAL_MS = 250;
const int PWM_FREQUENCY = 20000;
const int PWM_RESOLUTION = 10;
const int PWM_MAX = (1 << PWM_RESOLUTION) - 1;

// Gia tri danh nghia: 11 chu ky/kenh/vong motor, hop so 21.3, doc x4.
// Can do lai so dem tren mot vong truc ra de hieu chuan RPM thuc te.
const float ENCODER_PPR = 11.0f;
const float GEAR_RATIO = 21.3f;
const float COUNTS_PER_WHEEL_REV = ENCODER_PPR * GEAR_RATIO * 4.0f;  // 937.2
