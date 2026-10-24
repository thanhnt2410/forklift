// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "firmware_core.h"

namespace forklift_board {
// User-reported allocation for ESP32-WROOM DevKit 38 pin. Not electrically verified.
// GPIO12 (FR A) can select the wrong flash voltage when HIGH at reset.
// GPIO5 (RL A) is also a strap. Review external boot levels before enabling.
constexpr bool wiring_reviewed = false;
constexpr bool signal_levels_reviewed = false;
constexpr bool strapping_levels_reviewed = false;
constexpr bool enable_lift = false;
// Internal joint order stays FL, FR, RL, RR. Each pair is RPWM/LPWM or A/B.
constexpr int pwm[4][2] = {{23,22}, {18,19}, {25,26}, {21,27}};
constexpr int encoder[4][2] = {{34,35}, {12,14}, {5,17}, {16,4}};
// User reports 11 PPR. Interpreted as cycles/channel/motor revolution.
// PCNT counts both edges of A and B (x4). Ratio 21.3 is nominal; verify by
// counting output-shaft revolutions before using this for REAL odometry.
constexpr double encoder_motor_ppr = 11.;
constexpr double nominal_gear_ratio = 21.3;
constexpr unsigned encoder_decode_multiplier = 4;
constexpr double nominal_encoder_cpr = encoder_motor_ppr * encoder_decode_multiplier * nominal_gear_ratio;
constexpr int encoder_sign[4] = {1,1,1,1};
constexpr int motor_sign[4] = {1,1,1,1};
// User selected R_EN/L_EN tied HIGH externally on all four BTS7960 modules.
// No ESP32 EN GPIO in this mode; PWM=0 is not electrical driver isolation.
constexpr bool base_enable_hardwired_high = true;
constexpr int base_enable_pin = -1;
// User explicitly selected manual power cutoff, including on communication loss.
constexpr bool stop_base_on_fault = false;
// Optional for base-only manual-cutoff mode; required before enabling lift.
constexpr int estop_pin = -1;
constexpr int lower_pin = -1, upper_pin = -1;
// User reports PUL+ = 13, DIR+ = 33, PUL- and DIR- = GND.
constexpr int step_pin = 13, direction_pin = 33;
// ENA behavior differs across TB6600 modules. Leave unassigned until reviewed;
// an initialized lift holds current, including during a firmware stop.
constexpr int lift_enable_pin = -1;
constexpr bool lift_enable_level = false;
constexpr bool step_up_level = true;
// Candidate NC circuits: healthy contact to GND; open/tripped reads HIGH.
// E-stop must also have a physical stop circuit, independent of the MCU.
constexpr bool estop_active_level = true, limit_active_level = true;
constexpr uint32_t baud = 230400;
constexpr uint32_t control_period_us = 10000;
constexpr uint32_t telemetry_period_us = 20000;

inline forklift_mcu::Config control_config() {
  forklift_mcu::Config c;
  c.base_ready = wiring_reviewed && signal_levels_reviewed && strapping_levels_reviewed;
  c.lift_ready = enable_lift;
  c.stop_base_on_fault = stop_base_on_fault;
  c.encoder_cpr.fill(nominal_encoder_cpr);  // 937.2 counts / output revolution.
  // PID and lift calibration still pending; outputs remain locked.
  c.gains = {{{0.,0.,0.},{0.,0.,0.},{0.,0.,0.},{0.,0.,0.}}};
  c.max_wheel_rad_s = 2.;  // Initial bench envelope, not GA25 rated speed.
  c.max_duty = .25;
  c.steps_per_m = 0.; c.travel_m = 0.;
  c.lift_max_steps_s = 0.; c.lift_accel_steps_s2 = 0.;
  c.home_steps_s = 0.; c.backoff_m = 0.; c.home_timeout_s = 0.;
  return c;
}
}  // namespace forklift_board
