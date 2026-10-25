// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "firmware_core.h"

namespace forklift_board {
// User-reported allocation for ESP32-WROOM DevKit 38 pin. Not electrically verified.
// FR encoder A moved from GPIO12 to GPIO2 to avoid the flash-voltage strap.
// GPIO2 (FR A) and GPIO5 (RL A) are straps; review external boot levels.
// GPIO2 must be LOW or floating when entering UART download mode.
// User-authorized bench profile: uncalibrated PI and single-lower-switch lift.
// This permits GPIO initialization without claiming electrical checks were done.
// MCU boots with PWM=0. A single enabled C command starts the wheels;
// no disabled handshake or periodic serial heartbeat is required.
constexpr bool base_bench_profile = true;
constexpr bool wiring_reviewed = false;
constexpr bool signal_levels_reviewed = false;
constexpr bool strapping_levels_reviewed = false;
constexpr bool single_lower_bench_profile = true;
constexpr bool enable_lift = true;
// Internal joint order stays FL, FR, RL, RR. Each pair is RPWM/LPWM or A/B.
// Synced with esp32_test: FL/RL PWM pairs were swapped in the original map.
constexpr int pwm[4][2] = {{25,26}, {18,19}, {23,22}, {21,27}};
constexpr int encoder[4][2] = {{34,35}, {2,14}, {5,17}, {16,4}};
// User reports 11 PPR. Interpreted as cycles/channel/motor revolution.
// PCNT counts both edges of A and B (x4). Ratio 21.3 is nominal; verify by
// counting output-shaft revolutions before using this for REAL odometry.
constexpr double encoder_motor_ppr = 11.;
constexpr double nominal_gear_ratio = 21.3;
constexpr unsigned encoder_decode_multiplier = 4;
constexpr double nominal_encoder_cpr = encoder_motor_ppr * encoder_decode_multiplier * nominal_gear_ratio;
// Same polarity corrections as the standalone motor test.
constexpr int encoder_sign[4] = {1,1,1,-1};
constexpr int motor_sign[4] = {1,-1,1,-1};
// User selected R_EN/L_EN tied HIGH externally on all four BTS7960 modules.
// No ESP32 EN GPIO in this mode; PWM=0 is not electrical driver isolation.
constexpr bool base_enable_hardwired_high = true;
constexpr int base_enable_pin = -1;
// User explicitly selected manual power cutoff, including on communication loss.
constexpr bool stop_base_on_fault = false;
// Optional in the user-selected manual-cutoff/single-lower bench profile.
constexpr int estop_pin = -1;
// COM -> GND, NC -> GPIO32; healthy LOW, pressed/open HIGH (INPUT_PULLUP).
constexpr int lower_pin = 15, upper_pin = -1;
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
constexpr uint32_t baud = 115200;
constexpr bool simple_serial = true;
constexpr uint32_t control_period_us = 10000;
constexpr uint32_t telemetry_period_us = 50000;  // 20 Hz, readable Serial Monitor.

// Starting guesses, not gains measured on GA25-370. Duty is normalized [-1,1],
// velocity rad/s, dt seconds; tune each wheel independently after checking sign.
constexpr forklift_mcu::Gains wheel_pi[4] = {
  {.08,.12,0.05},  // FL
  {.08,.12,0.05},  // FR
  {.08,.12,0.05},  // RL
  {.08,.12,0.05},  // RR
};
// Provisional distance conversion ONLY, not calibrated physical metres.
// 1.8-degree NEMA17, TB6600 1/8 step, direct screw with 8 mm lead/revolution.
// Match actual motor, DIP switches, lead, travel and assigned limits before use.
constexpr double lift_full_steps_per_rev = 200.;
constexpr unsigned lift_microsteps = 8;
constexpr double lift_screw_lead_m = .008;
constexpr double provisional_lift_steps_per_m =
  lift_full_steps_per_rev * lift_microsteps / lift_screw_lead_m;
// Moi lan U/ROS R nang them 2000 xung tu vi tri hien tai.
constexpr int64_t lift_raise_steps = 2000;
constexpr double lift_lower_timeout_s = 3.;
// Thang toa do/profile position cu; khong gioi han tong hanh trinh nang thu cong.
constexpr int64_t lift_max_steps = 2000;
// Chi dung neu tat single_lower_bench_profile de quay lai homing kieu cu.
constexpr int64_t home_seek_max_steps = 6000;
constexpr int64_t home_release_max_steps = 200;
constexpr int64_t home_backoff_steps = 100;

inline constexpr bool base_initialization_allowed() {
  return (base_bench_profile && (!enable_lift || single_lower_bench_profile)) ||
    (wiring_reviewed && signal_levels_reviewed && strapping_levels_reviewed);
}

inline forklift_mcu::Config control_config() {
  forklift_mcu::Config c;
  c.base_ready = base_initialization_allowed();
  c.lift_ready = enable_lift;
  c.lower_only_lift = single_lower_bench_profile;
  c.lift_raise_steps = lift_raise_steps;
  c.lift_lower_timeout_s = lift_lower_timeout_s;
  c.home_seek_max_steps = home_seek_max_steps;
  c.home_release_max_steps = home_release_max_steps;
  c.stop_base_on_fault = stop_base_on_fault;
  c.encoder_cpr.fill(nominal_encoder_cpr);  // 937.2 counts / output revolution.
  for (unsigned i=0;i<4;++i) c.gains[i] = wheel_pi[i];
  c.max_wheel_rad_s = 10.;  // MCU ceiling; ROS limits translation to 6 rad/s (0.30 m/s).
  c.max_duty = .50;
  c.steps_per_m = provisional_lift_steps_per_m;
  c.travel_m = lift_max_steps / provisional_lift_steps_per_m;
  c.lift_max_steps_s = 1000.; c.lift_accel_steps_s2 = 4000.;
  c.home_steps_s = 1000.;  // Toc do ha: 1000 xung/s, toi da 3 giay.
  c.backoff_m = home_backoff_steps / provisional_lift_steps_per_m;
  c.home_timeout_s = 30.;
  return c;
}
}  // namespace forklift_board
