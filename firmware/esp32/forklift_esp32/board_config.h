// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "firmware_core.h"

namespace forklift_board {

// =============================================================================
// CẤU HÌNH HỆ THỐNG CƠ BẢN
// =============================================================================
constexpr bool base_bench_profile = true;
constexpr bool wiring_reviewed = false;
constexpr bool signal_levels_reviewed = false;
constexpr bool strapping_levels_reviewed = false;
constexpr bool single_lower_bench_profile = true;
constexpr bool enable_lift = true;

// =============================================================================
// CẤU HÌNH 4 BÁNH XE (FL, FR, RL, RR) VÀ DRIVER BTS7960
// =============================================================================
constexpr int pwm[4][2] = {{25, 26}, {18, 19}, {23, 22}, {21, 27}};
constexpr int encoder[4][2] = {{34, 35}, {2, 14}, {5, 17}, {16, 4}};

// Thông số Encoder và Hộp số bánh xe
constexpr double encoder_motor_ppr = 11.;
constexpr double nominal_gear_ratio = 21.3;
constexpr unsigned encoder_decode_multiplier = 4;
constexpr double nominal_encoder_cpr = encoder_motor_ppr * encoder_decode_multiplier * nominal_gear_ratio;

// Chiều quay Encoder và Motor
constexpr int encoder_sign[4] = {1, 1, 1, -1};
constexpr int motor_sign[4] = {1, -1, 1, -1};

constexpr bool base_enable_hardwired_high = true;
constexpr int base_enable_pin = -1;
constexpr bool stop_base_on_fault = false;
// Encoder spikes are discarded and reported in the wheel warning mask.
constexpr bool encoder_spikes_warn_only = true;
// Optional in the user-selected manual-cutoff/single-lower bench profile.
constexpr int estop_pin = -1;

// =============================================================================
// CẤU HÌNH CÔNG TẮC HÀNH TRÌNH VÀ MOTOR NÂNG HẠ (TB6600 + VÍT ME)
// =============================================================================
// Chân COM nối GND, chân NC nối GPIO 15 (Trạng thái bình thường LOW, chạm ngắt HIGH)
constexpr int lower_pin = 15, upper_pin = -1;

// Chân điều khiển Driver TB6600: PUL+ = 13, DIR+ = 33, PUL- và DIR- nối GND
constexpr int step_pin = 13, direction_pin = 33;
constexpr int lift_enable_pin = -1;
constexpr bool lift_enable_level = false;

// ĐẢO CHIỀU ĐỘNG CƠ: false để R nâng lên, F/H hạ xuống đáy
constexpr bool step_up_level = false;

// Mức kích hoạt công tắc hành trình (INPUT_PULLUP)
constexpr bool estop_active_level = true, limit_active_level = true;

// Giao tiếp Serial UART
constexpr uint32_t baud = 115200;
constexpr bool simple_serial = true;
constexpr uint32_t control_period_us = 10000;
constexpr uint32_t telemetry_period_us = 50000;  // 20 Hz

// Thông số PI vận tốc bánh xe
constexpr forklift_mcu::Gains wheel_pi[4] = {
  {0.032, 0.16, 0.0},  // FL
  {0.032, 0.16, 0.0},  // FR
  {0.032, 0.16, 0.0},  // RL
  {0.032, 0.16, 0.0},  // RR
};

// =============================================================================
// THÔNG SỐ TRỤC VÍT ME VÀ HÀNH TRÌNH NÂNG HẠ
// =============================================================================
// Động cơ bước 1.8 độ (200 bước/vòng), Driver vi bước 1/8, Trục vít me bước 8 (8mm/vòng)
constexpr double lift_full_steps_per_rev = 200.;
constexpr unsigned lift_microsteps = 8;
constexpr double lift_screw_lead_m = .008;
constexpr double provisional_lift_steps_per_m =
  lift_full_steps_per_rev * lift_microsteps / lift_screw_lead_m; // 200.000 xung/mét

// SỐ XUNG MỖI LẦN BẤM NÂNG: 20.000 xung tương đương ~10 cm
constexpr int64_t lift_raise_steps = 20000;

// THỜI GIAN CHỜ HẠ TỐI ĐA: 30 giây (chờ chạm công tắc đáy mới dừng)
constexpr double lift_lower_timeout_s = 30.;

// GIỚI HẠN CHIỀU CAO TỐI ĐA (Thay thế công tắc trên): 20.000 xung (~10 cm)
constexpr int64_t lift_max_steps = 20000;

// Giới hạn tìm công tắc đáy khi Homing: 35.000 xung
constexpr int64_t home_seek_max_steps = 35000;
constexpr int64_t home_release_max_steps = 200;
constexpr int64_t home_backoff_steps = 100;

// =============================================================================
// HÀM KHỞI TẠO CẤU HÌNH (CONTROL CONFIG)
// =============================================================================
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
  c.encoder_spikes_warn_only = encoder_spikes_warn_only;
  c.encoder_cpr.fill(nominal_encoder_cpr);
  for (unsigned i = 0; i < 4; ++i) c.gains[i] = wheel_pi[i];
  c.max_wheel_rad_s = 10.;
  c.max_duty = .50;
  c.steps_per_m = provisional_lift_steps_per_m;
  c.travel_m = lift_max_steps / provisional_lift_steps_per_m;

  // [TỐI ƯU ĐỂ VƯỢT NGƯỠNG TIMEOUT 10 GIÂY]:
  // Tốc độ nâng: Tăng lên 2500 xung/s (chạy 20.000 bước chỉ mất 8.5 giây, hoàn thành trước 10s)
  c.lift_max_steps_s = 2500.; 
  // Gia tốc khởi động: 3000 xung/s² (tăng tốc dứt khoát nhưng vẫn êm, không trượt bước)
  c.lift_accel_steps_s2 = 3000.;

  // Tốc độ hạ: 2000 xung/s (chạy mượt mà cho tới khi chạm công tắc đáy)
  c.home_steps_s = 2000.;  
  c.backoff_m = home_backoff_steps / provisional_lift_steps_per_m;
  c.home_timeout_s = 30.;

  return c;
}

}  // namespace forklift_board