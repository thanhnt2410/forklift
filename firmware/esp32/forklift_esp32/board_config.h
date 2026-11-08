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
// FR encoder A: GPIO32 (31/10/2026, người dùng chuyển dây từ GPIO2). GPIO2 là chân
// strapping: phải rút dây mỗi lần nạp, quên cắm lại thì FR đo 0 rad/s và PI đẩy FR lên
// trần duty (gặp lại 31/10). GPIO32 không strapping, có pull-up trong, nhận ngắt.
// DÂY THẬT PHẢI Ở GPIO32: đọc nhầm chân thì FR mất kênh A, tốc độ đo = 0.
constexpr int encoder[4][2] = {{34, 35}, {32, 14}, {5, 17}, {16, 4}};

// Thông số Encoder và Hộp số bánh xe
constexpr double encoder_motor_ppr = 11.;
constexpr double nominal_gear_ratio = 21.3;
constexpr unsigned encoder_decode_multiplier = 4;
constexpr double nominal_encoder_cpr = encoder_motor_ppr * encoder_decode_multiplier * nominal_gear_ratio;

// Chiều quay Encoder và Motor
// FR 1 -> -1 (30/10/2026, sau khi sửa dây encoder FR): thử trên không, FR quay tiến
// mà đếm âm -> PI hồi tiếp dương, kẹt ~14,6 rad/s ngược dấu lệnh. Khớp esp32_test.
constexpr int encoder_sign[4] = {1, -1, 1, -1};
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
// Chân COM nối GND, chân NC nối GPIO 15 (Trạng thái bình thường LOW, chạm ngắt HIGH).
// Lịch sử chân này: 15 -> 36 (01/11/2026, vì GPIO15 luôn đọc LOW kể cả khi ấn) -> 15 lại
// (06/11/2026, người dùng yêu cầu). Lý do kỹ thuật ủng hộ: GPIO15 CÓ pull-up trong (~45 kΩ)
// nên KHÔNG cần điện trở 10 kΩ ngoài như GPIO36 (GPIO34-39 không có pull-up trong), và
// GPIO15 là strapping "hiền": mức lúc boot chỉ bật/tắt log boot, không làm hỏng chế độ boot
// như GPIO0 (phải HIGH) hay GPIO12 (phải LOW).
// NẾU LẠI LUÔN ĐỌC LOW: pull-up trong ~45 kΩ quá yếu so với trở kéo xuống/LED mà nhiều board
// gắn sẵn ở GPIO15 — thêm điện trở 4,7 kΩ từ GPIO15 lên 3,3 V (không lên 5 V) là hết.
// Dù dùng chân nào cũng nên có tụ 100 nF xuống GND để lọc nhiễu motor.
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

// Thông số PI vận tốc bánh xe — tuning KHÔNG TẢI 29/10/2026 (lệnh G, FL/RL):
// Kp 0,032->0,1, Ki 0,16->0,5 khởi động 0,05–0,25 s ở 0,6 rad/s, không vọt lố
// tới 6 rad/s và khi chạy lùi. Kp 0,032 cũ có lần không khởi động ở 0,6 rad/s.
// Kiểm lại TRÊN SÀN 29/10/2026 (xe 3–4 kg, ff 0,19+0,028·ω, cả 4 bánh, bậc
// tiến/lùi 3 s): 0,8 / 1,6 / 4,0 rad/s sai số TB ≤ 6 % / 2 % / 1 %, vọt đỉnh ~10 %,
// không warning stall; ở 4 rad/s bánh bám đúng ramp wheel_accel 2 rad/s².
// So sánh ở 0,8 rad/s: 0,15/0,6 không tốt hơn (vọt +8–11 % khi tiến); 0,06/0,4
// dính-trượt (sd 0,5–0,7 rad/s, đỉnh 2,4). Giữ 0,1/0,5. Chưa thử khi có pallet.
// Vận tốc đo từ ticks 20 Hz với CPR danh nghĩa 937,2 (chưa hiệu chuẩn).
constexpr forklift_mcu::Gains wheel_pi[4] = {
  {0.1, 0.5, 0.0},  // FL
  {0.1, 0.5, 0.0},  // FR
  {0.1, 0.5, 0.0},  // RL
  {0.1, 0.5, 0.0},  // RR
};

// Feedforward bù ma sát (FL, FR, RL, RR) — đặt 0 thì quay về PI thuần như cũ.
// Không có phần này, ở tốc độ thấp (< ~0,3 m/s) khâu I phải tích lũy vài giây
// mới đủ duty để bánh bứt khỏi ma sát tĩnh, rồi bánh giật quá đà.
// wheel_static_duty: duty nhỏ nhất để bánh VẪN QUAY ĐỀU ở tốc độ thấp (đo bằng
//   lệnh P, giảm dần duty tới khi bánh dừng) — không lấy duty bứt khởi động.
// wheel_velocity_duty: duty thêm cho mỗi rad/s = (duty2-duty1)/(v2-v1) từ dòng V.
// Ràng buộc: static < max_duty và velocity*max_wheel_rad_s <= max_duty.
// Đo 29/10/2026 TRÊN SÀN, xe 3–4 kg, không pallet (lệnh P cả 4 bánh, xung
// tiến/lùi): bứt khởi động 0,25–0,28; giữ quay ~0,22 tiến / ~0,19 lùi; khớp
// 1–6 rad/s duty ≈ 0,19 + 0,028·ω, 4 bánh lệch nhau ≤ 0,3 rad/s nên dùng chung.
// (Không tải khi kê bánh chỉ cần ~0,12 + 0,02·ω.) Chưa đo khi có pallet.
constexpr double wheel_static_duty[4] = {0.19, 0.19, 0.19, 0.19};
constexpr double wheel_velocity_duty[4] = {0.028, 0.028, 0.028, 0.028};

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
  for (unsigned i = 0; i < 4; ++i) {
    c.gains[i] = wheel_pi[i];
    c.static_duty[i] = wheel_static_duty[i];
    c.velocity_duty[i] = wheel_velocity_duty[i];
  }
  // Lệnh P (duty thô, bỏ qua PI) chỉ dùng khi kê bánh đo trên bàn.
  c.bench_duty_command = base_bench_profile;
  c.max_wheel_rad_s = 10.;
  // Ramp tăng/giảm tốc bánh (rad/s²). Mặc định core 2,0 = chỉ ~0,38 rad/s² thân xe
  // khi quay (r 0,05 / k 0,26) và 0,1 m/s² khi đi ngang: 30/10/2026 xung quay
  // 0,25 rad/s cần ~0,65 s mới dừng, lố ~9° quanh đích; đi ngang 0,09 m/s trôi 4-7 cm.
  // 6,0 -> dừng nhanh gấp 3. PI đã kiểm trên sàn tới 4 rad/s (29/10), feedforward giữ
  // bánh bám ramp. Chưa đo lại sau khi đổi.
  c.wheel_accel = 6.;
  // Trần PWM 0,50 -> 0,70 (30/10/2026, người dùng yêu cầu). Bánh cao su skid-steer khi
  // quay tại chỗ phải kéo lê bánh ngang nên cần mô-men lớn hơn đi thẳng nhiều; với trần
  // 0,50 PI dễ kịch trần khi quay. Trần này chặn cả PI lẫn lệnh P thử bàn. CHƯA đo điện
  // áp nguồn so với định mức motor GA25-370 (robot_measurements.yaml chưa có) và dòng
  // khi kẹt: theo dõi nhiệt motor/driver BTS7960 khi thử.
  // 0,70 -> 1,0 (03/11/2026, người dùng xác nhận nguồn motor 12 V = định mức GA25-370 12 V):
  // quay tại chỗ trên sàn ở 0,5-0,6 rad/s vẫn kẹt cả 4 bánh vì PI đã kịch trần 0,70; tăng
  // tốc độ lệnh không thêm mô-men, chỉ trần duty mới thêm. Kẹt lâu ở 100% thì dòng kẹt lớn
  // (stall chỉ là warning, không cắt): sờ nhiệt motor/driver sau vài lần quay. Chưa đo dòng.
  c.max_duty = 1.;
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