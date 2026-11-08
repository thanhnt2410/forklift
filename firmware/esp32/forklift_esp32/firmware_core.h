// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

namespace forklift_mcu {
constexpr double kTau = 6.2831853071795864769;
constexpr size_t kFrameCapacity = 256;
enum Fault : uint32_t {
  // Bit 4 is reserved for the old serial watchdog; it is no longer raised.
  Configuration = 1, Emergency = 2, Encoder = 8,
  Limit = 16, HomeTimeout = 32, Deadline = 64, Peripheral = 128
};
enum class Mode : uint8_t { Locked, Disarmed, Active, Homing, Faulted };
// Duty: lenh ban thu P,dFL,dFR,dRL,dRR dat duty tho de do ma sat; wheel[] la duty.
// Tune: lenh ban thu G,kp,ki,kd doi he so PI ca 4 banh luc chay; wheel[0..2] la kp,ki,kd.
enum class Operation { Control, Home, Raise, Reset, Stop, Duty, Tune };
struct Command {
  Operation operation{Operation::Stop};
  uint64_t boot{}, sequence{};
  std::array<double, 4> wheel{};
  double lift_m{};
  bool enabled{};
};
uint32_t crc32(const char * data, size_t size);
bool parse_command(const char * data, size_t size, Command & command);
size_t encode_frame(const char * payload, char * output, size_t capacity);
class Receiver {
 public:
  bool push(char byte, Command & command);
  uint64_t rejected() const { return rejected_; }
 private:
  char buffer_[kFrameCapacity]{};
  size_t used_{};
  bool discard_{};
  uint64_t rejected_{};
};
// Plain newline-delimited bench protocol; no CRC, boot ID or sequence on wire.
bool parse_simple_command(const char * data, size_t size, double steps_per_m, Command & command);
class SimpleReceiver {
 public:
  bool push(char byte, double steps_per_m, Command & command);
 private:
  char buffer_[kFrameCapacity]{};
  size_t used_{};
  bool discard_{};
};

struct Gains { double kp{}, ki{}, kd{}; };
struct Config {
  bool base_ready{false}, lift_ready{false};
  bool stop_base_on_fault{true};
  bool encoder_spikes_warn_only{false};
  bool lower_only_lift{false};
  int64_t home_seek_max_steps{}, home_release_max_steps{};
  int64_t lift_raise_steps{2000};
  double lift_lower_timeout_s{3.};
  std::array<double, 4> encoder_cpr{};  // Actual x4 counts / output shaft revolution.
  std::array<Gains, 4> gains{};
  // Feedforward bu ma sat: duty = sign(w)*(static_duty + velocity_duty*|w|) + PI.
  // Mac dinh 0 = hanh vi PI cu. Gia tri phai do tren ban (lenh P).
  std::array<double, 4> static_duty{}, velocity_duty{};
  // Cho phep lenh P (duty tho, bo qua PI). Chi bat trong profile thu ban.
  bool bench_duty_command{false};
  double max_wheel_rad_s{2.0}, wheel_accel{2.0}, max_duty{0.25};
  double steps_per_m{}, travel_m{}, lift_max_steps_s{}, lift_accel_steps_s2{};
  double home_steps_s{}, backoff_m{}, home_timeout_s{};
  bool valid() const;
};
struct Sensors {
  std::array<int64_t, 4> ticks{};
  int64_t lift_steps{};
  bool estop{}, emergency_event{}, low{}, high{}, io_ok{true};
};
struct Outputs {
  std::array<double, 4> duty{};
  double lift_steps_s{};
  bool base_enable{}, lift_hold{}, zero_lift{}, clear_emergency{};
  bool lift_position_limit{};
  int64_t lift_goal_steps{};
};
class Pid {
 public:
  double update(double target, double actual, double dt, const Gains & gains, double bound,
                double feedforward = 0.);
  void reset() { integral_ = previous_ = 0.; primed_ = false; }
 private:
  double integral_{}, previous_{};
  bool primed_{};
};
class Controller {
 public:
  Controller(const Config & config, uint64_t boot);
  bool receive(const Command & command, uint64_t now_us, const Sensors & sensors);
  void tick(uint64_t now_us, const Sensors & sensors);
  const Outputs & outputs() const { return output_; }
  void acknowledge_outputs() { output_.zero_lift = output_.clear_emergency = false; }
  uint32_t faults() const { return faults_; }
  // Bits 0..3: FL/FR/RL/RR tracking trouble or discarded encoder spikes.
  uint32_t warnings() const { return warnings_; }
  bool homed() const { return homed_; }
  Mode mode() const { return mode_; }
  uint64_t ack() const { return ack_; }
  const std::array<double, 4> & velocity() const { return velocity_; }
  size_t telemetry(uint64_t now_us, const Sensors & sensors, char * output, size_t capacity);
  size_t simple_telemetry(uint64_t now_us, const Sensors & sensors, char * output, size_t capacity);
  // Dong "V,ms,vFL..vRR,dFL..dRR" chi khi dang chay lenh P; 0 neu khong.
  size_t bench_telemetry(uint64_t now_us, char * output, size_t capacity) const;
  bool raw_duty() const { return raw_duty_; }
  const Gains & gains(size_t wheel) const { return config_.gains[wheel]; }
 private:
  void stop();
  void stop_for_fault();
  void trip(uint32_t fault);
  bool stopped() const;
  bool wheels_moved_during_lift(const Sensors & sensors) const;
  void lift_tick(double dt, uint64_t now_us, const Sensors & sensors);
  void manual_lift_tick(double dt, uint64_t now_us, const Sensors & sensors);
  Config config_;
  uint64_t boot_, ack_{}, state_seq_{}, last_tick_{}, home_started_{};
  bool primed_{}, homed_{};
  Mode mode_{Mode::Locked};
  uint32_t faults_{};
  uint32_t warnings_{};
  std::array<Pid, 4> pid_;
  std::array<double, 4> target_{}, ramp_{}, velocity_{}, stationary_s_{}, stall_s_{};
  std::array<double, 4> encoder_warning_s_{};
  // Khóa "bánh không lăn khi nâng": số đếm encoder lúc bắt đầu đoạn nâng và thời
  // gian tốc độ lọc vượt ngưỡng liên tục (xem wheels_moved_during_lift).
  std::array<int64_t, 4> lift_start_ticks_{};
  std::array<double, 4> moving_s_{};
  bool raw_duty_{};
  std::array<double, 4> raw_duty_value_{};
  std::array<int, 4> direction_{};
  std::array<int64_t, 4> previous_ticks_{};
  double lift_target_{}, lift_rate_{};
  bool lift_lowering_{};
  int64_t lift_raise_goal_{};
  enum class HomePhase { Seek, Backoff, Approach } home_phase_{HomePhase::Seek};
  int64_t release_step_{};
  int64_t home_phase_start_step_{};
  bool released_{};
  Outputs output_;
};
}  // namespace forklift_mcu
