// SPDX-License-Identifier: Apache-2.0
#include "firmware_core.h"
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <inttypes.h>
#include <limits>

namespace forklift_mcu {
namespace {
bool unsigned_value(const char * text, uint64_t & value) {
  if (!*text) return false;
  value = 0;
  for (const char * p = text; *p; ++p) {
    if (*p < '0' || *p > '9') return false;
    const auto digit = unsigned(*p - '0');
    if (value > (std::numeric_limits<uint64_t>::max() - digit) / 10) return false;
    value = value * 10 + digit;
  }
  return true;
}
bool real_value(const char * text, double & value) {
  if (!*text || std::strspn(text, "+-0123456789.eE") != std::strlen(text)) return false;
  char * end; errno = 0; value = std::strtod(text, &end);
  return !*end && errno != ERANGE && std::isfinite(value);
}
double approach(double value, double target, double step) {
  return value + std::clamp(target - value, -step, step);
}
int sign(double value) { return value > 0. ? 1 : value < 0. ? -1 : 0; }
bool positive(double value) { return std::isfinite(value) && value > 0.; }
}
uint32_t crc32(const char * data, size_t size) {
  uint32_t crc = 0xffffffffu;
  for (size_t i = 0; i < size; ++i) {
    crc ^= static_cast<unsigned char>(data[i]);
    for (int bit = 0; bit < 8; ++bit) crc = (crc >> 1) ^ ((crc & 1) ? 0xedb88320u : 0u);
  }
  return crc ^ 0xffffffffu;
}
size_t encode_frame(const char * payload, char * output, size_t capacity) {
  const size_t size = std::strlen(payload);
  if (size + 10 >= capacity || size + 10 > kFrameCapacity) return 0;
  const int n = std::snprintf(output, capacity, "%s*%08" PRIx32 "\n", payload, crc32(payload, size));
  return n > 0 && size_t(n) < capacity ? size_t(n) : 0;
}
bool parse_command(const char * input, size_t size, Command & command) {
  if (size && input[size - 1] == '\n') --size;
  if (size < 10 || size >= kFrameCapacity || std::memchr(input, '\0', size)) return false;
  const char * star = static_cast<const char *>(std::memchr(input, '*', size));
  if (!star || size_t(star - input) + 9 != size) return false;
  uint32_t checksum = 0;
  for (const char * p = star + 1; p != input + size; ++p) {
    unsigned digit;
    if (*p >= '0' && *p <= '9') digit = unsigned(*p - '0');
    else if (*p >= 'a' && *p <= 'f') digit = unsigned(*p - 'a' + 10);
    else if (*p >= 'A' && *p <= 'F') digit = unsigned(*p - 'A' + 10);
    else return false;
    checksum = (checksum << 4) | digit;
  }
  if (crc32(input, size_t(star - input)) != checksum) return false;
  char payload[kFrameCapacity];
  const size_t length = size_t(star - input);
  std::memcpy(payload, input, length); payload[length] = '\0';
  char * fields[10]; size_t count = 1; fields[0] = payload;
  for (size_t i = 0; i < length; ++i) if (payload[i] == ',') {
    if (count == 10) return false;
    payload[i] = '\0'; fields[count++] = payload + i + 1;
  }
  if (count < 4 || std::strcmp(fields[1], "2")) return false;
  Command next;
  if (!unsigned_value(fields[2], next.boot) || !unsigned_value(fields[3], next.sequence) || !next.sequence) return false;
  if (!std::strcmp(fields[0], "C") && count == 10) {
    next.operation = Operation::Control;
    for (size_t i = 0; i < 4; ++i) if (!real_value(fields[4 + i], next.wheel[i])) return false;
    if (!real_value(fields[8], next.lift_m) || (std::strcmp(fields[9], "0") && std::strcmp(fields[9], "1"))) return false;
    next.enabled = fields[9][0] == '1';
  } else if (count == 4) {
    if (!std::strcmp(fields[0], "H")) next.operation = Operation::Home;
    else if (!std::strcmp(fields[0], "R")) next.operation = Operation::Reset;
    else if (!std::strcmp(fields[0], "D")) next.operation = Operation::Stop;
    else return false;
  } else return false;
  command = next; return true;
}
bool Receiver::push(char byte, Command & command) {
  if (byte == '\n') {
    const bool ok = !discard_ && parse_command(buffer_, used_, command);
    if (!ok) ++rejected_;
    used_ = 0; discard_ = false; return ok;
  }
  if (discard_) return false;
  if (used_ >= sizeof(buffer_) - 1 || byte == '\0' || byte == '\r') {
    discard_ = true; return false;
  }
  buffer_[used_++] = byte; return false;
}
bool Config::valid() const {
  if (!base_ready || !positive(max_wheel_rad_s) || max_wheel_rad_s > 12. ||
      !positive(wheel_accel) || !positive(max_duty) || max_duty > 1. || watchdog_us < 50000 || watchdog_us > 300000) return false;
  for (size_t i = 0; i < 4; ++i) {
    const auto & g = gains[i];
    if (!positive(encoder_cpr[i]) || !positive(g.kp) || !std::isfinite(g.ki) || g.ki < 0. ||
        !std::isfinite(g.kd) || g.kd < 0.) return false;
  }
  return !lift_ready || (positive(steps_per_m) && positive(travel_m) && positive(lift_max_steps_s) &&
    steps_per_m * travel_m <= double(INT32_MAX) &&
    lift_max_steps_s <= 5000. && positive(lift_accel_steps_s2) && positive(home_steps_s) &&
    home_steps_s <= lift_max_steps_s && positive(backoff_m) && backoff_m < travel_m && positive(home_timeout_s));
}
double Pid::update(double target, double actual, double dt, const Gains & g, double bound) {
  if (!std::isfinite(target) || !std::isfinite(actual) || !positive(dt)) { reset(); return 0.; }
  if (std::abs(target) < 1e-6) { reset(); return 0.; }
  const double error = target - actual;
  const double derivative = primed_ ? -(actual - previous_) / dt : 0.;
  const double proposed = integral_ + g.ki * error * dt;
  const double raw = g.kp * error + proposed + g.kd * derivative;
  if (std::abs(raw) <= bound || raw * error < 0.) integral_ = std::clamp(proposed, -bound, bound);
  previous_ = actual; primed_ = true;
  // Overspeed stops PWM; the bridge wiring determines braking vs coasting.
  // Never reverse H-bridge polarity just to brake a wheel.
  const double output = std::clamp(g.kp * error + integral_ + g.kd * derivative, -bound, bound);
  return output * target > 0. ? output : 0.;
}
Controller::Controller(const Config & config, uint64_t boot) : config_(config), boot_(boot) {
  if (!config_.valid()) { faults_ = Configuration; mode_ = Mode::Locked; }
  else mode_ = Mode::Disarmed;
}
void Controller::stop() {
  output_.duty.fill(0.); output_.base_enable = false; output_.lift_steps_s = 0.;
  output_.lift_position_limit = false;
  target_.fill(0.); ramp_.fill(0.); stall_s_.fill(0.); direction_.fill(0);
  for (auto & p : pid_) p.reset();
  lift_rate_ = 0.;
}
void Controller::stop_for_fault() {
  const auto duty = output_.duty;
  const bool enabled = output_.base_enable;
  stop();
  // Opt-in manual-cutoff profile: fault telemetry remains latched, but wheels
  // retain PWM until explicit D/C-disabled or physical power removal. Lift stops.
  if (!config_.stop_base_on_fault && config_.valid()) {
    output_.duty = duty; output_.base_enable = enabled;
  }
}
void Controller::trip(uint32_t fault) {
  faults_ |= fault; mode_ = Mode::Faulted; homed_ = false; disabled_seen_ = false; stop_for_fault();
  output_.zero_lift = output_.clear_emergency = false;
  // Keep lift driver energized if initialized: stopping STEP is not a brake.
}
bool Controller::stopped() const {
  for (double v : velocity_) if (std::abs(v) > .15) return false;
  return true;
}
bool Controller::receive(const Command & c, uint64_t now, const Sensors & s) {
  if (c.boot != boot_ || c.sequence <= ack_ || !primed_) return false;
  if (c.operation == Operation::Reset) {
    if (!config_.valid() || s.estop || (s.low && s.high) || !s.io_ok || !stopped() || output_.base_enable || output_.lift_steps_s != 0.) return false;
    stop(); faults_ = 0; homed_ = false; mode_ = Mode::Disarmed; disabled_seen_ = true;
    output_.clear_emergency = true;
  } else if (c.operation == Operation::Stop) {
    if (mode_ == Mode::Homing) homed_ = false;
    stop(); if (!faults_) mode_ = Mode::Disarmed; disabled_seen_ = true;
  } else if (c.operation == Operation::Home) {
    if (faults_ || !config_.lift_ready || !disabled_seen_ || mode_ != Mode::Disarmed || !stopped() || s.estop || s.emergency_event || !s.io_ok) return false;
    stop(); homed_ = false; output_.lift_hold = true; mode_ = Mode::Homing;
    home_started_ = now; home_phase_ = s.low ? HomePhase::Backoff : HomePhase::Seek; released_ = false;
  } else {
    if (!std::isfinite(c.lift_m) || c.lift_m < 0. ||
        c.lift_m > (config_.lift_ready ? config_.travel_m : 0.)) return false;
    for (double v : c.wheel) if (!std::isfinite(v) || std::abs(v) > config_.max_wheel_rad_s) return false;
    if (!c.enabled) {
      // Disabled C frames are also the heartbeat during an explicit H operation.
      if (mode_ != Mode::Homing) { stop(); if (!faults_) mode_ = Mode::Disarmed; disabled_seen_ = true; }
    } else {
      if (faults_ || !disabled_seen_ || s.estop || s.emergency_event || !s.io_ok || mode_ == Mode::Homing ||
          (!homed_ && c.lift_m != 0.)) return false;
      if (mode_ != Mode::Active && !stopped()) return false;
      if (homed_ && (std::abs(c.lift_m * config_.steps_per_m - double(s.lift_steps)) > .5)) {
        if (!stopped()) return false;
        for (double v : c.wheel) if (std::abs(v) > 1e-6) return false;
      }
      target_ = c.wheel; lift_target_ = c.lift_m; mode_ = Mode::Active;
    }
  }
  ack_ = c.sequence; last_rx_ = now; return true;
}
void Controller::lift_tick(double dt, uint64_t now, const Sensors & s) {
  if (!config_.lift_ready) return;
  output_.lift_position_limit = false;
  double requested = 0.;
  if (mode_ == Mode::Homing) {
    if (double(now - home_started_) / 1e6 > config_.home_timeout_s) { trip(HomeTimeout); return; }
    if (home_phase_ == HomePhase::Seek) {
      if (s.low) { home_phase_ = HomePhase::Backoff; lift_rate_ = 0.; released_ = false; }
      else requested = -config_.home_steps_s;
    }
    if (home_phase_ == HomePhase::Backoff) {
      requested = config_.home_steps_s;
      if (!s.low && !released_) { released_ = true; release_step_ = s.lift_steps; }
      if (released_ && double(s.lift_steps - release_step_) >= config_.backoff_m * config_.steps_per_m) {
        home_phase_ = HomePhase::Approach; lift_rate_ = 0.; requested = 0.;
      }
    } else if (home_phase_ == HomePhase::Approach) {
      if (s.low) {
        lift_rate_ = 0.; output_.zero_lift = true; homed_ = true;
        lift_target_ = 0.; mode_ = Mode::Disarmed;
      } else requested = -.5 * config_.home_steps_s;
    }
  } else if (mode_ == Mode::Active && homed_) {
    output_.lift_position_limit = true;
    output_.lift_goal_steps = int64_t(std::llround(lift_target_ * config_.steps_per_m));
    const double error = double(output_.lift_goal_steps) - double(s.lift_steps);
    if (std::abs(error) > .5) {
      requested = sign(error) * std::min(config_.lift_max_steps_s,
        std::sqrt(2. * config_.lift_accel_steps_s2 * std::abs(error)));
    }
  }
  if ((requested < 0. && s.low) || (requested > 0. && s.high)) {
    if (mode_ == Mode::Homing && requested < 0. && s.low) requested = 0.;
    else { trip(Limit); return; }
  }
  if (requested != 0. && !stopped()) { trip(Encoder); return; }
  lift_rate_ = approach(lift_rate_, requested, config_.lift_accel_steps_s2 * dt);
  if (requested == 0.) lift_rate_ = 0.;
  output_.lift_steps_s = lift_rate_;
  if (lift_rate_ != 0.) output_.lift_hold = true;
}
void Controller::tick(uint64_t now, const Sensors & s) {
  output_.zero_lift = false; output_.clear_emergency = false;
  if (!primed_) { primed_ = true; previous_ticks_ = s.ticks; last_tick_ = now; return; }
  const uint64_t elapsed = now >= last_tick_ ? now - last_tick_ : UINT64_MAX;
  last_tick_ = now;
  if (s.estop || s.emergency_event) trip(Emergency);
  if (s.low && s.high) trip(Limit);
  if (!s.io_ok && config_.valid()) trip(Peripheral);
  if (elapsed > 50000 && (mode_ == Mode::Active || mode_ == Mode::Homing)) trip(Deadline);
  // Continue observing coasting wheels after a fault so reset can be accepted
  // once they actually stop. Uncalibrated builds must never divide by zero CPR.
  if (!config_.valid()) { stop(); previous_ticks_ = s.ticks; return; }
  if (!elapsed) return;
  const double dt = double(elapsed) / 1e6;
  for (size_t i = 0; i < 4; ++i) {
    const double actual = (double(s.ticks[i]) - double(previous_ticks_[i])) * kTau / config_.encoder_cpr[i] / dt;
    previous_ticks_[i] = s.ticks[i];
    if (!std::isfinite(actual) || std::abs(actual) > 2. * config_.max_wheel_rad_s + 5.) { trip(Encoder); return; }
    velocity_[i] += (actual - velocity_[i]) * dt / (.03 + dt);
    stationary_s_[i] = std::abs(velocity_[i]) < .15 ? stationary_s_[i] + dt : 0.;
  }
  if (faults_) { stop_for_fault(); return; }
  if ((mode_ == Mode::Active || mode_ == Mode::Homing) && (now < last_rx_ || now - last_rx_ >= config_.watchdog_us)) { trip(Watchdog); return; }
  output_.base_enable = mode_ == Mode::Active;
  for (size_t i = 0; i < 4; ++i) {
    if (mode_ != Mode::Active) { output_.duty[i] = 0.; pid_[i].reset(); continue; }
    const int wanted = sign(target_[i]);
    if (wanted && direction_[i] && wanted != direction_[i] && stationary_s_[i] < .03) {
      ramp_[i] = 0.; output_.duty[i] = 0.; pid_[i].reset(); continue;
    }
    if (wanted && wanted != direction_[i]) { direction_[i] = wanted; ramp_[i] = 0.; pid_[i].reset(); }
    ramp_[i] = approach(ramp_[i], target_[i], config_.wheel_accel * dt);
    output_.duty[i] = pid_[i].update(ramp_[i], velocity_[i], dt, config_.gains[i], config_.max_duty);
    const bool stalled = std::abs(ramp_[i]) > .5 && std::abs(output_.duty[i]) > .2 && std::abs(velocity_[i]) < .05;
    const bool wrong_direction = ramp_[i] * velocity_[i] < -.1;
    stall_s_[i] = stalled || wrong_direction ? stall_s_[i] + dt : 0.;
    if (stall_s_[i] > .5) { trip(Encoder); return; }
  }
  lift_tick(dt, now, s);
}
size_t Controller::telemetry(uint64_t now, const Sensors & s, char * output, size_t capacity) {
  char payload[kFrameCapacity];
  const int n = std::snprintf(payload, sizeof(payload),
    "S,2,%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRId64 ",%" PRId64 ",%" PRId64 ",%" PRId64 ",%" PRId64 ",%" PRIu32 ",%u,%u,%u,%u,%u,%u",
    boot_, ++state_seq_, now, ack_, s.ticks[0], s.ticks[1], s.ticks[2], s.ticks[3], s.lift_steps,
    faults_, unsigned(mode_), unsigned(homed_), unsigned(s.low), unsigned(s.high), unsigned(s.estop),
    unsigned(config_.valid() ? (config_.lift_ready ? 3 : 1) : 0));
  return n > 0 && size_t(n) < sizeof(payload) ? encode_frame(payload, output, capacity) : 0;
}
}  // namespace forklift_mcu
