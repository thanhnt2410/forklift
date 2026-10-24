// SPDX-License-Identifier: Apache-2.0
#include "board_hal.h"
#include "board_config.h"
#include <Arduino.h>
#include <driver/gpio.h>
#include <driver/pulse_cnt.h>
#include <esp_timer.h>
#include <algorithm>
#include <cmath>

#if !CONFIG_IDF_TARGET_ESP32
#error This pin/peripheral profile targets classic ESP32-WROOM, not ESP32-S3/C3.
#endif
namespace forklift_board {
namespace {
pcnt_unit_handle_t units[4]{};
int32_t last_count[4]{};
int64_t total_count[4]{};
hw_timer_t * timer{};
portMUX_TYPE pulse_lock = portMUX_INITIALIZER_UNLOCKED;
volatile int32_t requested_rate{};
volatile int64_t lift_count{};
int64_t lift_goal{};
bool lift_position_limit{};
volatile bool emergency_latched{};
volatile bool deadline_latched{};
int64_t control_heartbeat_us{};
bool initialized{}, step_high{};
int pulse_direction{};
uint32_t phase{};
constexpr uint32_t pulse_hz = 20000;
bool asserted(int pin, bool level) { return pin>=0 && gpio_get_level(gpio_num_t(pin)) == int(level); }

void ARDUINO_ISR_ATTR emergency_interrupt() {
  portENTER_CRITICAL_ISR(&pulse_lock);
  if (gpio_get_level(gpio_num_t(estop_pin)) == int(estop_active_level)) {
    emergency_latched = true;
    if (stop_base_on_fault && !base_enable_hardwired_high) gpio_set_level(gpio_num_t(base_enable_pin),0);
    if (enable_lift) gpio_set_level(gpio_num_t(step_pin), 0);
  }
  portEXIT_CRITICAL_ISR(&pulse_lock);
}
void ARDUINO_ISR_ATTR pulse_tick() {
  portENTER_CRITICAL_ISR(&pulse_lock);
  if (esp_timer_get_time() - control_heartbeat_us > 50000) deadline_latched = true;
  if ((deadline_latched || emergency_latched) && stop_base_on_fault && !base_enable_hardwired_high)
    gpio_set_level(gpio_num_t(base_enable_pin),0);
  if (!enable_lift) { portEXIT_CRITICAL_ISR(&pulse_lock); return; }
  const int32_t rate = requested_rate;
  const int direction = rate > 0 ? 1 : rate < 0 ? -1 : 0;
  const bool inhibited = emergency_latched || deadline_latched ||
    (lift_position_limit && ((direction < 0 && lift_count <= lift_goal) ||
                             (direction > 0 && lift_count >= lift_goal))) ||
    (estop_pin >= 0 && gpio_get_level(gpio_num_t(estop_pin)) == int(estop_active_level)) ||
    (direction < 0 && gpio_get_level(gpio_num_t(lower_pin)) == int(limit_active_level)) ||
    (direction > 0 && upper_pin >= 0 && gpio_get_level(gpio_num_t(upper_pin)) == int(limit_active_level)) ||
    (single_lower_bench_profile && direction > 0 && lift_count >= lift_max_steps);
  if (!direction || inhibited) {
    gpio_set_level(gpio_num_t(step_pin), 0); step_high = false; phase = 0;
    portEXIT_CRITICAL_ISR(&pulse_lock); return;
  }
  if (direction != pulse_direction) {
    gpio_set_level(gpio_num_t(step_pin), 0); step_high = false;
    gpio_set_level(gpio_num_t(direction_pin), direction > 0 ? step_up_level : !step_up_level);
    pulse_direction = direction; phase = 0;  // >=50 us DIR setup before STEP.
    portEXIT_CRITICAL_ISR(&pulse_lock); return;
  }
  phase += uint32_t(rate > 0 ? rate : -rate);
  if (step_high) {
    gpio_set_level(gpio_num_t(step_pin), 0); step_high = false;
  } else if (phase >= pulse_hz) {
    phase -= pulse_hz; gpio_set_level(gpio_num_t(step_pin), 1);
    step_high = true; lift_count += direction;
  }
  portEXIT_CRITICAL_ISR(&pulse_lock);
}
bool pin_plan_valid() {
  bool used[40]{};
  const auto claim = [&](int pin, bool output) {
    if (pin < 0 || pin >= 40 || used[pin] || (pin >= 6 && pin <= 11) ||
        ((pin == 0 || pin == 2 || pin == 5 || pin == 12 || pin == 15) &&
          !strapping_levels_reviewed && !base_bench_profile) ||
        !GPIO_IS_VALID_GPIO(pin) || (output && !GPIO_IS_VALID_OUTPUT_GPIO(pin))) return false;
    used[pin] = true; return true;
  };
  used[1] = used[3] = true; // USB-UART transport.
  for (int i = 0; i < 4; ++i) {
    if (std::abs(encoder_sign[i]) != 1 || std::abs(motor_sign[i]) != 1) return false;
    for (int j = 0; j < 2; ++j) if (!claim(pwm[i][j],true) || !claim(encoder[i][j],false)) return false;
  }
  if (base_enable_hardwired_high) {
    if (base_enable_pin != -1) return false;
  } else if (!claim(base_enable_pin,true)) return false;
  if (estop_pin>=0) { if (!claim(estop_pin,false)) return false; }
  else if (stop_base_on_fault || (enable_lift && !single_lower_bench_profile)) return false;
  if (enable_lift) {
    if (!claim(step_pin,true) || !claim(direction_pin,true) || !claim(lower_pin,false)) return false;
    if (upper_pin >= 0) { if (!claim(upper_pin,false)) return false; }
    else if (!single_lower_bench_profile) return false;
    if (lift_enable_pin >= 0 && !claim(lift_enable_pin,true)) return false;
  }
  return true;
}
void all_off() {
  if (!base_enable_hardwired_high) gpio_set_level(gpio_num_t(base_enable_pin),0);
  for (const auto & pins:pwm) for (int pin:pins) ledcWrite(pin,0);
  portENTER_CRITICAL(&pulse_lock);requested_rate=0;portEXIT_CRITICAL(&pulse_lock);
}
}
bool initialize() {
  if (!base_initialization_allowed() || !pin_plan_valid()) return false;
  if (!base_enable_hardwired_high) {
    pinMode(base_enable_pin,OUTPUT);digitalWrite(base_enable_pin,LOW);
  }
  if (estop_pin>=0) {
    pinMode(estop_pin,INPUT_PULLUP);attachInterrupt(estop_pin,emergency_interrupt,CHANGE);
  }
  if (enable_lift) {
    pinMode(lower_pin,INPUT_PULLUP);
    if (upper_pin >= 0) pinMode(upper_pin,INPUT_PULLUP);
    pinMode(step_pin,OUTPUT); digitalWrite(step_pin,LOW);
    pinMode(direction_pin,OUTPUT); digitalWrite(direction_pin,LOW);
    if (lift_enable_pin >= 0) {
      pinMode(lift_enable_pin,OUTPUT); digitalWrite(lift_enable_pin,!lift_enable_level);
    }
  }
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 2; ++j) {
      pinMode(pwm[i][j],OUTPUT); digitalWrite(pwm[i][j],LOW);
      if (!ledcAttachChannel(pwm[i][j],20000,10,2*i+j)) { all_off(); return false; }
      ledcWrite(pwm[i][j],0);
      pinMode(encoder[i][j],encoder[i][j]>=34 ? INPUT : INPUT_PULLUP);
    }
    pcnt_unit_config_t config{};
    config.low_limit=-30000; config.high_limit=30000; config.flags.accum_count=1;
    if (pcnt_new_unit(&config,&units[i]) != ESP_OK) { all_off(); return false; }
    pcnt_glitch_filter_config_t filter{}; filter.max_glitch_ns=500;
    if (pcnt_unit_set_glitch_filter(units[i],&filter)!=ESP_OK) { all_off(); return false; }
    for (int j=0;j<2;++j) {
      pcnt_chan_config_t channel{};
      channel.edge_gpio_num=encoder[i][j];channel.level_gpio_num=encoder[i][1-j];
      pcnt_channel_handle_t handle{};
      if (pcnt_new_channel(units[i],&channel,&handle)!=ESP_OK ||
          pcnt_channel_set_edge_action(handle,j ? PCNT_CHANNEL_EDGE_ACTION_INCREASE : PCNT_CHANNEL_EDGE_ACTION_DECREASE,
            j ? PCNT_CHANNEL_EDGE_ACTION_DECREASE : PCNT_CHANNEL_EDGE_ACTION_INCREASE)!=ESP_OK ||
          pcnt_channel_set_level_action(handle,PCNT_CHANNEL_LEVEL_ACTION_KEEP,PCNT_CHANNEL_LEVEL_ACTION_INVERSE)!=ESP_OK) {
        all_off(); return false;
      }
    }
    if (pcnt_unit_add_watch_point(units[i],-30000)!=ESP_OK || pcnt_unit_add_watch_point(units[i],30000)!=ESP_OK ||
        pcnt_unit_enable(units[i])!=ESP_OK || pcnt_unit_clear_count(units[i])!=ESP_OK || pcnt_unit_start(units[i])!=ESP_OK) {
      all_off(); return false;
    }
  }
  // Timer latches loop stalls and inhibits STEP. In the manual-cutoff wheel
  // profile it deliberately leaves PWM running. A deadline latch requires reboot.
  timer=timerBegin(1000000);
  if (!timer) { all_off(); return false; }
  control_heartbeat_us=esp_timer_get_time();
  timerAttachInterrupt(timer,pulse_tick); timerAlarm(timer,50,true,0);
  initialized=true;return true;
}
void control_heartbeat() {
  portENTER_CRITICAL(&pulse_lock);
  control_heartbeat_us=esp_timer_get_time();
  portEXIT_CRITICAL(&pulse_lock);
}
forklift_mcu::Sensors read_sensors() {
  forklift_mcu::Sensors s;
  if (!initialized) { s.io_ok=false; return s; }
  if (deadline_latched) s.io_ok=false;
  for (int i=0;i<4;++i) {
    int count;
    if (pcnt_unit_get_count(units[i],&count)!=ESP_OK) s.io_ok=false;
    else {
      const auto delta=int32_t(uint32_t(count)-uint32_t(last_count[i]));
      total_count[i]+=int64_t(delta)*encoder_sign[i];last_count[i]=count;
    }
    s.ticks[i]=total_count[i];
  }
  s.estop=asserted(estop_pin,estop_active_level); s.emergency_event=emergency_latched;
  if (enable_lift) { s.low=asserted(lower_pin,limit_active_level);s.high=asserted(upper_pin,limit_active_level); }
  portENTER_CRITICAL(&pulse_lock);s.lift_steps=lift_count;portEXIT_CRITICAL(&pulse_lock);
  return s;
}
void apply(const forklift_mcu::Outputs & output) {
  if (!initialized) return;
  portENTER_CRITICAL(&pulse_lock);
  if (output.clear_emergency && !asserted(estop_pin,estop_active_level)) emergency_latched=false;
  portEXIT_CRITICAL(&pulse_lock);
  const bool healthy=!emergency_latched && !deadline_latched && !asserted(estop_pin,estop_active_level);
  const bool allowed=output.base_enable && (!stop_base_on_fault || healthy);
  if (!allowed && !base_enable_hardwired_high) gpio_set_level(gpio_num_t(base_enable_pin),0);
  for (int i=0;i<4;++i) {
    const double duty=allowed ? std::clamp(output.duty[i]*motor_sign[i],-1.,1.) : 0.;
    // Clear the opposite input before asserting a direction.
    if (duty>0.) { ledcWrite(pwm[i][1],0);ledcWrite(pwm[i][0],uint32_t(std::lround(1023.*duty))); }
    else { ledcWrite(pwm[i][0],0);ledcWrite(pwm[i][1],uint32_t(std::lround(-1023.*duty))); }
  }
  // Critical section also prevents a GPIO E-stop ISR being undone by this write.
  portENTER_CRITICAL(&pulse_lock);
  const bool still_healthy=!emergency_latched && !deadline_latched && !asserted(estop_pin,estop_active_level);
  if (allowed && !base_enable_hardwired_high && (!stop_base_on_fault || still_healthy)) gpio_set_level(gpio_num_t(base_enable_pin),1);
  portEXIT_CRITICAL(&pulse_lock);
  if (enable_lift) {
    portENTER_CRITICAL(&pulse_lock);
    requested_rate=(emergency_latched || deadline_latched) ? 0 : int32_t(std::lround(std::clamp(output.lift_steps_s,-5000.,5000.)));
    lift_goal=output.lift_goal_steps;lift_position_limit=output.lift_position_limit;
    if (output.zero_lift) { lift_count=0;phase=0; }
    portEXIT_CRITICAL(&pulse_lock);
    if (lift_enable_pin>=0) digitalWrite(lift_enable_pin,output.lift_hold ? lift_enable_level : !lift_enable_level);
  }
}
}  // namespace forklift_board
