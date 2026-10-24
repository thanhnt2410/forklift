// SPDX-License-Identifier: Apache-2.0
#include <Arduino.h>
#include <esp_random.h>
#include <esp_timer.h>
#include <optional>
#include "board_config.h"
#include "board_hal.h"

forklift_mcu::Receiver receiver;
forklift_mcu::SimpleReceiver simple_receiver;
std::optional<forklift_mcu::Controller> controller;
uint64_t next_control{}, next_state{};
uint64_t boot_id{}, serial_sequence{};
double serial_steps_per_m{};

void setup() {
  Serial.setRxBufferSize(1024); Serial.setTxBufferSize(1024);
  Serial.begin(forklift_board::baud);
  auto config=forklift_board::control_config();
  serial_steps_per_m=config.steps_per_m;
  // Bench profile initializes wheel GPIO at zero duty, then waits for host arm.
  if (config.valid() && !forklift_board::initialize()) config.base_ready=false;
  boot_id=(uint64_t(esp_random())<<32)|esp_random();
  controller.emplace(config,boot_id);
  next_control=next_state=uint64_t(esp_timer_get_time());
  controller->tick(next_control,forklift_board::read_sensors());
}
void loop() {
  const uint64_t now=uint64_t(esp_timer_get_time());
  if (now>=next_control) {
    controller->tick(now,forklift_board::read_sensors());
    forklift_board::apply(controller->outputs());
    controller->acknowledge_outputs();
    forklift_board::control_heartbeat();
    next_control=now+forklift_board::control_period_us;
  }
  for (unsigned budget=0;budget<256 && Serial.available();++budget) {
    forklift_mcu::Command command;
    const char byte=char(Serial.read());
    const bool valid=forklift_board::simple_serial ?
      simple_receiver.push(byte,serial_steps_per_m,command) : receiver.push(byte,command);
    if (valid) {
      if (forklift_board::simple_serial) {command.boot=boot_id;command.sequence=++serial_sequence;}
      controller->receive(command,uint64_t(esp_timer_get_time()),forklift_board::read_sensors());
      forklift_board::apply(controller->outputs());
      controller->acknowledge_outputs();
    }
  }
  if (now>=next_state) {
    char frame[forklift_mcu::kFrameCapacity+1];
    const auto sensors=forklift_board::read_sensors();
    const size_t size=forklift_board::simple_serial ?
      controller->simple_telemetry(now,sensors,frame,sizeof(frame)) :
      controller->telemetry(now,sensors,frame,sizeof(frame));
    if (size && Serial.availableForWrite()>=int(size)) Serial.write(reinterpret_cast<const uint8_t *>(frame),size);
    next_state=now+forklift_board::telemetry_period_us;
  }
  delay(1);
}
