// SPDX-License-Identifier: Apache-2.0
#include <Arduino.h>
#include <esp_random.h>
#include <esp_timer.h>
#include <optional>
#include "board_config.h"
#include "board_hal.h"

forklift_mcu::Receiver receiver;
std::optional<forklift_mcu::Controller> controller;
uint64_t next_control{}, next_state{};

void setup() {
  Serial.setRxBufferSize(1024); Serial.setTxBufferSize(1024);
  Serial.begin(forklift_board::baud);
  auto config=forklift_board::control_config();
  // No actuator/encoder GPIO is configured in the distributed, locked build.
  if (config.valid() && !forklift_board::initialize()) config.base_ready=false;
  const uint64_t boot=(uint64_t(esp_random())<<32)|esp_random();
  controller.emplace(config,boot);
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
    if (receiver.push(char(Serial.read()),command)) {
      controller->receive(command,uint64_t(esp_timer_get_time()),forklift_board::read_sensors());
      forklift_board::apply(controller->outputs());
      controller->acknowledge_outputs();
    }
  }
  if (now>=next_state) {
    char frame[forklift_mcu::kFrameCapacity+1];
    const size_t size=controller->telemetry(now,forklift_board::read_sensors(),frame,sizeof(frame));
    if (size && Serial.availableForWrite()>=int(size)) Serial.write(reinterpret_cast<const uint8_t *>(frame),size);
    next_state=now+forklift_board::telemetry_period_us;
  }
  delay(1);
}
