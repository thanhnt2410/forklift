// SPDX-License-Identifier: Apache-2.0
#include <Arduino.h>
#include <esp_random.h>
#include <esp_timer.h>
#include "board_config.h"
#include "board_hal.h"

// Serial Monitor: 115200 baud, Newline (hoac Both NL & CR).
// C,1,0,0,0,0,1 : banh truoc trai 1 rad/s, chi can gui mot lan.
// D: dung tat ca. H: ha toi da 3s/cong tac. U: nang them 2000 step.
// R: xoa loi khi da dung (khac phim R tren ban phim ROS).
// Thu tu banh: truoc trai, truoc phai, sau trai, sau phai.

forklift_mcu::Config motorConfig = forklift_board::control_config();
forklift_mcu::Controller controller(motorConfig, 0);
forklift_mcu::SimpleReceiver serialReceiver;
forklift_mcu::Receiver legacyReceiver;  // Chi dung neu simple_serial=false.

uint64_t bootId = 0;
uint64_t commandNumber = 0;
unsigned long lastControlMs = 0;
unsigned long lastStatusMs = 0;

const unsigned long CONTROL_INTERVAL_MS = forklift_board::control_period_us / 1000;
const unsigned long STATUS_INTERVAL_MS = forklift_board::telemetry_period_us / 1000;

void applyMotorOutputs() {
  forklift_board::apply(controller.outputs());
  controller.acknowledge_outputs();
}

void updateMotors() {
  const auto sensors = forklift_board::read_sensors();
  controller.tick(esp_timer_get_time(), sensors);
  applyMotorOutputs();
  forklift_board::control_heartbeat();
}

void readSerialCommands() {
  // Doc tung ky tu, khong cho Serial chan vong dieu khien motor.
  // Gioi han so byte moi vong de van cap nhat motor khi nhan lien tuc.
  for (int bytesRead = 0; bytesRead < 256 && Serial.available(); bytesRead++) {
    const char receivedChar = Serial.read();
    forklift_mcu::Command command;
    bool completeCommand;

    if (forklift_board::simple_serial) {
      completeCommand = serialReceiver.push(receivedChar, motorConfig.steps_per_m, command);
    } else {
      completeCommand = legacyReceiver.push(receivedChar, command);
    }

    if (!completeCommand) {
      continue;
    }

    if (forklift_board::simple_serial) {
      command.boot = bootId;
      command.sequence = ++commandNumber;
    }

    controller.receive(command, esp_timer_get_time(), forklift_board::read_sensors());
    applyMotorOutputs();  // Lenh D dung PWM ngay khi nhan du dong.
  }
}

void sendStatus() {
  char line[forklift_mcu::kFrameCapacity + 1];
  const auto sensors = forklift_board::read_sensors();
  size_t length;

  if (forklift_board::simple_serial) {
    length = controller.simple_telemetry(esp_timer_get_time(), sensors, line, sizeof(line));
  } else {
    length = controller.telemetry(esp_timer_get_time(), sensors, line, sizeof(line));
  }

  // Chi gui khi bo dem du cho, tranh lam tre vong dieu khien.
  if (length > 0 && Serial.availableForWrite() >= int(length)) {
    Serial.write(reinterpret_cast<const uint8_t *>(line), length);
  }
}

void setup() {
  Serial.setRxBufferSize(1024);
  Serial.setTxBufferSize(1024);
  Serial.begin(forklift_board::baud);

  if (motorConfig.valid() && !forklift_board::initialize()) {
    motorConfig.base_ready = false;
  }

  bootId = (uint64_t(esp_random()) << 32) | esp_random();
  controller = forklift_mcu::Controller(motorConfig, bootId);
  updateMotors();  // Khoi dong PWM=0, chua chay banh hay nang.
  sendStatus();
  lastControlMs = millis();
  lastStatusMs = lastControlMs;
}

void loop() {
  const unsigned long now = millis();

  if (now - lastControlMs >= CONTROL_INTERVAL_MS) {
    lastControlMs = now;
    updateMotors();
  }

  readSerialCommands();

  if (now - lastStatusMs >= STATUS_INTERVAL_MS) {
    lastStatusMs = now;
    sendStatus();
  }

  delay(1);
}
