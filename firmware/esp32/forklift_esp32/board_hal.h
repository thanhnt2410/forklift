// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "firmware_core.h"
namespace forklift_board {
bool initialize();
void control_heartbeat();
forklift_mcu::Sensors read_sensors();
void apply(const forklift_mcu::Outputs & output);
}  // namespace forklift_board
