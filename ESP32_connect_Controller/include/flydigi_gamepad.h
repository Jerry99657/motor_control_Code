#pragma once

#include <stdint.h>

#include "flydigi_direwolf3_report.h"

namespace flydigi_gamepad {

void begin();
void service();

// Events are coalesced: the consumer receives the latest controller state.
bool takeState(flydigi_direwolf3::State* state);
bool takeConnectionChange(bool* connected);

bool isConnected();
int rssi();
uint32_t reportCount();
uint32_t droppedReportCount();

}  // namespace flydigi_gamepad
