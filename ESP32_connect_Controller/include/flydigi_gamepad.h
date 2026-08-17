#pragma once

#include <stdint.h>

#include "flydigi_direwolf3_report.h"

namespace flydigi_gamepad {

void begin();
void service();

// Pause only target discovery; an already connected physical pad is retained.
// This releases RF time slices for high-rate Wi-Fi camera streaming.
void setScanSuspended(bool suspended);
bool isScanSuspended();

// Fully stop/restart the BLE controller when maximum Wi-Fi throughput is
// required. Suspension is refused while a physical controller is connected.
bool setRadioSuspended(bool suspended);
bool isRadioSuspended();

// Events are coalesced: the consumer receives the latest controller state.
bool takeState(flydigi_direwolf3::State* state);
bool takeConnectionChange(bool* connected);

bool isConnected();
int rssi();
uint32_t reportCount();
uint32_t droppedReportCount();

}  // namespace flydigi_gamepad
