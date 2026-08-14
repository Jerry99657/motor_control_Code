#ifndef TELEMETRY_SERVICE_H
#define TELEMETRY_SERVICE_H

#include <stdint.h>

void TelemetryService_Init(void);
void TelemetryService_Process(void);
void TelemetryService_UsbRx(const uint8_t *buffer, uint32_t length);

#endif /* TELEMETRY_SERVICE_H */
