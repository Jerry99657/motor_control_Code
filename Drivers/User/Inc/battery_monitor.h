#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ADC1 is sampled from foreground context at this rate.  The conversion
 * complete callback only publishes the raw sample. */
#define BATTERY_MONITOR_SAMPLE_PERIOD_MS       100U

typedef enum
{
    BATTERY_MONITOR_WAITING = 0U,
    BATTERY_MONITOR_NORMAL,
    BATTERY_MONITOR_LOW,
    BATTERY_MONITOR_STALE
} BatteryMonitorState;

typedef struct
{
    uint32_t raw_adc;
    uint32_t sample_sequence;
    uint32_t sample_age_ms;
    uint32_t low_entry_count;
    uint32_t stale_entry_count;
    float instant_voltage;
    float filtered_voltage;
    BatteryMonitorState state;
    uint8_t sample_valid;
    uint8_t low_active;
    uint8_t stale_active;
} BatteryMonitorSnapshot;

void BatteryMonitor_Init(void);
void BatteryMonitor_AdcSampleFromISR(uint32_t raw_adc);
void BatteryMonitor_Process(void);
void BatteryMonitor_GetSnapshot(BatteryMonitorSnapshot *snapshot);
uint8_t BatteryMonitor_IsLow(void);
const char *BatteryMonitor_StateText(BatteryMonitorState state);

#ifdef __cplusplus
}
#endif

#endif /* BATTERY_MONITOR_H */
