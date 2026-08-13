#include "battery_monitor.h"

#include "main.h"
#include "safety_manager.h"

#include <string.h>

/* Board divider: battery -> PC0/ADC1_INP10 is currently 7:1.  Keep the
 * calibration factor separate so it can be corrected later with a meter. */
#define BATTERY_ADC_FULL_SCALE_COUNTS          65535.0f
#define BATTERY_ADC_REFERENCE_VOLTAGE          3.3f
#define BATTERY_DIVIDER_RATIO                  7.0f
#define BATTERY_CALIBRATION_GAIN               1.0f

#define BATTERY_LOW_THRESHOLD_VOLTAGE          11.0f
#define BATTERY_RECOVER_THRESHOLD_VOLTAGE      11.3f
#define BATTERY_LOW_CONFIRM_MS                  500U
#define BATTERY_RECOVER_CONFIRM_MS             2000U
#define BATTERY_STALE_TIMEOUT_MS                1500U
#define BATTERY_FILTER_ALPHA                    0.25f
#define BATTERY_MEDIAN_SAMPLE_COUNT             3U

static volatile uint32_t s_isr_raw_adc = 0U;
static volatile uint32_t s_isr_sample_tick = 0U;
static volatile uint32_t s_isr_sequence = 0U;

static BatteryMonitorSnapshot s_snapshot;
static uint32_t s_processed_sequence = 0U;
static uint32_t s_raw_history[BATTERY_MEDIAN_SAMPLE_COUNT];
static uint8_t s_raw_history_count = 0U;
static uint8_t s_raw_history_index = 0U;
static uint32_t s_low_candidate_tick = 0U;
static uint32_t s_recover_candidate_tick = 0U;
static uint8_t s_low_candidate_active = 0U;
static uint8_t s_recover_candidate_active = 0U;

static uint32_t battery_lock(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return primask;
}

static void battery_unlock(uint32_t primask)
{
    __DMB();
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static uint32_t battery_median_raw(void)
{
    uint32_t a = s_raw_history[0];
    uint32_t b;
    uint32_t c;
    uint32_t temporary;

    if (s_raw_history_count == 1U)
    {
        return a;
    }

    b = s_raw_history[1];
    if (s_raw_history_count == 2U)
    {
        return (a + b) / 2U;
    }

    c = s_raw_history[2];
    if (a > b)
    {
        temporary = a;
        a = b;
        b = temporary;
    }
    if (b > c)
    {
        temporary = b;
        b = c;
        c = temporary;
    }
    if (a > b)
    {
        b = a;
    }
    return b;
}

static float battery_raw_to_voltage(uint32_t raw_adc)
{
    return ((float)raw_adc / BATTERY_ADC_FULL_SCALE_COUNTS) *
           BATTERY_ADC_REFERENCE_VOLTAGE * BATTERY_DIVIDER_RATIO *
           BATTERY_CALIBRATION_GAIN;
}

static void battery_set_stale(uint8_t active)
{
    active = (active != 0U) ? 1U : 0U;
    if ((active != 0U) && (s_snapshot.stale_active == 0U))
    {
        s_snapshot.stale_entry_count++;
    }
    s_snapshot.stale_active = active;
    Safety_SetWarning(SAFETY_WARNING_BATTERY_STALE, active);
}

void BatteryMonitor_Init(void)
{
    uint32_t primask = battery_lock();

    s_isr_raw_adc = 0U;
    s_isr_sample_tick = 0U;
    s_isr_sequence = 0U;
    battery_unlock(primask);

    memset(&s_snapshot, 0, sizeof(s_snapshot));
    memset(s_raw_history, 0, sizeof(s_raw_history));
    s_snapshot.state = BATTERY_MONITOR_WAITING;
    s_processed_sequence = 0U;
    s_raw_history_count = 0U;
    s_raw_history_index = 0U;
    s_low_candidate_tick = 0U;
    s_recover_candidate_tick = 0U;
    s_low_candidate_active = 0U;
    s_recover_candidate_active = 0U;
    Safety_SetWarning(SAFETY_WARNING_BATTERY_LOW, 0U);
    Safety_SetWarning(SAFETY_WARNING_BATTERY_STALE, 0U);
}

void BatteryMonitor_AdcSampleFromISR(uint32_t raw_adc)
{
    s_isr_raw_adc = raw_adc;
    s_isr_sample_tick = HAL_GetTick();
    __DMB();
    s_isr_sequence++;
}

void BatteryMonitor_Process(void)
{
    uint32_t raw_adc;
    uint32_t sample_tick;
    uint32_t sequence;
    uint32_t now = HAL_GetTick();
    uint32_t primask = battery_lock();

    raw_adc = s_isr_raw_adc;
    sample_tick = s_isr_sample_tick;
    sequence = s_isr_sequence;
    battery_unlock(primask);

    if ((sequence != 0U) && (sequence != s_processed_sequence))
    {
        uint32_t median_raw;
        float median_voltage;

        s_processed_sequence = sequence;
        s_raw_history[s_raw_history_index] = raw_adc;
        s_raw_history_index = (uint8_t)((s_raw_history_index + 1U) %
                                        BATTERY_MEDIAN_SAMPLE_COUNT);
        if (s_raw_history_count < BATTERY_MEDIAN_SAMPLE_COUNT)
        {
            s_raw_history_count++;
        }

        median_raw = battery_median_raw();
        median_voltage = battery_raw_to_voltage(median_raw);
        s_snapshot.raw_adc = raw_adc;
        s_snapshot.sample_sequence = sequence;
        s_snapshot.instant_voltage = battery_raw_to_voltage(raw_adc);
        if (s_snapshot.sample_valid == 0U)
        {
            s_snapshot.filtered_voltage = median_voltage;
            s_snapshot.sample_valid = 1U;
        }
        else
        {
            s_snapshot.filtered_voltage += BATTERY_FILTER_ALPHA *
                (median_voltage - s_snapshot.filtered_voltage);
        }

        battery_set_stale(0U);

        if (s_snapshot.low_active == 0U)
        {
            s_recover_candidate_active = 0U;
            if (s_snapshot.filtered_voltage < BATTERY_LOW_THRESHOLD_VOLTAGE)
            {
                if (s_low_candidate_active == 0U)
                {
                    s_low_candidate_active = 1U;
                    s_low_candidate_tick = now;
                }
                else if ((uint32_t)(now - s_low_candidate_tick) >=
                         BATTERY_LOW_CONFIRM_MS)
                {
                    s_snapshot.low_active = 1U;
                    s_snapshot.low_entry_count++;
                    s_low_candidate_active = 0U;
                    Safety_SetWarning(SAFETY_WARNING_BATTERY_LOW, 1U);
                }
            }
            else
            {
                s_low_candidate_active = 0U;
            }
        }
        else
        {
            s_low_candidate_active = 0U;
            if (s_snapshot.filtered_voltage > BATTERY_RECOVER_THRESHOLD_VOLTAGE)
            {
                if (s_recover_candidate_active == 0U)
                {
                    s_recover_candidate_active = 1U;
                    s_recover_candidate_tick = now;
                }
                else if ((uint32_t)(now - s_recover_candidate_tick) >=
                         BATTERY_RECOVER_CONFIRM_MS)
                {
                    s_snapshot.low_active = 0U;
                    s_recover_candidate_active = 0U;
                    Safety_SetWarning(SAFETY_WARNING_BATTERY_LOW, 0U);
                }
            }
            else
            {
                s_recover_candidate_active = 0U;
            }
        }
    }

    if (sequence == 0U)
    {
        s_snapshot.sample_age_ms = now;
        s_snapshot.state = BATTERY_MONITOR_WAITING;
        return;
    }

    s_snapshot.sample_age_ms = now - sample_tick;
    if (s_snapshot.sample_age_ms > BATTERY_STALE_TIMEOUT_MS)
    {
        battery_set_stale(1U);
    }

    if (s_snapshot.stale_active != 0U)
    {
        s_snapshot.state = BATTERY_MONITOR_STALE;
    }
    else if (s_snapshot.low_active != 0U)
    {
        s_snapshot.state = BATTERY_MONITOR_LOW;
    }
    else
    {
        s_snapshot.state = BATTERY_MONITOR_NORMAL;
    }
}

void BatteryMonitor_GetSnapshot(BatteryMonitorSnapshot *snapshot)
{
    if (snapshot != NULL)
    {
        uint32_t primask = battery_lock();
        *snapshot = s_snapshot;
        battery_unlock(primask);
    }
}

uint8_t BatteryMonitor_IsLow(void)
{
    return s_snapshot.low_active;
}

const char *BatteryMonitor_StateText(BatteryMonitorState state)
{
    switch (state)
    {
        case BATTERY_MONITOR_NORMAL: return "NORMAL";
        case BATTERY_MONITOR_LOW: return "LOW";
        case BATTERY_MONITOR_STALE: return "STALE";
        default: return "WAIT";
    }
}
