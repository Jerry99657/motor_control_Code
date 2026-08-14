#include "adc_sampler.h"
#include "battery_monitor.h"

static ADC_HandleTypeDef *s_adc;
static volatile uint8_t s_start_pending;
static uint32_t s_next_start_tick;

void AdcSampler_Init(ADC_HandleTypeDef *adc)
{
    s_adc = adc;
    s_start_pending = 0U;
    s_next_start_tick = 0U;
}

void AdcSampler_Process(void)
{
    uint8_t start_requested;
    uint32_t now;
    uint32_t primask;

    if (s_adc == NULL)
    {
        return;
    }

    now = HAL_GetTick();
    primask = __get_PRIMASK();
    __disable_irq();
    start_requested = s_start_pending;
    s_start_pending = 0U;
    if (primask == 0U)
    {
        __enable_irq();
    }

    if ((int32_t)(now - s_next_start_tick) >= 0)
    {
        start_requested = 1U;
        s_next_start_tick = now + BATTERY_MONITOR_SAMPLE_PERIOD_MS;
    }

    if ((start_requested != 0U) && (HAL_ADC_Start_IT(s_adc) != HAL_OK))
    {
        s_start_pending = 1U;
    }
}

void AdcSampler_RequestFromISR(void)
{
    s_start_pending = 1U;
}

void AdcSampler_ConversionCompleteFromISR(ADC_HandleTypeDef *adc)
{
    if ((adc != NULL) && (adc == s_adc))
    {
        BatteryMonitor_AdcSampleFromISR(HAL_ADC_GetValue(adc));
    }
}
