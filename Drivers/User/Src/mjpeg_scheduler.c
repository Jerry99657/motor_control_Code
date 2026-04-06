#include "mjpeg_scheduler.h"

#include "main.h"

#define MJPEG_SCHEDULER_TIM7_TICK_US      100U
#define MJPEG_SCHEDULER_MIN_INTERVAL_MS   1U
#define MJPEG_SCHEDULER_MAX_INTERVAL_MS   6553U

extern TIM_HandleTypeDef htim7;

static volatile uint32_t s_mjpeg_frame_tick_count = 0U;
static volatile uint8_t s_mjpeg_frame_ready = 0U;

static uint32_t mjpeg_scheduler_calc_arr(uint32_t frame_interval_ms)
{
    uint32_t interval_ms;
    uint32_t ticks;

    interval_ms = frame_interval_ms;
    if (interval_ms < MJPEG_SCHEDULER_MIN_INTERVAL_MS)
    {
        interval_ms = MJPEG_SCHEDULER_MIN_INTERVAL_MS;
    }
    else if (interval_ms > MJPEG_SCHEDULER_MAX_INTERVAL_MS)
    {
        interval_ms = MJPEG_SCHEDULER_MAX_INTERVAL_MS;
    }

    ticks = (interval_ms * 1000U) / MJPEG_SCHEDULER_TIM7_TICK_US;
    if (ticks == 0U)
    {
        ticks = 1U;
    }

    return ticks - 1U;
}

HAL_StatusTypeDef MJPEG_Scheduler_SetFrameIntervalMs(uint32_t frame_interval_ms)
{
    uint32_t arr;

    arr = mjpeg_scheduler_calc_arr(frame_interval_ms);

    __disable_irq();
    s_mjpeg_frame_ready = 0U;
    __HAL_TIM_SET_COUNTER(&htim7, 0U);
    __HAL_TIM_SET_AUTORELOAD(&htim7, arr);
    __HAL_TIM_CLEAR_FLAG(&htim7, TIM_FLAG_UPDATE);
    __enable_irq();

    if ((TIM7->CR1 & TIM_CR1_CEN) == 0U)
    {
        return HAL_TIM_Base_Start_IT(&htim7);
    }

    return HAL_OK;
}

void MJPEG_Scheduler_OnTim7Tick(void)
{
    s_mjpeg_frame_tick_count++;
    s_mjpeg_frame_ready = 1U;
}

uint8_t MJPEG_Scheduler_ConsumeFrameTick(void)
{
    if (s_mjpeg_frame_ready != 0U)
    {
        s_mjpeg_frame_ready = 0U;
        return 1U;
    }

    return 0U;
}

uint32_t MJPEG_Scheduler_GetFrameTickCount(void)
{
    return s_mjpeg_frame_tick_count;
}
