#include "ui_perf_diag.h"

#include "lvgl.h"
#include "safety_manager.h"
#include <string.h>

#define UI_PERF_SAMPLE_PERIOD_MS        1000U
#define UI_PERF_BUSY_REFRESH_MS           16U
#define UI_PERF_OVERLOAD_REFRESH_MS       33U
#define UI_PERF_BUSY_HEAP_PCT              80U
#define UI_PERF_OVERLOAD_HEAP_PCT          90U
#define UI_PERF_OVERLOAD_FRAG_PCT          30U
#define UI_PERF_MIN_BIG_BLOCK_BYTES      2048U

typedef struct
{
    uint32_t refresh_count;
    uint32_t refresh_time_sum_ms;
    uint32_t refresh_time_max_ms;
    uint64_t pixels_sum;
    uint32_t pixels_max;
    uint32_t flush_sample_count;
    uint64_t flush_time_sum_us;
    uint32_t flush_time_max_us;
    uint32_t flush_wait_count;
    uint32_t flush_timeout_count;
    uint32_t ui_sample_count;
    uint64_t ui_time_sum_us;
    uint32_t ui_time_max_us;
} ui_perf_window_t;

static ui_perf_snapshot_t s_snapshot;
static ui_perf_window_t s_window;
static uint32_t s_last_sample_tick = 0U;
static uint32_t s_flush_start_cycles = 0U;
static uint32_t s_flush_start_tick = 0U;
static uint8_t s_flush_measure_active = 0U;

static uint32_t ui_perf_cycles_per_us(void)
{
    uint32_t cycles_per_us = SystemCoreClock / 1000000U;
    return (cycles_per_us == 0U) ? 1U : cycles_per_us;
}

static uint32_t ui_perf_cycles_to_us(uint32_t cycles)
{
    return cycles / ui_perf_cycles_per_us();
}

static uint16_t ui_perf_clamp_u16(uint32_t value)
{
    return (value > 0xFFFFU) ? 0xFFFFU : (uint16_t)value;
}

void UI_PerfDiag_Init(void)
{
    memset(&s_snapshot, 0, sizeof(s_snapshot));
    memset(&s_window, 0, sizeof(s_window));
    s_last_sample_tick = HAL_GetTick();
    s_flush_measure_active = 0U;
}

uint32_t UI_PerfDiag_BeginMeasure(void)
{
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U)
    {
        return 0U;
    }
    return DWT->CYCCNT;
}

void UI_PerfDiag_EndUiHandler(uint32_t start_cycles)
{
    uint32_t elapsed_us;

    if ((start_cycles == 0U) || ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U))
    {
        return;
    }

    elapsed_us = ui_perf_cycles_to_us(DWT->CYCCNT - start_cycles);
    s_window.ui_sample_count++;
    s_window.ui_time_sum_us += elapsed_us;
    if (elapsed_us > s_window.ui_time_max_us)
    {
        s_window.ui_time_max_us = elapsed_us;
    }
}

void UI_PerfDiag_OnRefresh(uint32_t time_ms, uint32_t pixels)
{
    s_window.refresh_count++;
    s_window.refresh_time_sum_ms += time_ms;
    s_window.pixels_sum += pixels;
    if (time_ms > s_window.refresh_time_max_ms)
    {
        s_window.refresh_time_max_ms = time_ms;
    }
    if (pixels > s_window.pixels_max)
    {
        s_window.pixels_max = pixels;
    }
}

void UI_PerfDiag_OnFlushStart(void)
{
    s_flush_start_cycles = UI_PerfDiag_BeginMeasure();
    s_flush_start_tick = HAL_GetTick();
    s_flush_measure_active = 1U;
}

void UI_PerfDiag_OnFlushWait(void)
{
    s_snapshot.flush_wait_count++;
    s_window.flush_wait_count++;
}

void UI_PerfDiag_OnFlushComplete(HAL_StatusTypeDef status)
{
    uint32_t elapsed_us;

    if (s_flush_measure_active != 0U)
    {
        if ((s_flush_start_cycles != 0U) &&
            ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U))
        {
            elapsed_us = ui_perf_cycles_to_us(DWT->CYCCNT - s_flush_start_cycles);
        }
        else
        {
            elapsed_us = (HAL_GetTick() - s_flush_start_tick) * 1000U;
        }

        s_window.flush_sample_count++;
        s_window.flush_time_sum_us += elapsed_us;
        if (elapsed_us > s_window.flush_time_max_us)
        {
            s_window.flush_time_max_us = elapsed_us;
        }
        s_flush_measure_active = 0U;
    }

    s_snapshot.flush_count++;
    if (status != HAL_OK)
    {
        s_snapshot.flush_error_count++;
    }
}

void UI_PerfDiag_OnFlushTimeout(void)
{
    s_snapshot.flush_timeout_count++;
    s_window.flush_timeout_count++;
}

void UI_PerfDiag_Process(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t elapsed_ms = now - s_last_sample_tick;
    lv_mem_monitor_t memory;
    ui_perf_status_t status = UI_PERF_STATUS_GOOD;

    if (elapsed_ms < UI_PERF_SAMPLE_PERIOD_MS)
    {
        return;
    }

    if (s_window.refresh_count != 0U)
    {
        s_snapshot.refresh_fps = ui_perf_clamp_u16(
            (s_window.refresh_count * 1000U) / elapsed_ms);
        s_snapshot.refresh_avg_ms_x10 = ui_perf_clamp_u16(
            (s_window.refresh_time_sum_ms * 10U) / s_window.refresh_count);
        s_snapshot.refresh_max_ms = ui_perf_clamp_u16(s_window.refresh_time_max_ms);
        s_snapshot.pixels_avg = (uint32_t)(s_window.pixels_sum / s_window.refresh_count);
        s_snapshot.pixels_max = s_window.pixels_max;
    }
    else
    {
        s_snapshot.refresh_fps = 0U;
        s_snapshot.refresh_avg_ms_x10 = 0U;
        s_snapshot.refresh_max_ms = 0U;
        s_snapshot.pixels_avg = 0U;
        s_snapshot.pixels_max = 0U;
    }

    if (s_window.flush_sample_count != 0U)
    {
        s_snapshot.flush_avg_us = ui_perf_clamp_u16(
            (uint32_t)(s_window.flush_time_sum_us / s_window.flush_sample_count));
        s_snapshot.flush_max_us = ui_perf_clamp_u16(s_window.flush_time_max_us);
    }
    else
    {
        s_snapshot.flush_avg_us = 0U;
        s_snapshot.flush_max_us = 0U;
    }

    if (s_window.ui_sample_count != 0U)
    {
        s_snapshot.ui_handler_avg_us = ui_perf_clamp_u16(
            (uint32_t)(s_window.ui_time_sum_us / s_window.ui_sample_count));
        s_snapshot.ui_handler_max_us = ui_perf_clamp_u16(s_window.ui_time_max_us);
    }
    else
    {
        s_snapshot.ui_handler_avg_us = 0U;
        s_snapshot.ui_handler_max_us = 0U;
    }

    lv_mem_monitor(&memory);
    s_snapshot.lv_mem_total = memory.total_size;
    s_snapshot.lv_mem_free = memory.free_size;
    s_snapshot.lv_mem_biggest_free = memory.free_biggest_size;
    s_snapshot.lv_mem_max_used = memory.max_used;
    s_snapshot.lv_mem_used_pct = memory.used_pct;
    s_snapshot.lv_mem_frag_pct = memory.frag_pct;
    s_snapshot.control_max_us = ui_perf_cycles_to_us(Safety_GetControlMaxCycles());
    s_snapshot.control_overrun_count = Safety_GetControlOverrunCount();

    if ((s_snapshot.refresh_max_ms >= UI_PERF_OVERLOAD_REFRESH_MS) ||
        (s_snapshot.lv_mem_used_pct >= UI_PERF_OVERLOAD_HEAP_PCT) ||
        (s_snapshot.lv_mem_frag_pct >= UI_PERF_OVERLOAD_FRAG_PCT) ||
        ((s_snapshot.lv_mem_biggest_free != 0U) &&
         (s_snapshot.lv_mem_biggest_free < UI_PERF_MIN_BIG_BLOCK_BYTES)) ||
        (s_window.flush_timeout_count != 0U))
    {
        status = UI_PERF_STATUS_OVERLOAD;
    }
    else if ((s_snapshot.refresh_max_ms >= UI_PERF_BUSY_REFRESH_MS) ||
             (s_snapshot.lv_mem_used_pct >= UI_PERF_BUSY_HEAP_PCT) ||
             (s_window.flush_wait_count != 0U))
    {
        status = UI_PERF_STATUS_BUSY;
    }
    s_snapshot.status = status;

    memset(&s_window, 0, sizeof(s_window));
    s_last_sample_tick = now;
}

void UI_PerfDiag_GetSnapshot(ui_perf_snapshot_t *snapshot)
{
    if (snapshot != NULL)
    {
        *snapshot = s_snapshot;
    }
}
