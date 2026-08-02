#ifndef UI_PERF_DIAG_H
#define UI_PERF_DIAG_H

#include "main.h"
#include <stdint.h>

typedef enum
{
    UI_PERF_STATUS_GOOD = 0,
    UI_PERF_STATUS_BUSY,
    UI_PERF_STATUS_OVERLOAD
} ui_perf_status_t;

typedef struct
{
    uint16_t refresh_fps;
    uint16_t refresh_avg_ms_x10;
    uint16_t refresh_max_ms;
    uint32_t pixels_avg;
    uint32_t pixels_max;
    uint16_t flush_avg_us;
    uint16_t flush_max_us;
    uint32_t flush_count;
    uint32_t flush_wait_count;
    uint32_t flush_timeout_count;
    uint32_t flush_error_count;
    uint16_t ui_handler_avg_us;
    uint16_t ui_handler_max_us;
    uint32_t lv_mem_total;
    uint32_t lv_mem_free;
    uint32_t lv_mem_biggest_free;
    uint32_t lv_mem_max_used;
    uint8_t lv_mem_used_pct;
    uint8_t lv_mem_frag_pct;
    uint32_t control_max_us;
    uint32_t control_overrun_count;
    ui_perf_status_t status;
} ui_perf_snapshot_t;

void UI_PerfDiag_Init(void);
void UI_PerfDiag_Process(void);
uint32_t UI_PerfDiag_BeginMeasure(void);
void UI_PerfDiag_EndUiHandler(uint32_t start_cycles);
void UI_PerfDiag_OnRefresh(uint32_t time_ms, uint32_t pixels);
void UI_PerfDiag_OnFlushStart(void);
void UI_PerfDiag_OnFlushWait(void);
void UI_PerfDiag_OnFlushComplete(HAL_StatusTypeDef status);
void UI_PerfDiag_OnFlushTimeout(void);
void UI_PerfDiag_GetSnapshot(ui_perf_snapshot_t *snapshot);

#endif /* UI_PERF_DIAG_H */
