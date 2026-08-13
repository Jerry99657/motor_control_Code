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
    uint32_t safety_faults;
    uint32_t safety_warnings;
    uint32_t output_blocked_count;
    uint32_t battery_raw_adc;
    uint32_t battery_sample_age_ms;
    uint32_t battery_low_entry_count;
    uint32_t battery_stale_entry_count;
    float battery_voltage;
    int32_t motor_target_rpm[4];
    int32_t motor_measured_rpm[4];
    int16_t motor_duty_percent[4];
    uint32_t motor_max_error_rpm[4];
    uint32_t motor_encoder_suspect_events[4];
    float odometry_x_mm;
    float odometry_y_mm;
    float odometry_heading_deg;
    float odometry_body_vx_mm_s;
    float odometry_body_vy_mm_s;
    float odometry_body_wz_deg_s;
    float odometry_travel_distance_mm;
    float heading_reference_deg;
    float heading_error_deg;
    float heading_requested_wz_dps;
    float heading_correction_wz_dps;
    float heading_output_wz_dps;
    uint32_t odometry_update_count;
    uint32_t odometry_rejected_count;
    uint32_t odometry_timing_gap_count;
    uint8_t odometry_encoder_ready_mask;
    uint8_t odometry_last_sample_accepted;
    uint8_t heading_control_active;
    uint32_t runtime_reset_flags;
    uint32_t watchdog_feed_count;
    uint32_t watchdog_missed_vote_count;
    uint32_t watchdog_last_feed_age_ms;
    uint32_t foreground_age_ms;
    uint32_t control_age_ms;
    uint32_t stack_total_bytes;
    uint32_t stack_used_bytes;
    uint32_t stack_min_free_bytes;
    uint32_t last_fault_pc;
    uint32_t last_fault_lr;
    uint32_t last_fault_cfsr;
    uint8_t stack_guard_ok;
    uint8_t last_fault_valid;
    uint8_t runtime_fault_type;
    uint8_t battery_state;
    uint8_t battery_sample_valid;
    uint8_t motor_encoder_suspect_mask;
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
