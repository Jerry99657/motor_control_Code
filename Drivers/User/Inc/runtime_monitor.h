#ifndef RUNTIME_MONITOR_H
#define RUNTIME_MONITOR_H

#include "main.h"
#include <stdint.h>

typedef enum
{
    RUNTIME_MONITOR_STATE_EARLY = 0,
    RUNTIME_MONITOR_STATE_BOOT,
    RUNTIME_MONITOR_STATE_RUNNING,
    RUNTIME_MONITOR_STATE_FATAL
} RuntimeMonitorState;

typedef enum
{
    RUNTIME_FAULT_NONE = 0U,
    RUNTIME_FAULT_HARD = 1U,
    RUNTIME_FAULT_MEMMANAGE = 2U,
    RUNTIME_FAULT_BUS = 3U,
    RUNTIME_FAULT_USAGE = 4U,
    RUNTIME_FAULT_ERROR_HANDLER = 5U,
    RUNTIME_FAULT_STACK_GUARD = 6U,
    RUNTIME_FAULT_CONTROL_STALL = 7U
} RuntimeFaultType;

typedef struct
{
    uint32_t reset_flags;
    RuntimeMonitorState state;
    uint32_t watchdog_feed_count;
    uint32_t watchdog_missed_vote_count;
    uint32_t last_feed_age_ms;
    uint32_t foreground_age_ms;
    uint32_t control_age_ms;
    uint32_t stack_total_bytes;
    uint32_t stack_used_bytes;
    uint32_t stack_min_free_bytes;
    uint8_t stack_guard_ok;
    uint8_t last_fault_valid;
    RuntimeFaultType last_fault_type;
    uint32_t last_fault_pc;
    uint32_t last_fault_lr;
    uint32_t last_fault_cfsr;
    uint32_t last_fault_hfsr;
    uint32_t last_fault_bfar;
    uint32_t last_fault_mmfar;
} RuntimeMonitorSnapshot;

void RuntimeMonitor_EarlyInit(void);
void RuntimeMonitor_AttachWatchdog(IWDG_HandleTypeDef *hiwdg);
void RuntimeMonitor_BootProgress(void);
void RuntimeMonitor_StackInit(void);
void RuntimeMonitor_StartRuntime(void);
void RuntimeMonitor_MainLoopHeartbeat(void);
void RuntimeMonitor_ForegroundHeartbeat(void);
void RuntimeMonitor_ControlTickFromISR(void);
void RuntimeMonitor_GetSnapshot(RuntimeMonitorSnapshot *snapshot);
const char *RuntimeMonitor_ResetReasonText(uint32_t reset_flags);
const char *RuntimeMonitor_FaultTypeText(RuntimeFaultType fault_type);

void RuntimeMonitor_SafeShutdownImmediate(void);
void RuntimeMonitor_RecordFatal(RuntimeFaultType fault_type,
                                const uint32_t *stacked_sp,
                                uint32_t exception_lr);
void RuntimeMonitor_CaptureFaultAndReset(RuntimeFaultType fault_type,
                                         const uint32_t *stacked_sp,
                                         uint32_t exception_lr)
    __attribute__((noreturn));

#endif /* RUNTIME_MONITOR_H */
