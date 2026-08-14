#ifndef APP_HEALTH_H
#define APP_HEALTH_H

#include "app_event.h"
#include "app_scheduler.h"
#include "command_control.h"
#include "runtime_monitor.h"
#include <stdint.h>

#define APP_HEALTH_FLAG_EVENT_DROP       (1UL << 0U)
#define APP_HEALTH_FLAG_UART_RX_OVERFLOW (1UL << 1U)
#define APP_HEALTH_FLAG_UART_TX_DROP     (1UL << 2U)
#define APP_HEALTH_FLAG_WATCHDOG_MISS    (1UL << 3U)
#define APP_HEALTH_FLAG_STACK_GUARD      (1UL << 4U)

typedef enum
{
    APP_HEALTH_GOOD = 0,
    APP_HEALTH_WARNING,
    APP_HEALTH_FAULT
} AppHealthState;

typedef struct
{
    uint32_t uptime_ms;
    uint32_t flags;
    uint32_t uart_rx_overflow_count;
    uint32_t uart_tx_drop_count;
    AppEventStats events;
    AppSchedulerStats scheduler;
    CommandControlSnapshot command;
    RuntimeMonitorSnapshot runtime;
    AppHealthState state;
} AppHealthSnapshot;

void AppHealth_GetSnapshot(AppHealthSnapshot *snapshot);
const char *AppHealth_StateText(AppHealthState state);

#endif /* APP_HEALTH_H */
