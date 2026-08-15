#ifndef APP_HEALTH_H
#define APP_HEALTH_H

#include "app_event.h"
#include "app_scheduler.h"
#include "command_protocol.h"
#include "comm_service.h"
#include "command_control.h"
#include "foc_link.h"
#include "runtime_monitor.h"
#include <stdint.h>

#define APP_HEALTH_FLAG_EVENT_DROP       (1UL << 0U)
#define APP_HEALTH_FLAG_UART_RX_OVERFLOW (1UL << 1U)
#define APP_HEALTH_FLAG_UART_TX_DROP     (1UL << 2U)
#define APP_HEALTH_FLAG_WATCHDOG_MISS    (1UL << 3U)
#define APP_HEALTH_FLAG_STACK_GUARD      (1UL << 4U)
#define APP_HEALTH_FLAG_USB_RX_OVERFLOW  (1UL << 5U)
#define APP_HEALTH_FLAG_USB_TX_DROP      (1UL << 6U)
#define APP_HEALTH_FLAG_UART_ERROR       (1UL << 7U)
#define APP_HEALTH_FLAG_FOC_LINK_ERROR   (1UL << 8U)
#define APP_HEALTH_FLAG_PROTOCOL_ERROR   (1UL << 9U)
#define APP_HEALTH_FLAG_RESPONSE_DROP    (1UL << 10U)

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
    CommServiceStats communication;
    CommandProtocolStats protocol;
    FOC_LinkTelemetry foc_link;
    AppEventStats events;
    AppSchedulerStats scheduler;
    CommandControlSnapshot command;
    RuntimeMonitorSnapshot runtime;
    AppHealthState state;
} AppHealthSnapshot;

void AppHealth_GetSnapshot(AppHealthSnapshot *snapshot);
const char *AppHealth_StateText(AppHealthState state);

#endif /* APP_HEALTH_H */
