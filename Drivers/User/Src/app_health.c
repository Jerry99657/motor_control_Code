#include "app_health.h"

#include "app_runtime.h"
#include "comm_service.h"
#include <stddef.h>
#include <string.h>

void AppHealth_GetSnapshot(AppHealthSnapshot *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }

    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->uptime_ms = HAL_GetTick();
    CommService_GetStats(&snapshot->communication);
    CommandProtocol_GetStats(&snapshot->protocol);
    FOC_Link_GetTelemetry(&snapshot->foc_link);
    AppEvent_GetStats(&snapshot->events);
    AppRuntime_GetSchedulerStats(&snapshot->scheduler);
    CommandControl_GetSnapshot(&snapshot->command);
    RuntimeMonitor_GetSnapshot(&snapshot->runtime);

    if (snapshot->events.dropped_count != 0U)
    {
        snapshot->flags |= APP_HEALTH_FLAG_EVENT_DROP;
    }
    if (snapshot->communication.uart_rx_overflow_count != 0U)
    {
        snapshot->flags |= APP_HEALTH_FLAG_UART_RX_OVERFLOW;
    }
    if (snapshot->communication.uart_tx_drop_count != 0U)
    {
        snapshot->flags |= APP_HEALTH_FLAG_UART_TX_DROP;
    }
    if (snapshot->runtime.watchdog_missed_vote_count != 0U)
    {
        snapshot->flags |= APP_HEALTH_FLAG_WATCHDOG_MISS;
    }
    if (snapshot->runtime.stack_guard_ok == 0U)
    {
        snapshot->flags |= APP_HEALTH_FLAG_STACK_GUARD;
    }
    if (snapshot->communication.usb_rx_overflow_count != 0U)
    {
        snapshot->flags |= APP_HEALTH_FLAG_USB_RX_OVERFLOW;
    }
    if (snapshot->communication.usb_tx_drop_count != 0U)
    {
        snapshot->flags |= APP_HEALTH_FLAG_USB_TX_DROP;
    }
    if ((snapshot->communication.uart_rx_error_count != 0U) ||
        (snapshot->communication.uart_rx_rearm_error_count != 0U) ||
        (snapshot->communication.uart_tx_start_error_count != 0U))
    {
        snapshot->flags |= APP_HEALTH_FLAG_UART_ERROR;
    }
    if ((snapshot->foc_link.rx_overflow_count != 0U) ||
        (snapshot->foc_link.rx_error_count != 0U) ||
        (snapshot->foc_link.tx_drop_count != 0U))
    {
        snapshot->flags |= APP_HEALTH_FLAG_FOC_LINK_ERROR;
    }
    if ((snapshot->protocol.frame_error_count[0] != 0U) ||
        (snapshot->protocol.frame_error_count[1] != 0U))
    {
        snapshot->flags |= APP_HEALTH_FLAG_PROTOCOL_ERROR;
    }
    if (snapshot->protocol.response_drop_count != 0U)
    {
        snapshot->flags |= APP_HEALTH_FLAG_RESPONSE_DROP;
    }

    if ((snapshot->flags & APP_HEALTH_FLAG_STACK_GUARD) != 0U)
    {
        snapshot->state = APP_HEALTH_FAULT;
    }
    else if (snapshot->flags != 0U)
    {
        snapshot->state = APP_HEALTH_WARNING;
    }
    else
    {
        snapshot->state = APP_HEALTH_GOOD;
    }
}

const char *AppHealth_StateText(AppHealthState state)
{
    switch (state)
    {
        case APP_HEALTH_GOOD: return "GOOD";
        case APP_HEALTH_WARNING: return "WARNING";
        case APP_HEALTH_FAULT: return "FAULT";
        default: return "UNKNOWN";
    }
}
