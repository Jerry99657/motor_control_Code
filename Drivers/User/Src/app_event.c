#include "app_event.h"

#include "main.h"
#include <stddef.h>
#include <string.h>

static AppEvent s_queue[APP_EVENT_QUEUE_DEPTH];
static volatile uint8_t s_head;
static volatile uint8_t s_tail;
static volatile uint8_t s_count;
static volatile uint8_t s_high_watermark;
static volatile uint32_t s_posted_count;
static volatile uint32_t s_handled_count;
static volatile uint32_t s_dropped_count;

static uint32_t app_event_lock(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return primask;
}

static void app_event_unlock(uint32_t primask)
{
    __DMB();
    if (primask == 0U)
    {
        __enable_irq();
    }
}

void AppEvent_Init(void)
{
    uint32_t primask = app_event_lock();

    memset(s_queue, 0, sizeof(s_queue));
    s_head = 0U;
    s_tail = 0U;
    s_count = 0U;
    s_high_watermark = 0U;
    s_posted_count = 0U;
    s_handled_count = 0U;
    s_dropped_count = 0U;
    app_event_unlock(primask);
}

uint8_t AppEvent_PostCommRx(uint8_t channel, const uint8_t *data,
                            uint16_t length)
{
    AppEvent *event;
    uint32_t primask;

    if ((data == NULL) || (length == 0U) ||
        (length > APP_EVENT_PAYLOAD_CAPACITY))
    {
        return 0U;
    }

    primask = app_event_lock();
    if (s_count >= APP_EVENT_QUEUE_DEPTH)
    {
        s_dropped_count++;
        app_event_unlock(primask);
        return 0U;
    }

    event = &s_queue[s_head];
    event->type = APP_EVENT_COMM_RX;
    event->channel = channel;
    event->length = length;
    memcpy(event->payload, data, length);
    s_head = (uint8_t)((s_head + 1U) % APP_EVENT_QUEUE_DEPTH);
    s_count++;
    if (s_count > s_high_watermark)
    {
        s_high_watermark = s_count;
    }
    s_posted_count++;
    app_event_unlock(primask);
    return 1U;
}

uint8_t AppEvent_Get(AppEvent *event)
{
    uint32_t primask;

    if (event == NULL)
    {
        return 0U;
    }

    primask = app_event_lock();
    if (s_count == 0U)
    {
        app_event_unlock(primask);
        return 0U;
    }

    *event = s_queue[s_tail];
    s_tail = (uint8_t)((s_tail + 1U) % APP_EVENT_QUEUE_DEPTH);
    s_count--;
    app_event_unlock(primask);
    return 1U;
}

void AppEvent_MarkHandled(void)
{
    uint32_t primask = app_event_lock();
    s_handled_count++;
    app_event_unlock(primask);
}

void AppEvent_GetStats(AppEventStats *stats)
{
    uint32_t primask;

    if (stats == NULL)
    {
        return;
    }

    primask = app_event_lock();
    stats->posted_count = s_posted_count;
    stats->handled_count = s_handled_count;
    stats->dropped_count = s_dropped_count;
    stats->pending_count = s_count;
    stats->high_watermark = s_high_watermark;
    app_event_unlock(primask);
}
