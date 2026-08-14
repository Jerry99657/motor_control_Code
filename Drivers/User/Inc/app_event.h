#ifndef APP_EVENT_H
#define APP_EVENT_H

#include <stdint.h>

#define APP_EVENT_QUEUE_DEPTH       8U
#define APP_EVENT_PAYLOAD_CAPACITY 64U

typedef enum
{
    APP_EVENT_NONE = 0,
    APP_EVENT_COMM_RX
} AppEventType;

typedef struct
{
    AppEventType type;
    uint8_t channel;
    uint16_t length;
    uint8_t payload[APP_EVENT_PAYLOAD_CAPACITY];
} AppEvent;

typedef struct
{
    uint32_t posted_count;
    uint32_t handled_count;
    uint32_t dropped_count;
    uint8_t pending_count;
    uint8_t high_watermark;
} AppEventStats;

void AppEvent_Init(void);
uint8_t AppEvent_PostCommRx(uint8_t channel, const uint8_t *data,
                            uint16_t length);
uint8_t AppEvent_Get(AppEvent *event);
void AppEvent_MarkHandled(void);
void AppEvent_GetStats(AppEventStats *stats);

#endif /* APP_EVENT_H */
