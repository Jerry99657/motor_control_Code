#ifndef CAMERA_STREAM_H
#define CAMERA_STREAM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define CAMERA_STREAM_DEVICE_ID          0x0FU
#define CAMERA_STREAM_COMMAND_ENABLE     0x02U
#define CAMERA_STREAM_COMMAND_FRAME_ACK  0x03U

typedef struct
{
    uint32_t frames_started;
    uint32_t frames_completed;
    uint32_t frames_dropped;
    uint32_t packets_sent;
    uint32_t ack_timeout_count;
    uint32_t camera_error_count;
    uint32_t uart_busy_count;
    uint32_t last_frame_size;
    uint16_t last_frame_sequence;
    uint8_t requested;
    uint8_t camera_owned;
} CameraStreamStats;

void CameraStream_Init(void);
void CameraStream_Process(void);

/* ENABLE is refreshed periodically by ESP32 and expires automatically if the
 * controller disappears. Streaming itself is allowed only while the LCD
 * Command Control page owns motor commands. */
void CameraStream_SetRemoteEnabled(uint8_t enabled);
void CameraStream_AcknowledgeFrame(uint16_t sequence);
void CameraStream_GetStats(CameraStreamStats *stats);

/* A 1 ms application-loop cadence removes idle gaps between UART5 DMA
 * chunks while streaming. Other pages can retain the lower-power 5 ms pace. */
uint8_t CameraStream_NeedsFastService(void);

#ifdef __cplusplus
}
#endif

#endif /* CAMERA_STREAM_H */
