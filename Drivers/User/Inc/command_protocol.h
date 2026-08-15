#ifndef COMMAND_PROTOCOL_H
#define COMMAND_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define COMMAND_PROTOCOL_CHANNEL_UART5 0U
#define COMMAND_PROTOCOL_CHANNEL_USB   1U
#define COMMAND_PROTOCOL_CHANNEL_COUNT 2U

typedef struct
{
    uint32_t rx_byte_count[COMMAND_PROTOCOL_CHANNEL_COUNT];
    uint32_t valid_frame_count[COMMAND_PROTOCOL_CHANNEL_COUNT];
    uint32_t frame_error_count[COMMAND_PROTOCOL_CHANNEL_COUNT];
    uint32_t resync_discard_count[COMMAND_PROTOCOL_CHANNEL_COUNT];
    uint32_t text_line_count[COMMAND_PROTOCOL_CHANNEL_COUNT];
    uint32_t accepted_command_count;
    uint32_t rejected_command_count;
    uint32_t response_drop_count;
} CommandProtocolStats;

void CommandProtocol_Init(void);
void CommandProtocol_ResetInput(void);
void CommandProtocol_Receive(uint8_t channel, const uint8_t *data,
                             uint16_t length);
void CommandProtocol_GetStats(CommandProtocolStats *stats);

#ifdef __cplusplus
}
#endif

#endif /* COMMAND_PROTOCOL_H */
