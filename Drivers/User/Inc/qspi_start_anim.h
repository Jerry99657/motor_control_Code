#ifndef QSPI_START_ANIM_H
#define QSPI_START_ANIM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define QSPI_START_ANIM_OK            0
#define QSPI_START_ANIM_ERR_PARAM    -1
#define QSPI_START_ANIM_ERR_HEADER   -2
#define QSPI_START_ANIM_ERR_QSPI     -3

#define QSPI_START_ANIM_BASE_ADDR    0x000000U
#define QSPI_START_ANIM_MAGIC        0x314E4151U
#define QSPI_START_ANIM_VERSION      1U
#define QSPI_START_ANIM_HEADER_SIZE  32U

/* Playback speed = NUM / DEN. Default is 2x speed. */
#define QSPI_START_ANIM_PLAYBACK_SPEED_NUM  8U
#define QSPI_START_ANIM_PLAYBACK_SPEED_DEN  1U

/* In extreme mode, allow skipping late frames to keep perceived smoothness. */
#define QSPI_START_ANIM_DROP_LATE_FRAMES    1U
#define QSPI_START_ANIM_MAX_DROP_PER_LOOP   2U

typedef struct
{
  uint16_t width;
  uint16_t height;
  uint16_t frame_count;
  uint16_t frame_delay_ms;
  uint32_t data_offset_bytes;
  uint32_t frame_size_bytes;
  uint32_t payload_size_bytes;
} QSPI_StartAnimInfo;

uint8_t *QSPI_StartAnim_GetFrameBuffer(uint32_t *buffer_size_bytes);
int8_t QSPI_StartAnim_ReadInfo(QSPI_StartAnimInfo *info);
int8_t QSPI_StartAnim_Play(void);

#ifdef __cplusplus
}
#endif

#endif
