#include "qspi_start_anim.h"

#include "lcd_spi_154.h"
#include "qspi_w25q64.h"

#define QSPI_START_ANIM_MAX_FRAME_BYTES (LCD_Width * LCD_Height * 2U)

static uint8_t s_anim_frame_buffer[QSPI_START_ANIM_MAX_FRAME_BYTES];

static void QSPI_StartAnim_Log(const char *msg)
{
  Boot_DebugStageLog(msg);
}

uint8_t *QSPI_StartAnim_GetFrameBuffer(uint32_t *buffer_size_bytes)
{
  if (buffer_size_bytes != NULL)
  {
    *buffer_size_bytes = QSPI_START_ANIM_MAX_FRAME_BYTES;
  }

  return s_anim_frame_buffer;
}

static uint16_t read_u16_le(const uint8_t *p)
{
  return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t read_u32_le(const uint8_t *p)
{
  return (uint32_t)p[0] |
         ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

int8_t QSPI_StartAnim_ReadInfo(QSPI_StartAnimInfo *info)
{
  uint8_t header[QSPI_START_ANIM_HEADER_SIZE] = {0};
  uint32_t magic;
  uint16_t version;
  uint16_t header_size;
  uint16_t width;
  uint16_t height;
  uint16_t frame_count;
  uint16_t frame_delay;
  uint32_t frame_size;
  uint32_t payload_size;
  uint32_t data_offset;
  uint32_t expected_payload;

  if (info == NULL)
  {
    return QSPI_START_ANIM_ERR_PARAM;
  }

  if (QSPI_W25Qxx_ReadBuffer(header, QSPI_START_ANIM_BASE_ADDR, QSPI_START_ANIM_HEADER_SIZE) != QSPI_W25QXX_OK)
  {
    return QSPI_START_ANIM_ERR_QSPI;
  }

  magic = read_u32_le(&header[0]);
  version = read_u16_le(&header[4]);
  header_size = read_u16_le(&header[6]);
  width = read_u16_le(&header[8]);
  height = read_u16_le(&header[10]);
  frame_count = read_u16_le(&header[12]);
  frame_delay = read_u16_le(&header[14]);
  frame_size = read_u32_le(&header[16]);
  payload_size = read_u32_le(&header[20]);
  data_offset = read_u32_le(&header[24]);

  if (magic != QSPI_START_ANIM_MAGIC ||
      version != QSPI_START_ANIM_VERSION ||
      header_size < QSPI_START_ANIM_HEADER_SIZE ||
      data_offset < QSPI_START_ANIM_HEADER_SIZE)
  {
    return QSPI_START_ANIM_ERR_HEADER;
  }

  if (width == 0U || height == 0U || width > LCD_Width || height > LCD_Height)
  {
    return QSPI_START_ANIM_ERR_HEADER;
  }

  if (frame_count == 0U || frame_delay == 0U)
  {
    return QSPI_START_ANIM_ERR_HEADER;
  }

  expected_payload = (uint32_t)width * (uint32_t)height * 2U * (uint32_t)frame_count;
  if (frame_size != ((uint32_t)width * (uint32_t)height * 2U) ||
      payload_size != expected_payload ||
      frame_size > QSPI_START_ANIM_MAX_FRAME_BYTES)
  {
    return QSPI_START_ANIM_ERR_HEADER;
  }

  if ((uint64_t)QSPI_START_ANIM_BASE_ADDR + (uint64_t)data_offset + (uint64_t)payload_size > (uint64_t)W25QXX_FLASH_SIZE_BYTES)
  {
    return QSPI_START_ANIM_ERR_HEADER;
  }

  info->width = width;
  info->height = height;
  info->frame_count = frame_count;
  info->frame_delay_ms = frame_delay;
  info->data_offset_bytes = data_offset;
  info->frame_size_bytes = frame_size;
  info->payload_size_bytes = payload_size;

  return QSPI_START_ANIM_OK;
}

int8_t QSPI_StartAnim_Play(void)
{
  QSPI_StartAnimInfo info = {0};
  int8_t status;
  uint16_t frame_index;
  uint16_t x;
  uint16_t y;
  uint32_t frame_addr;
  uint32_t anim_start_tick;
  uint32_t frame_start_tick;
  uint32_t elapsed_ms;
  uint32_t expected_elapsed_ms;
  uint32_t actual_elapsed_ms;
  uint32_t lag_ms;
  uint32_t drop_frames;
  uint16_t frames_left;
  uint32_t target_delay_ms;

  QSPI_StartAnim_Log("QSA: enter\r\n");

  status = QSPI_StartAnim_ReadInfo(&info);
  if (status != QSPI_START_ANIM_OK)
  {
    QSPI_StartAnim_Log("QSA: read info fail\r\n");
    return status;
  }

  QSPI_StartAnim_Log("QSA: play start\r\n");

  x = (uint16_t)((LCD_Width - info.width) / 2U);
  y = (uint16_t)((LCD_Height - info.height) / 2U);

  target_delay_ms = ((uint32_t)info.frame_delay_ms * (uint32_t)QSPI_START_ANIM_PLAYBACK_SPEED_DEN +
                    ((uint32_t)QSPI_START_ANIM_PLAYBACK_SPEED_NUM - 1U)) /
                   (uint32_t)QSPI_START_ANIM_PLAYBACK_SPEED_NUM;
  if (target_delay_ms == 0U)
  {
    target_delay_ms = 1U;
  }

  LCD_SetBackColor(LCD_BLACK);
  LCD_Clear();

  anim_start_tick = HAL_GetTick();

  for (frame_index = 0; frame_index < info.frame_count; )
  {
    frame_start_tick = HAL_GetTick();

    frame_addr = QSPI_START_ANIM_BASE_ADDR + info.data_offset_bytes + ((uint32_t)frame_index * info.frame_size_bytes);
    if (QSPI_W25Qxx_ReadBuffer(s_anim_frame_buffer, frame_addr, info.frame_size_bytes) != QSPI_W25QXX_OK)
    {
      QSPI_StartAnim_Log("QSA: frame read fail\r\n");
      return QSPI_START_ANIM_ERR_QSPI;
    }

    LCD_CopyBuffer(x, y, info.width, info.height, (const uint16_t *)s_anim_frame_buffer);

    frame_index++;

#if (QSPI_START_ANIM_DROP_LATE_FRAMES != 0U)
    if (frame_index < info.frame_count)
    {
      actual_elapsed_ms = HAL_GetTick() - anim_start_tick;
      expected_elapsed_ms = (uint32_t)frame_index * target_delay_ms;

      if (actual_elapsed_ms > (expected_elapsed_ms + target_delay_ms))
      {
        lag_ms = actual_elapsed_ms - expected_elapsed_ms;
        drop_frames = lag_ms / target_delay_ms;
        if (drop_frames > (uint32_t)QSPI_START_ANIM_MAX_DROP_PER_LOOP)
        {
          drop_frames = (uint32_t)QSPI_START_ANIM_MAX_DROP_PER_LOOP;
        }

        frames_left = (uint16_t)(info.frame_count - frame_index);
        if (drop_frames > (uint32_t)frames_left)
        {
          drop_frames = (uint32_t)frames_left;
        }

        frame_index = (uint16_t)(frame_index + (uint16_t)drop_frames);
      }
    }
#endif

    actual_elapsed_ms = HAL_GetTick() - anim_start_tick;
    expected_elapsed_ms = (uint32_t)frame_index * target_delay_ms;
    if (actual_elapsed_ms < expected_elapsed_ms)
    {
      HAL_Delay(expected_elapsed_ms - actual_elapsed_ms);
    }

    elapsed_ms = HAL_GetTick() - frame_start_tick;
    (void)elapsed_ms;
  }

  QSPI_StartAnim_Log("QSA: ok\r\n");

  return QSPI_START_ANIM_OK;
}
