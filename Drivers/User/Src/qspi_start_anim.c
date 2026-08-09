#include "qspi_start_anim.h"

#include "lcd_spi_154.h"
#include "media_memory.h"
#include "mjpeg_scheduler.h"
#include "qspi_w25q64.h"

#define QSPI_START_ANIM_MAX_FRAME_BYTES (LCD_Width * LCD_Height * 2U)

static void QSPI_StartAnim_Log(const char *msg)
{
  Boot_DebugStageLog(msg);
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

static void qspi_wait_frame_pace(uint32_t wait_ms)
{
  uint32_t start_ms;

  if (wait_ms == 0U)
  {
    return;
  }

  start_ms = HAL_GetTick();

  for (;;)
  {
    if (MJPEG_Scheduler_ConsumeFrameTick() != 0U)
    {
      return;
    }

    if ((HAL_GetTick() - start_ms) >= wait_ms)
    {
      return;
    }

    LCD_TransferService();
    HAL_Delay(1U);
  }
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

  if ((uint64_t)QSPI_START_ANIM_BASE_ADDR + (uint64_t)data_offset +
      (uint64_t)payload_size > (uint64_t)QSPI_PARTITION_START_ANIM_END)
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
  uint32_t target_delay_ms;
  uint8_t use_async_lcd = 1U;
  uint8_t lcd_transfer_pending = 0U;
  uint8_t *frame_buffer;
  uint32_t media_capacity = 0U;

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

  if (MJPEG_Scheduler_SetFrameIntervalMs(target_delay_ms) != HAL_OK)
  {
    return QSPI_START_ANIM_ERR_QSPI;
  }

  frame_buffer = MediaMemory_Acquire(
    MEDIA_MEMORY_OWNER_QSPI_ANIM,
    info.frame_size_bytes,
    &media_capacity
  );
  if ((frame_buffer == NULL) || (media_capacity < info.frame_size_bytes))
  {
    return QSPI_START_ANIM_ERR_BUSY;
  }

  LCD_SetBackColor(LCD_BLACK);
  LCD_Clear();

  for (frame_index = 0; frame_index < info.frame_count; )
  {
    frame_addr = QSPI_START_ANIM_BASE_ADDR + info.data_offset_bytes + ((uint32_t)frame_index * info.frame_size_bytes);
    if (QSPI_W25Qxx_ReadBuffer(frame_buffer, frame_addr, info.frame_size_bytes) != QSPI_W25QXX_OK)
    {
      QSPI_StartAnim_Log("QSA: frame read fail\r\n");
      status = QSPI_START_ANIM_ERR_QSPI;
      goto cleanup;
    }

    if (use_async_lcd != 0U)
    {
      if (LCD_CopyBufferAsync(x, y, info.width, info.height, (const uint16_t *)frame_buffer) == HAL_OK)
      {
        lcd_transfer_pending = 1U;
      }
      else
      {
        LCD_ResetTransferState();
        use_async_lcd = 0U;
        LCD_CopyBuffer(x, y, info.width, info.height, (const uint16_t *)frame_buffer);
      }
    }
    else
    {
      LCD_CopyBuffer(x, y, info.width, info.height, (const uint16_t *)frame_buffer);
    }

    frame_index++;

    if (frame_index < info.frame_count)
    {
      qspi_wait_frame_pace(target_delay_ms + 10U);
    }

    if (lcd_transfer_pending != 0U)
    {
      if (LCD_WaitTransmitDone(1000U) != HAL_OK)
      {
        LCD_ResetTransferState();
        use_async_lcd = 0U;
        LCD_CopyBuffer(x, y, info.width, info.height, (const uint16_t *)frame_buffer);
      }
      lcd_transfer_pending = 0U;
    }
  }

  status = QSPI_START_ANIM_OK;

cleanup:
  MediaMemory_Release(MEDIA_MEMORY_OWNER_QSPI_ANIM);
  if (status != QSPI_START_ANIM_OK)
  {
    return status;
  }

  QSPI_StartAnim_Log("QSA: ok\r\n");

  return QSPI_START_ANIM_OK;
}
