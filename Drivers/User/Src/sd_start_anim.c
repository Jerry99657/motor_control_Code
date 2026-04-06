#include "sd_start_anim.h"

#include "fatfs.h"
#include "lcd_spi_154.h"
#include "main.h"
#include "mjpeg_scheduler.h"
#include "qspi_start_anim.h"
#include <stdio.h>
#include <string.h>

#define SD_START_ANIM_MAX_FRAME_BYTES (LCD_Width * LCD_Height * 2U)
#define SD_START_ANIM_FRAME_BUFFER_COUNT 1U
#define SD_START_ANIM_READ_CHUNK_BYTES  (32U * 1024U)
#define SD_START_ANIM_STAGE_BYTES       4096U
#define SD_START_ANIM_SEEK_STEP_FRAMES  10U
#define SD_START_ANIM_SEEK_REPEAT_START 500U
#define SD_START_ANIM_SEEK_REPEAT_STEP  300U
/* SD playback can run faster than the source encoding without changing the
 * generated frame rate. Keep this conservative to avoid starving the LCD/SD path. */
#define SD_START_ANIM_PLAYBACK_SPEED_NUM  8U
#define SD_START_ANIM_PLAYBACK_SPEED_DEN  1U

typedef enum
{
  SD_PLAYBACK_ACTION_NONE = 0,
  SD_PLAYBACK_ACTION_STOP,
  SD_PLAYBACK_ACTION_SEEK_BACK,
  SD_PLAYBACK_ACTION_SEEK_FWD
} sd_playback_action_t;

static uint8_t s_frame_buffers[SD_START_ANIM_FRAME_BUFFER_COUNT][SD_START_ANIM_MAX_FRAME_BYTES] __attribute__((section(".ram_d2"), aligned(32)));
static uint8_t s_sd_read_stage[SD_START_ANIM_STAGE_BYTES] __attribute__((aligned(32)));
static FRESULT s_sd_last_read_fr = FR_OK;
static UINT s_sd_last_read_len = 0U;
static UINT s_sd_last_read_req = 0U;
static sd_playback_action_t s_seek_active_action = SD_PLAYBACK_ACTION_NONE;
static uint32_t s_seek_press_tick = 0U;
static uint32_t s_seek_last_repeat_tick = 0U;

static void SD_StartAnim_Log(const char *msg)
{
  Boot_DebugStageLog(msg);
}

static void SD_StartAnim_LogReadFail(const char *tag)
{
  char line[96];
  int len;

  len = snprintf(
    line,
    sizeof(line),
    "SDA: %s fail fr=%d req=%u got=%u\\r\\n",
    tag,
    (int)s_sd_last_read_fr,
    (unsigned int)s_sd_last_read_req,
    (unsigned int)s_sd_last_read_len
  );

  if (len > 0)
  {
    SD_StartAnim_Log(line);
  }
}

static uint8_t sd_playback_stop_requested(void)
{
  return (HAL_GPIO_ReadPin(Key2_GPIO_Port, Key2_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
}

static void sd_playback_reset_seek_state(void)
{
  s_seek_active_action = SD_PLAYBACK_ACTION_NONE;
  s_seek_press_tick = 0U;
  s_seek_last_repeat_tick = 0U;
}

static sd_playback_action_t sd_playback_get_action(void)
{
  uint8_t left_pressed;
  uint8_t right_pressed;
  sd_playback_action_t active_action;
  uint32_t now;

  if (sd_playback_stop_requested() != 0U)
  {
    return SD_PLAYBACK_ACTION_STOP;
  }

  left_pressed = (HAL_GPIO_ReadPin(Key_Left_GPIO_Port, Key_Left_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
  right_pressed = (HAL_GPIO_ReadPin(Key_Right_GPIO_Port, Key_Right_Pin) == GPIO_PIN_RESET) ? 1U : 0U;

  if ((left_pressed == 0U) && (right_pressed == 0U))
  {
    sd_playback_reset_seek_state();
    return SD_PLAYBACK_ACTION_NONE;
  }

  if ((left_pressed != 0U) && (right_pressed == 0U))
  {
    active_action = SD_PLAYBACK_ACTION_SEEK_BACK;
  }
  else if ((right_pressed != 0U) && (left_pressed == 0U))
  {
    active_action = SD_PLAYBACK_ACTION_SEEK_FWD;
  }
  else
  {
    return SD_PLAYBACK_ACTION_NONE;
  }

  now = HAL_GetTick();
  if (s_seek_active_action != active_action)
  {
    s_seek_active_action = active_action;
    s_seek_press_tick = now;
    s_seek_last_repeat_tick = now;
    return active_action;
  }

  if ((now - s_seek_press_tick) < SD_START_ANIM_SEEK_REPEAT_START)
  {
    return SD_PLAYBACK_ACTION_NONE;
  }

  if ((now - s_seek_last_repeat_tick) >= SD_START_ANIM_SEEK_REPEAT_STEP)
  {
    s_seek_last_repeat_tick = now;
    return active_action;
  }

  return SD_PLAYBACK_ACTION_NONE;
}

static int8_t sd_seek_to_frame(FIL *file, const QSPI_StartAnimInfo *info, uint16_t frame_index)
{
  FRESULT fr;
  FSIZE_t offset;

  if ((file == NULL) || (info == NULL))
  {
    return SD_START_ANIM_ERR_PARAM;
  }

  offset = (FSIZE_t)info->data_offset_bytes + ((FSIZE_t)frame_index * (FSIZE_t)info->frame_size_bytes);
  fr = f_lseek(file, offset);
  if (fr != FR_OK)
  {
    return SD_START_ANIM_ERR_IO;
  }

  return SD_START_ANIM_OK;
}

static int8_t sd_wait_for_playback_action(uint32_t wait_ms, sd_playback_action_t *action)
{
  sd_playback_action_t current_action;
  uint32_t start_ms;

  if (action == NULL)
  {
    return SD_START_ANIM_ERR_PARAM;
  }

  start_ms = HAL_GetTick();

  for (;;)
  {
    current_action = sd_playback_get_action();
    if (current_action == SD_PLAYBACK_ACTION_STOP)
    {
      *action = current_action;
      return SD_START_ANIM_OK;
    }

    if ((current_action == SD_PLAYBACK_ACTION_SEEK_BACK) || (current_action == SD_PLAYBACK_ACTION_SEEK_FWD))
    {
      *action = current_action;
      return SD_START_ANIM_OK;
    }

    if (MJPEG_Scheduler_ConsumeFrameTick() != 0U)
    {
      *action = SD_PLAYBACK_ACTION_NONE;
      return SD_START_ANIM_OK;
    }

    if ((HAL_GetTick() - start_ms) >= wait_ms)
    {
      break;
    }

    HAL_Delay(1U);
  }

  *action = SD_PLAYBACK_ACTION_NONE;
  return SD_START_ANIM_OK;
}

static int8_t sd_read_exact(FIL *file, uint8_t *buffer, uint32_t bytes_to_read)
{
  FRESULT fr;
  UINT read_len;
  uint32_t remain;

  if ((file == NULL) || (buffer == NULL) || (bytes_to_read == 0U))
  {
    return SD_START_ANIM_ERR_PARAM;
  }

  remain = bytes_to_read;
  while (remain > 0U)
  {
    uint32_t outer_chunk = (remain > SD_START_ANIM_READ_CHUNK_BYTES) ? SD_START_ANIM_READ_CHUNK_BYTES : remain;
    uint32_t outer_remain = outer_chunk;

    while (outer_remain > 0U)
    {
      UINT chunk = (UINT)((outer_remain > SD_START_ANIM_STAGE_BYTES) ? SD_START_ANIM_STAGE_BYTES : outer_remain);

      s_sd_last_read_req = chunk;
      s_sd_last_read_len = 0U;
      s_sd_last_read_fr = FR_OK;

      fr = f_read(file, s_sd_read_stage, chunk, &read_len);
      s_sd_last_read_fr = fr;
      s_sd_last_read_len = read_len;

      if (fr != FR_OK)
      {
        return SD_START_ANIM_ERR_IO;
      }

      if (read_len != chunk)
      {
        return SD_START_ANIM_ERR_IO;
      }

      memcpy(buffer, s_sd_read_stage, read_len);
      buffer += read_len;
      outer_remain -= (uint32_t)read_len;
      remain -= (uint32_t)read_len;
    }
  }

  return SD_START_ANIM_OK;
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

static int8_t sd_parse_anim_header(const uint8_t *header, QSPI_StartAnimInfo *info, FSIZE_t file_size)
{
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

  if ((header == NULL) || (info == NULL))
  {
    return SD_START_ANIM_ERR_PARAM;
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
    return SD_START_ANIM_ERR_HEADER;
  }

  if (width == 0U || height == 0U || width > LCD_Width || height > LCD_Height)
  {
    return SD_START_ANIM_ERR_HEADER;
  }

  if (frame_count == 0U || frame_delay == 0U)
  {
    return SD_START_ANIM_ERR_HEADER;
  }

  expected_payload = (uint32_t)width * (uint32_t)height * 2U * (uint32_t)frame_count;
  if (frame_size != ((uint32_t)width * (uint32_t)height * 2U) ||
      payload_size != expected_payload ||
      frame_size > SD_START_ANIM_MAX_FRAME_BYTES)
  {
    return SD_START_ANIM_ERR_HEADER;
  }

  if (((uint64_t)data_offset + (uint64_t)payload_size) > (uint64_t)file_size)
  {
    return SD_START_ANIM_ERR_HEADER;
  }

  info->width = width;
  info->height = height;
  info->frame_count = frame_count;
  info->frame_delay_ms = frame_delay;
  info->data_offset_bytes = data_offset;
  info->frame_size_bytes = frame_size;
  info->payload_size_bytes = payload_size;

  return SD_START_ANIM_OK;
}

int8_t SD_StartAnim_Play(void)
{
  return SD_StartAnim_PlayFile(SD_START_ANIM_FILE_NAME);
}

int8_t SD_StartAnim_PlayFile(const char *file_path)
{
  uint8_t header[QSPI_START_ANIM_HEADER_SIZE] = {0};
  QSPI_StartAnimInfo info = {0};
  FIL file;
  FRESULT fr;
  UINT read_len;
  int8_t status;
  int8_t seek_status;
  uint16_t x;
  uint16_t y;
  uint16_t frame_index;
  sd_playback_action_t playback_action;
  uint8_t *current_frame_buffer;
  uint8_t use_async_lcd = 1U;
  uint32_t playback_delay_ms;

  if ((file_path == NULL) || (file_path[0] == '\0'))
  {
    return SD_START_ANIM_ERR_PARAM;
  }

  SD_StartAnim_Log("SDA: enter\r\n");

  fr = f_mount(&SDFatFS, (TCHAR const *)SDPath, 1U);
  if (fr != FR_OK)
  {
    SD_StartAnim_Log("SDA: mount fail\r\n");
    return SD_START_ANIM_ERR_MOUNT;
  }

  fr = f_open(&file, file_path, FA_READ);
  if (fr != FR_OK)
  {
    SD_StartAnim_Log("SDA: open fail\r\n");
    (void)f_mount(NULL, (TCHAR const *)SDPath, 1U);
    return SD_START_ANIM_ERR_FILE;
  }

  fr = f_read(&file, header, (UINT)sizeof(header), &read_len);
  if ((fr != FR_OK) || (read_len != (UINT)sizeof(header)))
  {
    SD_StartAnim_Log("SDA: hdr read fail\r\n");
    (void)f_close(&file);
    (void)f_mount(NULL, (TCHAR const *)SDPath, 1U);
    return SD_START_ANIM_ERR_IO;
  }

  status = sd_parse_anim_header(header, &info, f_size(&file));
  if (status != SD_START_ANIM_OK)
  {
    SD_StartAnim_Log("SDA: hdr parse fail\r\n");
    (void)f_close(&file);
    (void)f_mount(NULL, (TCHAR const *)SDPath, 1U);
    return status;
  }

  if (info.frame_size_bytes > SD_START_ANIM_MAX_FRAME_BYTES)
  {
    (void)f_close(&file);
    (void)f_mount(NULL, (TCHAR const *)SDPath, 1U);
    return SD_START_ANIM_ERR_HEADER;
  }

  fr = f_lseek(&file, (FSIZE_t)info.data_offset_bytes);
  if (fr != FR_OK)
  {
    SD_StartAnim_Log("SDA: seek data fail\r\n");
    (void)f_close(&file);
    (void)f_mount(NULL, (TCHAR const *)SDPath, 1U);
    return SD_START_ANIM_ERR_IO;
  }

  /* Keep playback aligned to the source frame rate. */
  if (info.frame_delay_ms == 0U)
  {
    info.frame_delay_ms = 1U;
  }

  playback_delay_ms = ((uint32_t)info.frame_delay_ms * (uint32_t)SD_START_ANIM_PLAYBACK_SPEED_DEN +
                       ((uint32_t)SD_START_ANIM_PLAYBACK_SPEED_NUM - 1U)) /
                      (uint32_t)SD_START_ANIM_PLAYBACK_SPEED_NUM;
  if (playback_delay_ms == 0U)
  {
    playback_delay_ms = 1U;
  }

  if (MJPEG_Scheduler_SetFrameIntervalMs(playback_delay_ms) != HAL_OK)
  {
    (void)f_close(&file);
    (void)f_mount(NULL, (TCHAR const *)SDPath, 1U);
    return SD_START_ANIM_ERR_IO;
  }

  x = (uint16_t)((LCD_Width - info.width) / 2U);
  y = (uint16_t)((LCD_Height - info.height) / 2U);

  LCD_SetBackColor(LCD_BLACK);
  LCD_Clear();

  current_frame_buffer = s_frame_buffers[0];
  status = SD_START_ANIM_OK;
  frame_index = 0U;
  sd_playback_reset_seek_state();

  while (frame_index < info.frame_count)
  {
    if (sd_playback_stop_requested() != 0U)
    {
      status = SD_START_ANIM_ERR_STOPPED;
      break;
    }

    seek_status = sd_seek_to_frame(&file, &info, frame_index);
    if (seek_status != SD_START_ANIM_OK)
    {
      status = seek_status;
      break;
    }

    status = sd_read_exact(&file, current_frame_buffer, info.frame_size_bytes);
    if (status != SD_START_ANIM_OK)
    {
      SD_StartAnim_LogReadFail((frame_index == 0U) ? "frame0 read" : "frame read");
      break;
    }

    if (use_async_lcd != 0U)
    {
      if (LCD_CopyBufferAsync(x, y, info.width, info.height, (const uint16_t *)current_frame_buffer) != HAL_OK)
      {
        SD_StartAnim_Log("SDA: lcd async start fail->sync\r\n");
        LCD_ResetTransferState();
        use_async_lcd = 0U;
        LCD_CopyBuffer(x, y, info.width, info.height, (const uint16_t *)current_frame_buffer);
      }
      else if (LCD_WaitTransmitDone(1000U) != HAL_OK)
      {
        SD_StartAnim_Log("SDA: lcd async wait fail->sync\r\n");
        LCD_ResetTransferState();
        use_async_lcd = 0U;
        LCD_CopyBuffer(x, y, info.width, info.height, (const uint16_t *)current_frame_buffer);
      }
    }
    else
    {
      LCD_CopyBuffer(x, y, info.width, info.height, (const uint16_t *)current_frame_buffer);
    }

    if (sd_playback_stop_requested() != 0U)
    {
      status = SD_START_ANIM_ERR_STOPPED;
      break;
    }

    playback_action = SD_PLAYBACK_ACTION_NONE;
    if (sd_wait_for_playback_action(playback_delay_ms, &playback_action) != SD_START_ANIM_OK)
    {
      status = SD_START_ANIM_ERR_IO;
      break;
    }

    if (playback_action == SD_PLAYBACK_ACTION_STOP)
    {
      status = SD_START_ANIM_ERR_STOPPED;
      break;
    }

    if (playback_action == SD_PLAYBACK_ACTION_SEEK_BACK)
    {
      if (frame_index > SD_START_ANIM_SEEK_STEP_FRAMES)
      {
        frame_index = (uint16_t)(frame_index - SD_START_ANIM_SEEK_STEP_FRAMES);
      }
      else
      {
        frame_index = 0U;
      }

      continue;
    }

    if (playback_action == SD_PLAYBACK_ACTION_SEEK_FWD)
    {
      if ((uint32_t)frame_index + SD_START_ANIM_SEEK_STEP_FRAMES < (uint32_t)info.frame_count)
      {
        frame_index = (uint16_t)(frame_index + SD_START_ANIM_SEEK_STEP_FRAMES);
      }
      else
      {
        frame_index = (uint16_t)(info.frame_count - 1U);
      }

      continue;
    }

    frame_index++;
  }

  if (status == SD_START_ANIM_OK)
  {
    SD_StartAnim_Log("SDA: ok\r\n");
  }
  else if (status == SD_START_ANIM_ERR_STOPPED)
  {
    SD_StartAnim_Log("SDA: stop by key2\r\n");
  }

  (void)f_close(&file);
  (void)f_mount(NULL, (TCHAR const *)SDPath, 1U);
  return status;
}
