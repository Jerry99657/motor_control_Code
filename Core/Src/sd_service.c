#include "sd_service.h"

#include "fatfs.h"
#include "main.h"
#include <string.h>

#define SD_BENCH_MAX_CHUNK 4096U

#if defined(__GNUC__)
#define SD_BENCH_BUF_ALIGN __attribute__((aligned(32)))
#else
#define SD_BENCH_BUF_ALIGN
#endif

static uint8_t sd_bench_write_buf[SD_BENCH_MAX_CHUNK] SD_BENCH_BUF_ALIGN;
static uint8_t sd_bench_read_buf[SD_BENCH_MAX_CHUNK] SD_BENCH_BUF_ALIGN;

static FRESULT sd_mount(void)
{
  return f_mount(&SDFatFS, (TCHAR const *)SDPath, 1U);
}

static void sd_unmount(void)
{
  (void)f_mount(NULL, (TCHAR const *)SDPath, 1U);
}

FRESULT SD_Service_WriteTextFile(const char *file_name, const char *text)
{
  FIL file;
  FRESULT fr;
  UINT written = 0U;
  UINT len;

  if ((file_name == NULL) || (text == NULL))
  {
    return FR_INVALID_PARAMETER;
  }

  fr = sd_mount();
  if (fr != FR_OK)
  {
    return fr;
  }

  fr = f_open(&file, file_name, FA_CREATE_ALWAYS | FA_WRITE);
  if (fr == FR_OK)
  {
    len = (UINT)strlen(text);
    fr = f_write(&file, text, len, &written);
    if ((fr == FR_OK) && (written != len))
    {
      fr = FR_DISK_ERR;
    }
    (void)f_close(&file);
  }

  sd_unmount();
  return fr;
}

FRESULT SD_Service_ReadTextFile(const char *file_name, char *out_buf, uint32_t out_buf_size, uint32_t *out_len)
{
  FIL file;
  FRESULT fr;
  UINT read_len = 0U;

  if ((file_name == NULL) || (out_buf == NULL) || (out_buf_size == 0U))
  {
    return FR_INVALID_PARAMETER;
  }

  fr = sd_mount();
  if (fr != FR_OK)
  {
    return fr;
  }

  fr = f_open(&file, file_name, FA_READ);
  if (fr == FR_OK)
  {
    fr = f_read(&file, out_buf, (UINT)(out_buf_size - 1U), &read_len);
    if (fr == FR_OK)
    {
      out_buf[read_len] = '\0';
      if (out_len != NULL)
      {
        *out_len = (uint32_t)read_len;
      }
    }
    (void)f_close(&file);
  }

  sd_unmount();
  return fr;
}

FRESULT SD_Service_DeleteFile(const char *file_name)
{
  FRESULT fr;

  if (file_name == NULL)
  {
    return FR_INVALID_PARAMETER;
  }

  fr = sd_mount();
  if (fr != FR_OK)
  {
    return fr;
  }

  fr = f_unlink(file_name);
  sd_unmount();
  return fr;
}

FRESULT SD_Service_BenchSequentialWrite(const char *file_name, uint32_t total_bytes, uint32_t chunk_bytes, sd_bench_result_t *result)
{
  FIL file;
  FRESULT fr;
  UINT written;
  uint32_t total_written = 0U;
  uint32_t start_tick;
  uint32_t i;

  if ((file_name == NULL) || (result == NULL) || (chunk_bytes == 0U) || (chunk_bytes > SD_BENCH_MAX_CHUNK) || (total_bytes == 0U))
  {
    return FR_INVALID_PARAMETER;
  }

  for (i = 0U; i < chunk_bytes; ++i)
  {
    sd_bench_write_buf[i] = (uint8_t)(i & 0xFFU);
  }

  fr = sd_mount();
  if (fr != FR_OK)
  {
    result->fresult = fr;
    return fr;
  }

  fr = f_open(&file, file_name, FA_CREATE_ALWAYS | FA_WRITE);
  if (fr != FR_OK)
  {
    sd_unmount();
    result->fresult = fr;
    return fr;
  }

  start_tick = HAL_GetTick();
  while (total_written < total_bytes)
  {
    uint32_t remaining = total_bytes - total_written;
    UINT write_now = (remaining > chunk_bytes) ? (UINT)chunk_bytes : (UINT)remaining;

    fr = f_write(&file, sd_bench_write_buf, write_now, &written);
    if ((fr != FR_OK) || (written != write_now))
    {
      if (fr == FR_OK)
      {
        fr = FR_DISK_ERR;
      }
      break;
    }
    total_written += write_now;
  }

  if (fr == FR_OK)
  {
    fr = f_sync(&file);
  }

  result->elapsed_ms = HAL_GetTick() - start_tick;
  result->bytes = total_written;
  result->bytes_per_sec = (result->elapsed_ms > 0U) ? ((total_written * 1000U) / result->elapsed_ms) : 0U;
  result->fresult = fr;

  (void)f_close(&file);
  sd_unmount();
  return fr;
}

FRESULT SD_Service_BenchRandomRead(const char *file_name, uint32_t read_count, uint32_t chunk_bytes, sd_bench_result_t *result)
{
  FIL file;
  FRESULT fr;
  UINT read_len;
  FSIZE_t file_size;
  uint32_t bytes = 0U;
  uint32_t start_tick;
  uint32_t i;

  if ((file_name == NULL) || (result == NULL) || (chunk_bytes == 0U) || (chunk_bytes > SD_BENCH_MAX_CHUNK) || (read_count == 0U))
  {
    return FR_INVALID_PARAMETER;
  }

  fr = sd_mount();
  if (fr != FR_OK)
  {
    result->fresult = fr;
    return fr;
  }

  fr = f_open(&file, file_name, FA_READ);
  if (fr != FR_OK)
  {
    sd_unmount();
    result->fresult = fr;
    return fr;
  }

  file_size = f_size(&file);
  if (file_size <= (FSIZE_t)chunk_bytes)
  {
    read_count = 1U;
  }

  start_tick = HAL_GetTick();
  for (i = 0U; i < read_count; ++i)
  {
    FSIZE_t pos;
    uint32_t span;

    if (file_size <= (FSIZE_t)chunk_bytes)
    {
      pos = 0U;
    }
    else
    {
      span = (uint32_t)(file_size - (FSIZE_t)chunk_bytes);
      pos = (FSIZE_t)(((i * 1103515245UL) + 12345UL) % span);
    }

    fr = f_lseek(&file, pos);
    if (fr != FR_OK)
    {
      break;
    }

    fr = f_read(&file, sd_bench_read_buf, (UINT)chunk_bytes, &read_len);
    if (fr != FR_OK)
    {
      break;
    }
    bytes += read_len;
  }

  result->elapsed_ms = HAL_GetTick() - start_tick;
  result->bytes = bytes;
  result->bytes_per_sec = (result->elapsed_ms > 0U) ? ((bytes * 1000U) / result->elapsed_ms) : 0U;
  result->fresult = fr;

  (void)f_close(&file);
  sd_unmount();
  return fr;
}
