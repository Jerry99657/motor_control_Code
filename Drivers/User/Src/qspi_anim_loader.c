#include "qspi_anim_loader.h"

#include "qspi_w25q64.h"
#include "usbd_cdc_if.h"

#include <stdio.h>
#include <string.h>

#define START_ANIM_LOADER_MAGIC           0x314C4451U
#define START_ANIM_LOADER_HEADER_SIZE     12U
#define START_ANIM_LOADER_RX_CHUNK        8192U
#define START_ANIM_LOADER_IO_TIMEOUT_MS   180000U
#define START_ANIM_LOADER_SECTOR_SIZE     4096U
#define START_ANIM_LOADER_PROGRESS_STEP   1048576U

static uint8_t s_loader_rx_buffer[START_ANIM_LOADER_RX_CHUNK];

static uint32_t read_u32_le(const uint8_t *p)
{
  return (uint32_t)p[0] |
         ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

static uint32_t align_up_u32(uint32_t value, uint32_t align)
{
  return (value + align - 1U) & ~(align - 1U);
}

typedef int8_t (*LoaderSendFn)(void *context, const uint8_t *buf, uint16_t len);
typedef int8_t (*LoaderRecvFn)(void *context, uint8_t *buf, uint16_t len, uint32_t timeout_ms);

static uint32_t crc32_update(uint32_t crc, const uint8_t *data, uint32_t len)
{
  uint32_t i;
  uint32_t bit;

  for (i = 0; i < len; ++i)
  {
    crc ^= data[i];
    for (bit = 0; bit < 8U; ++bit)
    {
      if ((crc & 1U) != 0U)
      {
        crc = (crc >> 1U) ^ 0xEDB88320U;
      }
      else
      {
        crc >>= 1U;
      }
    }
  }

  return crc;
}

static int8_t uart_send_bytes(void *context, const uint8_t *buf, uint16_t len)
{
  UART_HandleTypeDef *huart = (UART_HandleTypeDef *)context;

  if (huart == NULL || buf == NULL || len == 0U)
  {
    return QSPI_ANIM_LOADER_ERR_PARAM;
  }

  return (HAL_UART_Transmit(huart, (uint8_t *)buf, len, 2000U) == HAL_OK) ? QSPI_ANIM_LOADER_OK : QSPI_ANIM_LOADER_ERR_UART;
}

static int8_t uart_recv_bytes(void *context, uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
  UART_HandleTypeDef *huart = (UART_HandleTypeDef *)context;

  if (huart == NULL || buf == NULL || len == 0U)
  {
    return QSPI_ANIM_LOADER_ERR_PARAM;
  }

  return (HAL_UART_Receive(huart, buf, len, timeout_ms) == HAL_OK) ? QSPI_ANIM_LOADER_OK : QSPI_ANIM_LOADER_ERR_UART;
}

static int8_t cdc_send_bytes(void *context, const uint8_t *buf, uint16_t len)
{
  uint32_t timeout_ms;
  uint32_t start_tick;
  uint8_t result;

  (void)context;

  if (buf == NULL || len == 0U)
  {
    return QSPI_ANIM_LOADER_ERR_PARAM;
  }

  timeout_ms = 2000U;
  start_tick = HAL_GetTick();
  do
  {
    result = CDC_Transmit_FS((uint8_t *)buf, len);
    if (result == USBD_OK)
    {
      return QSPI_ANIM_LOADER_OK;
    }

    if ((HAL_GetTick() - start_tick) >= timeout_ms)
    {
      break;
    }

    HAL_Delay(1U);
  }
  while (1);

  return QSPI_ANIM_LOADER_ERR_UART;
}

static int8_t cdc_recv_bytes(void *context, uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
  (void)context;

  if ((buf == NULL) || (len == 0U))
  {
    return QSPI_ANIM_LOADER_ERR_PARAM;
  }

  if (CDC_GetAndClearRxOverflow() != 0U)
  {
    return QSPI_ANIM_LOADER_ERR_UART;
  }

  if (CDC_ReadBytes(buf, len, timeout_ms) != len)
  {
    if (CDC_GetAndClearRxOverflow() != 0U)
    {
      return QSPI_ANIM_LOADER_ERR_UART;
    }

    return QSPI_ANIM_LOADER_ERR_TIMEOUT;
  }

  return QSPI_ANIM_LOADER_OK;
}

static void send_text(LoaderSendFn send_fn, void *context, const char *text)
{
  if ((send_fn == NULL) || (text == NULL))
  {
    return;
  }

  (void)send_fn(context, (const uint8_t *)text, (uint16_t)strlen(text));
}

static void send_progress(LoaderSendFn send_fn, void *context, uint32_t written, uint32_t total)
{
  char msg[48];
  int len;

  len = snprintf(msg, sizeof(msg), "PROG %lu/%lu\r\n", (unsigned long)written, (unsigned long)total);
  if (len > 0)
  {
    (void)send_fn(context, (const uint8_t *)msg, (uint16_t)len);
  }
}

static int8_t QSPI_StartAnim_DownloadSession(void *context, LoaderSendFn send_fn, LoaderRecvFn recv_fn, uint32_t base_addr, uint32_t header_timeout_ms)
{
  uint8_t header[START_ANIM_LOADER_HEADER_SIZE] = {0};
  uint32_t magic;
  uint32_t payload_size;
  uint32_t expected_crc;
  uint32_t erase_end_addr;
  uint32_t erase_addr;
  uint32_t write_addr;
  uint32_t remaining;
  uint32_t chunk_size;
  uint32_t written;
  uint32_t next_progress;
  uint32_t running_crc;

  if ((send_fn == NULL) || (recv_fn == NULL))
  {
    return QSPI_ANIM_LOADER_ERR_PARAM;
  }

  if (base_addr >= W25QXX_FLASH_SIZE_BYTES)
  {
    return QSPI_ANIM_LOADER_ERR_PARAM;
  }

  send_text(send_fn, context, "START_ANIM_LOADER\r\n");
  send_text(send_fn, context, "SEND HEADER: [QDL1][size_u32_le][crc32_u32_le]\r\n");

  if (recv_fn(context, header, START_ANIM_LOADER_HEADER_SIZE, header_timeout_ms) != QSPI_ANIM_LOADER_OK)
  {
    send_text(send_fn, context, "ERR TIMEOUT\r\n");
    return QSPI_ANIM_LOADER_ERR_TIMEOUT;
  }

  magic = read_u32_le(&header[0]);
  payload_size = read_u32_le(&header[4]);
  expected_crc = read_u32_le(&header[8]);

  if (magic != START_ANIM_LOADER_MAGIC || payload_size == 0U)
  {
    send_text(send_fn, context, "ERR HEADER\r\n");
    return QSPI_ANIM_LOADER_ERR_HEADER;
  }

  if ((uint64_t)base_addr + (uint64_t)payload_size > (uint64_t)W25QXX_FLASH_SIZE_BYTES)
  {
    send_text(send_fn, context, "ERR SIZE\r\n");
    return QSPI_ANIM_LOADER_ERR_HEADER;
  }

  send_text(send_fn, context, "OKH\r\n");

  erase_end_addr = align_up_u32(base_addr + payload_size, START_ANIM_LOADER_SECTOR_SIZE);
  for (erase_addr = base_addr; erase_addr < erase_end_addr; erase_addr += START_ANIM_LOADER_SECTOR_SIZE)
  {
    if (QSPI_W25Qxx_SectorErase(erase_addr) != QSPI_W25QXX_OK)
    {
      send_text(send_fn, context, "ERR ERASE\r\n");
      return QSPI_ANIM_LOADER_ERR_QSPI;
    }
  }

  send_text(send_fn, context, "ERASE_OK\r\n");

  remaining = payload_size;
  write_addr = base_addr;
  written = 0U;
  next_progress = START_ANIM_LOADER_PROGRESS_STEP;
  running_crc = 0xFFFFFFFFU;

  while (remaining > 0U)
  {
    chunk_size = (remaining > START_ANIM_LOADER_RX_CHUNK) ? START_ANIM_LOADER_RX_CHUNK : remaining;

    if (recv_fn(context, s_loader_rx_buffer, (uint16_t)chunk_size, START_ANIM_LOADER_IO_TIMEOUT_MS) != QSPI_ANIM_LOADER_OK)
    {
      send_text(send_fn, context, "ERR RX\r\n");
      return QSPI_ANIM_LOADER_ERR_UART;
    }

    running_crc = crc32_update(running_crc, s_loader_rx_buffer, chunk_size);

    if (QSPI_W25Qxx_WriteBuffer(s_loader_rx_buffer, write_addr, chunk_size) != QSPI_W25QXX_OK)
    {
      send_text(send_fn, context, "ERR WRITE\r\n");
      return QSPI_ANIM_LOADER_ERR_QSPI;
    }

    write_addr += chunk_size;
    written += chunk_size;
    remaining -= chunk_size;

    if (written >= next_progress || remaining == 0U)
    {
      send_progress(send_fn, context, written, payload_size);
      next_progress += START_ANIM_LOADER_PROGRESS_STEP;
    }
  }

  running_crc ^= 0xFFFFFFFFU;
  if (running_crc != expected_crc)
  {
    send_text(send_fn, context, "ERR CRC\r\n");
    return QSPI_ANIM_LOADER_ERR_CRC;
  }

  send_text(send_fn, context, "DONE\r\n");
  return QSPI_ANIM_LOADER_OK;
}

int8_t QSPI_StartAnim_DownloadViaUart(UART_HandleTypeDef *huart, uint32_t base_addr, uint32_t header_timeout_ms)
{
  return QSPI_StartAnim_DownloadSession(huart, uart_send_bytes, uart_recv_bytes, base_addr, header_timeout_ms);
}

int8_t QSPI_StartAnim_DownloadViaCdc(uint32_t base_addr, uint32_t header_timeout_ms)
{
  CDC_SetDownloadMode(1U);
  return QSPI_StartAnim_DownloadSession(NULL, cdc_send_bytes, cdc_recv_bytes, base_addr, header_timeout_ms);
}
