#include "mjpeg_player.h"

#include "fatfs.h"
#include "lcd_spi_154.h"
#include "main.h"
#include "jpeg_utils.h"
#include "mjpeg_scheduler.h"
#include <ctype.h>
#include <stdio.h>
#include <string.h>

#define MJPEG_PLAYER_DEFAULT_FRAME_MS     33U
#define MJPEG_PLAYER_SCAN_CHUNK_SIZE      1024U
#define MJPEG_PLAYER_MAX_FRAME_BYTES      (LCD_Width * LCD_Height * 2U)
#define MJPEG_PLAYER_JPEG_TIMEOUT_MS      1000U
#define MJPEG_PLAYER_DMA2D_TIMEOUT_MS     1000U
#define MJPEG_DCACHE_LINE_SIZE             32U
#define MJPEG_PLAYER_JPEG_DMA_ALIGN_BYTES  32U
#define MJPEG_PLAYER_JPEG_IN_CHUNK_BYTES   4096U
#define MJPEG_PLAYER_JPEG_OUT_CHUNK_BYTES  3840U
#define MJPEG_PLAYER_MAX_YCBCR_BYTES       (64U * 1024U)
#define MJPEG_PLAYER_JPEG_STAGE_BYTES      (MJPEG_PLAYER_JPEG_OUT_CHUNK_BYTES + 384U)

#define AVI_FOURCC(a, b, c, d) ((uint32_t)(a) | ((uint32_t)(b) << 8) | ((uint32_t)(c) << 16) | ((uint32_t)(d) << 24))

#define AVI_FCC_RIFF AVI_FOURCC('R', 'I', 'F', 'F')
#define AVI_FCC_LIST AVI_FOURCC('L', 'I', 'S', 'T')
#define AVI_FCC_AVI  AVI_FOURCC('A', 'V', 'I', ' ')
#define AVI_FCC_HDRL AVI_FOURCC('h', 'd', 'r', 'l')
#define AVI_FCC_AVIH AVI_FOURCC('a', 'v', 'i', 'h')
#define AVI_FCC_STRL AVI_FOURCC('s', 't', 'r', 'l')
#define AVI_FCC_STRH AVI_FOURCC('s', 't', 'r', 'h')
#define AVI_FCC_STRF AVI_FOURCC('s', 't', 'r', 'f')
#define AVI_FCC_MOVI AVI_FOURCC('m', 'o', 'v', 'i')
#define AVI_FCC_REC  AVI_FOURCC('r', 'e', 'c', ' ')
#define AVI_FCC_VIDS AVI_FOURCC('v', 'i', 'd', 's')
#define AVI_FCC_MJPG AVI_FOURCC('M', 'J', 'P', 'G')
#define AVI_FCC_mjpg AVI_FOURCC('m', 'j', 'p', 'g')
#define AVI_VIDS_FLAG 0x6463U
#define AVI_AUDS_FLAG 0x7762U
#define MJPEG_PLAYER_AVI_MAX_STREAM_SIZE (260U * 1024U)

#define MJPEG_ITER_AVI_STACK_DEPTH 4U
#define MJPEG_PLAYER_MAX_CONSEC_FRAME_ERRORS 8U
#define MJPEG_PLAYER_STREAM_EVAL_FRAMES   8U

typedef enum
{
  MJPEG_CONTAINER_RAW = 0,
  MJPEG_CONTAINER_AVI
} mjpeg_container_type_t;

typedef enum
{
  MJPEG_READ_STATUS_OK = 0,
  MJPEG_READ_STATUS_EOF = 1,
  MJPEG_READ_STATUS_FRAME_TOO_LARGE = 2,
  MJPEG_READ_STATUS_IO_ERR = 3,
  MJPEG_READ_STATUS_FORMAT_ERR = 4
} mjpeg_read_status_t;

typedef struct
{
  uint32_t end_pos_stack[MJPEG_ITER_AVI_STACK_DEPTH];
  uint32_t next_pos_stack[MJPEG_ITER_AVI_STACK_DEPTH];
  uint8_t depth;
  uint8_t has_target_stream;
  uint8_t target_stream_id;
} mjpeg_avi_iter_t;

typedef struct
{
  const uint8_t *in_ptr;
  uint32_t in_len;
  uint32_t in_pos;
  uint8_t *out_ptr;
  uint32_t out_cap;
  uint32_t out_len;
  uint32_t get_cb_count;
  uint32_t out_cb_count;
  uint32_t info_cb_count;
  JPEG_ConfTypeDef jpeg_info;
  JPEG_YCbCrToRGB_Convert_Function convert_func;
  uint32_t convert_block_index;
  uint32_t convert_block_size;
  uint32_t convert_total_mcus;
  uint8_t input_exhausted;
  uint8_t out_overflow;
  uint8_t convert_ready;
  uint8_t timeout;
  volatile uint8_t active;
  volatile uint8_t done;
  volatile uint8_t error;
} mjpeg_dma_decode_ctx_t;

extern JPEG_HandleTypeDef hjpeg;
extern DMA2D_HandleTypeDef hdma2d;

static uint8_t s_mjpeg_frame_io_buffer[MJPEG_PLAYER_MAX_FRAME_BYTES] __attribute__((aligned(32)));
static uint8_t s_mjpeg_ycbcr_buffer[MJPEG_PLAYER_MAX_YCBCR_BYTES] __attribute__((section(".ram_d2"), aligned(32)));
static uint8_t s_mjpeg_dma_stage_buffer[MJPEG_PLAYER_JPEG_STAGE_BYTES] __attribute__((aligned(32)));
static uint8_t s_mjpeg_dma_out_chunk[MJPEG_PLAYER_JPEG_OUT_CHUNK_BYTES] __attribute__((aligned(32)));
static uint32_t s_mjpeg_decode_fail_count = 0U;
static uint32_t s_mjpeg_skip_log_count = 0U;
static uint32_t s_mjpeg_dht_inject_log_count = 0U;
static uint32_t s_mjpeg_aligned_len_log_count = 0U;
static uint8_t s_mjpeg_jpeg_tables_ready = 0U;
static mjpeg_dma_decode_ctx_t s_mjpeg_dma_ctx;

static void mjpeg_log_value(const char *prefix, int32_t value);
static void mjpeg_log_hex32(const char *prefix, uint32_t value);

static void mjpeg_cache_clean_range(const void *addr, uint32_t size)
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
  uint32_t start;
  uint32_t end;

  if ((addr == NULL) || (size == 0U) || ((SCB->CCR & SCB_CCR_DC_Msk) == 0U))
  {
    return;
  }

  start = ((uint32_t)addr) & ~(MJPEG_DCACHE_LINE_SIZE - 1U);
  end = (((uint32_t)addr + size + MJPEG_DCACHE_LINE_SIZE - 1U) & ~(MJPEG_DCACHE_LINE_SIZE - 1U));
  SCB_CleanDCache_by_Addr((uint32_t *)start, (int32_t)(end - start));
#else
  (void)addr;
  (void)size;
#endif
}

static void mjpeg_cache_invalidate_range(void *addr, uint32_t size)
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
  uint32_t start;
  uint32_t end;

  if ((addr == NULL) || (size == 0U) || ((SCB->CCR & SCB_CCR_DC_Msk) == 0U))
  {
    return;
  }

  start = ((uint32_t)addr) & ~(MJPEG_DCACHE_LINE_SIZE - 1U);
  end = (((uint32_t)addr + size + MJPEG_DCACHE_LINE_SIZE - 1U) & ~(MJPEG_DCACHE_LINE_SIZE - 1U));
  SCB_InvalidateDCache_by_Addr((uint32_t *)start, (int32_t)(end - start));
#else
  (void)addr;
  (void)size;
#endif
}

static uint32_t mjpeg_align_down_u32(uint32_t value, uint32_t align)
{
  if (align == 0U)
  {
    return value;
  }

  return value - (value % align);
}

static uint32_t mjpeg_align_up_u32(uint32_t value, uint32_t align)
{
  if (align == 0U)
  {
    return value;
  }

  if ((value % align) == 0U)
  {
    return value;
  }

  return value + (align - (value % align));
}

static uint32_t mjpeg_jpeg_mcu_input_size(const JPEG_ConfTypeDef *info)
{
  if (info == NULL)
  {
    return 0U;
  }

  if (info->ColorSpace == JPEG_GRAYSCALE_COLORSPACE)
  {
    return 64U;
  }

  if (info->ColorSpace != JPEG_YCBCR_COLORSPACE)
  {
    return 0U;
  }

  if (info->ChromaSubsampling == JPEG_420_SUBSAMPLING)
  {
    return 384U;
  }

  if (info->ChromaSubsampling == JPEG_422_SUBSAMPLING)
  {
    return 256U;
  }

  return 192U;
}

static int8_t mjpeg_flush_dma_stage(void)
{
  uint32_t consumed_bytes;
  uint32_t converted_blocks;

  if ((s_mjpeg_dma_ctx.convert_ready == 0U) || (s_mjpeg_dma_ctx.convert_func == NULL) || (s_mjpeg_dma_ctx.convert_block_size == 0U))
  {
    return MJPEG_PLAYER_ERR_DECODE;
  }

  if (s_mjpeg_dma_ctx.out_len < s_mjpeg_dma_ctx.convert_block_size)
  {
    return MJPEG_PLAYER_OK;
  }

  consumed_bytes = s_mjpeg_dma_ctx.out_len - (s_mjpeg_dma_ctx.out_len % s_mjpeg_dma_ctx.convert_block_size);
  converted_blocks = s_mjpeg_dma_ctx.convert_func(
    s_mjpeg_dma_ctx.out_ptr,
    s_mjpeg_frame_io_buffer,
    s_mjpeg_dma_ctx.convert_block_index,
    consumed_bytes,
    &consumed_bytes
  );

  if ((converted_blocks == 0U) || (consumed_bytes == 0U) || (consumed_bytes > s_mjpeg_dma_ctx.out_len))
  {
    s_mjpeg_dma_ctx.error = 1U;
    (void)HAL_JPEG_Abort(&hjpeg);
    return MJPEG_PLAYER_ERR_DECODE;
  }

  s_mjpeg_dma_ctx.convert_block_index += converted_blocks;

  if (consumed_bytes < s_mjpeg_dma_ctx.out_len)
  {
    memmove(s_mjpeg_dma_ctx.out_ptr, &s_mjpeg_dma_ctx.out_ptr[consumed_bytes], s_mjpeg_dma_ctx.out_len - consumed_bytes);
  }

  s_mjpeg_dma_ctx.out_len -= consumed_bytes;

  return MJPEG_PLAYER_OK;
}

void HAL_JPEG_GetDataCallback(JPEG_HandleTypeDef *h, uint32_t NbDecodedData)
{
  uint32_t remaining;
  uint32_t chunk_len;

  (void)NbDecodedData;

  if ((h != &hjpeg) || (s_mjpeg_dma_ctx.active == 0U))
  {
    return;
  }

  s_mjpeg_dma_ctx.get_cb_count++;

  if (s_mjpeg_dma_ctx.in_pos >= s_mjpeg_dma_ctx.in_len)
  {
    s_mjpeg_dma_ctx.input_exhausted = 1U;
    HAL_JPEG_ConfigInputBuffer(h, (uint8_t *)s_mjpeg_dma_ctx.in_ptr, 0U);
    return;
  }

  remaining = s_mjpeg_dma_ctx.in_len - s_mjpeg_dma_ctx.in_pos;
  chunk_len = (remaining > MJPEG_PLAYER_JPEG_IN_CHUNK_BYTES) ? MJPEG_PLAYER_JPEG_IN_CHUNK_BYTES : remaining;

  HAL_JPEG_ConfigInputBuffer(
    h,
    (uint8_t *)(s_mjpeg_dma_ctx.in_ptr + s_mjpeg_dma_ctx.in_pos),
    chunk_len
  );
  s_mjpeg_dma_ctx.in_pos += chunk_len;
}

void HAL_JPEG_InfoReadyCallback(JPEG_HandleTypeDef *h, JPEG_ConfTypeDef *pInfo)
{
  uint32_t block_size;
  HAL_StatusTypeDef status;

  if ((h != &hjpeg) || (s_mjpeg_dma_ctx.active == 0U) || (pInfo == NULL))
  {
    return;
  }

  s_mjpeg_dma_ctx.info_cb_count++;

  s_mjpeg_dma_ctx.jpeg_info = *pInfo;
  s_mjpeg_dma_ctx.convert_func = NULL;
  s_mjpeg_dma_ctx.convert_total_mcus = 0U;
  s_mjpeg_dma_ctx.convert_block_index = 0U;
  s_mjpeg_dma_ctx.convert_block_size = 0U;

  block_size = mjpeg_jpeg_mcu_input_size(pInfo);
  if (block_size == 0U)
  {
    s_mjpeg_dma_ctx.error = 1U;
    (void)HAL_JPEG_Abort(h);
    return;
  }

  status = JPEG_GetDecodeColorConvertFunc(pInfo, &s_mjpeg_dma_ctx.convert_func, &s_mjpeg_dma_ctx.convert_total_mcus);
  if ((status != HAL_OK) || (s_mjpeg_dma_ctx.convert_func == NULL))
  {
    s_mjpeg_dma_ctx.error = 1U;
    (void)HAL_JPEG_Abort(h);
    return;
  }

  s_mjpeg_dma_ctx.convert_block_size = block_size;
  s_mjpeg_dma_ctx.convert_ready = 1U;
}

void HAL_JPEG_DataReadyCallback(JPEG_HandleTypeDef *h, uint8_t *pDataOut, uint32_t OutDataLength)
{
  if ((h != &hjpeg) || (s_mjpeg_dma_ctx.active == 0U))
  {
    return;
  }

  s_mjpeg_dma_ctx.out_cb_count++;

  if ((pDataOut == NULL) || (OutDataLength == 0U))
  {
    HAL_JPEG_ConfigOutputBuffer(h, s_mjpeg_dma_out_chunk, sizeof(s_mjpeg_dma_out_chunk));
    return;
  }

  mjpeg_cache_invalidate_range(pDataOut, OutDataLength);

  if ((s_mjpeg_dma_ctx.out_len + OutDataLength) > s_mjpeg_dma_ctx.out_cap)
  {
    s_mjpeg_dma_ctx.out_overflow = 1U;
    s_mjpeg_dma_ctx.error = 1U;
    (void)HAL_JPEG_Abort(h);
    return;
  }

  memcpy(
    &s_mjpeg_dma_ctx.out_ptr[s_mjpeg_dma_ctx.out_len],
    pDataOut,
    OutDataLength
  );
  s_mjpeg_dma_ctx.out_len += OutDataLength;

  if (mjpeg_flush_dma_stage() != MJPEG_PLAYER_OK)
  {
    return;
  }

  HAL_JPEG_ConfigOutputBuffer(h, s_mjpeg_dma_out_chunk, sizeof(s_mjpeg_dma_out_chunk));
}

void HAL_JPEG_DecodeCpltCallback(JPEG_HandleTypeDef *h)
{
  if (h != &hjpeg)
  {
    return;
  }

  s_mjpeg_dma_ctx.done = 1U;
}

void HAL_JPEG_ErrorCallback(JPEG_HandleTypeDef *h)
{
  if (h != &hjpeg)
  {
    return;
  }

  s_mjpeg_dma_ctx.error = 1U;
}

static int8_t mjpeg_decode_frame_via_dma(uint8_t *jpeg_data, uint32_t decode_len, const JPEG_ConfTypeDef *prefill_info)
{
  HAL_StatusTypeDef status;
  uint32_t first_chunk;
  uint32_t start_ms;

  if ((jpeg_data == NULL) || (decode_len < 4U))
  {
    return MJPEG_PLAYER_ERR_PARAM;
  }

  mjpeg_log_value("MJPEG: dma start len=", (int32_t)decode_len);

  memset(&s_mjpeg_dma_ctx, 0, sizeof(s_mjpeg_dma_ctx));
  s_mjpeg_dma_ctx.in_ptr = jpeg_data;
  s_mjpeg_dma_ctx.in_len = decode_len;
  s_mjpeg_dma_ctx.out_ptr = s_mjpeg_dma_stage_buffer;
  s_mjpeg_dma_ctx.out_cap = sizeof(s_mjpeg_dma_stage_buffer);
  s_mjpeg_dma_ctx.active = 1U;

  if (prefill_info != NULL)
  {
    uint32_t block_size;

    s_mjpeg_dma_ctx.jpeg_info = *prefill_info;
    block_size = mjpeg_jpeg_mcu_input_size(prefill_info);
    if (block_size != 0U)
    {
      status = JPEG_GetDecodeColorConvertFunc(
        &s_mjpeg_dma_ctx.jpeg_info,
        &s_mjpeg_dma_ctx.convert_func,
        &s_mjpeg_dma_ctx.convert_total_mcus
      );
      if ((status == HAL_OK) && (s_mjpeg_dma_ctx.convert_func != NULL))
      {
        s_mjpeg_dma_ctx.convert_block_size = block_size;
        s_mjpeg_dma_ctx.convert_ready = 1U;
      }
    }
  }

  first_chunk = (decode_len > MJPEG_PLAYER_JPEG_IN_CHUNK_BYTES) ? MJPEG_PLAYER_JPEG_IN_CHUNK_BYTES : decode_len;

  s_mjpeg_dma_ctx.in_pos = first_chunk;

  mjpeg_cache_clean_range(jpeg_data, decode_len);
  mjpeg_cache_clean_range(s_mjpeg_frame_io_buffer, sizeof(s_mjpeg_frame_io_buffer));
  mjpeg_cache_clean_range(s_mjpeg_dma_stage_buffer, sizeof(s_mjpeg_dma_stage_buffer));
  mjpeg_cache_invalidate_range(s_mjpeg_dma_out_chunk, sizeof(s_mjpeg_dma_out_chunk));

  if (HAL_JPEG_Decode_DMA(
        &hjpeg,
        jpeg_data,
        first_chunk,
        s_mjpeg_dma_out_chunk,
        sizeof(s_mjpeg_dma_out_chunk)
      ) != HAL_OK)
  {
    s_mjpeg_dma_ctx.active = 0U;
    return MJPEG_PLAYER_ERR_DECODE;
  }

  start_ms = HAL_GetTick();
  while ((s_mjpeg_dma_ctx.done == 0U) && (s_mjpeg_dma_ctx.error == 0U))
  {
    if ((HAL_GetTick() - start_ms) > MJPEG_PLAYER_JPEG_TIMEOUT_MS)
    {
      (void)HAL_JPEG_Abort(&hjpeg);
      s_mjpeg_dma_ctx.timeout = 1U;
      s_mjpeg_dma_ctx.error = 1U;
      s_mjpeg_dma_ctx.active = 0U;
      return MJPEG_PLAYER_ERR_DECODE;
    }
  }

  s_mjpeg_dma_ctx.active = 0U;

  if (s_mjpeg_dma_ctx.error != 0U)
  {
    return MJPEG_PLAYER_ERR_DECODE;
  }

  if (mjpeg_flush_dma_stage() != MJPEG_PLAYER_OK)
  {
    return MJPEG_PLAYER_ERR_DECODE;
  }

  if (s_mjpeg_dma_ctx.out_len != 0U)
  {
    return MJPEG_PLAYER_ERR_DECODE;
  }

  return MJPEG_PLAYER_OK;
}

/* Default Huffman tables used by many MJPEG-in-AVI decoders when DHT is absent. */
static const uint8_t s_mjpeg_default_dht[] = {
  0xFF, 0xC4, 0x01, 0xA2,
  0x00,
  0x00, 0x01, 0x05, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B,
  0x10,
  0x00, 0x02, 0x01, 0x03, 0x03, 0x02, 0x04, 0x03, 0x05, 0x05, 0x04, 0x04, 0x00, 0x00, 0x01, 0x7D,
  0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12, 0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61,
  0x07, 0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xA1, 0x08, 0x23, 0x42, 0xB1, 0xC1, 0x15, 0x52, 0xD1,
  0xF0, 0x24, 0x33, 0x62, 0x72, 0x82,
  0x09, 0x0A, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x34, 0x35, 0x36,
  0x37, 0x38, 0x39, 0x3A, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x53, 0x54, 0x55, 0x56,
  0x57, 0x58, 0x59, 0x5A, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6A, 0x73, 0x74, 0x75, 0x76,
  0x77, 0x78, 0x79, 0x7A, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8A, 0x92, 0x93, 0x94, 0x95,
  0x96, 0x97, 0x98, 0x99, 0x9A, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xB2, 0xB3,
  0xB4, 0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xBA, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA,
  0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7, 0xD8, 0xD9, 0xDA, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7,
  0xE8, 0xE9, 0xEA, 0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8, 0xF9, 0xFA,
  0x01,
  0x00, 0x03, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B,
  0x11,
  0x00, 0x02, 0x01, 0x02, 0x04, 0x04, 0x03, 0x04, 0x07, 0x05, 0x04, 0x04, 0x00, 0x01, 0x02, 0x77,
  0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21, 0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61, 0x71,
  0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91, 0xA1, 0xB1, 0xC1, 0x09, 0x23, 0x33, 0x52, 0xF0,
  0x15, 0x62, 0x72, 0xD1, 0x0A, 0x16, 0x24, 0x34,
  0xE1, 0x25, 0xF1, 0x17, 0x18, 0x19, 0x1A, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x35, 0x36, 0x37, 0x38,
  0x39, 0x3A, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58,
  0x59, 0x5A, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6A, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78,
  0x79, 0x7A, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8A, 0x92, 0x93, 0x94, 0x95, 0x96,
  0x97, 0x98, 0x99, 0x9A, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xB2, 0xB3, 0xB4,
  0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xBA, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xD2,
  0xD3, 0xD4, 0xD5, 0xD6, 0xD7, 0xD8, 0xD9, 0xDA, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7, 0xE8, 0xE9,
  0xEA, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8, 0xF9, 0xFA
};

static uint32_t mjpeg_read_u32_le(const uint8_t *p)
{
  return (uint32_t)p[0] |
         ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

static uint8_t mjpeg_is_ext(const char *name, const char *ext)
{
  size_t name_len;
  size_t ext_len;
  size_t i;

  if ((name == NULL) || (ext == NULL))
  {
    return 0U;
  }

  name_len = strlen(name);
  ext_len = strlen(ext);
  if ((ext_len == 0U) || (name_len <= ext_len))
  {
    return 0U;
  }

  if (name[name_len - ext_len - 1U] != '.')
  {
    return 0U;
  }

  for (i = 0U; i < ext_len; ++i)
  {
    if ((char)toupper((unsigned char)name[name_len - ext_len + i]) != (char)toupper((unsigned char)ext[i]))
    {
      return 0U;
    }
  }

  return 1U;
}

static uint8_t mjpeg_stop_requested(void)
{
  return (HAL_GPIO_ReadPin(Key2_GPIO_Port, Key2_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
}

static void mjpeg_log(const char *msg)
{
  Boot_DebugStageLog(msg);
}

static void mjpeg_log_info(const char *prefix, uint32_t color_space, uint32_t chroma)
{
  char line[96];
  int len;

  len = snprintf(line, sizeof(line), "%s cs=%lu sub=%lu\r\n", prefix, (unsigned long)color_space, (unsigned long)chroma);
  if (len > 0)
  {
    Boot_DebugStageLog(line);
  }
}

static void mjpeg_log_value(const char *prefix, int32_t value)
{
  char line[96];
  int len;

  len = snprintf(line, sizeof(line), "%s%ld\r\n", prefix, (long)value);
  if (len > 0)
  {
    Boot_DebugStageLog(line);
  }
}

static void mjpeg_log_hex32(const char *prefix, uint32_t value)
{
  char line[96];
  int len;

  len = snprintf(line, sizeof(line), "%s%08lX\r\n", prefix, (unsigned long)value);
  if (len > 0)
  {
    Boot_DebugStageLog(line);
  }
}

static void mjpeg_log_text2(const char *prefix, const char *text)
{
  char line[96];
  int len;

  if ((prefix == NULL) || (text == NULL) || (text[0] == '\0'))
  {
    return;
  }

  len = snprintf(line, sizeof(line), "%s%s\r\n", prefix, text);
  if (len > 0)
  {
    Boot_DebugStageLog(line);
  }
}

static void mjpeg_build_avi_video_tag(uint8_t stream_id, char tag[5])
{
  if (tag == NULL)
  {
    return;
  }

  if (stream_id > 99U)
  {
    tag[0] = '\0';
    return;
  }

  tag[0] = (char)('0' + (stream_id / 10U));
  tag[1] = (char)('0' + (stream_id % 10U));
  tag[2] = 'd';
  tag[3] = 'c';
  tag[4] = '\0';
}

static uint8_t mjpeg_chunk_tag_match(const uint8_t *p, const char *tag)
{
  if (p == NULL)
  {
    return 0U;
  }

  if ((tag != NULL) && (tag[0] != '\0'))
  {
    return (uint8_t)((p[0] == (uint8_t)tag[0]) &&
                     (p[1] == (uint8_t)tag[1]) &&
                     (p[2] == (uint8_t)tag[2]) &&
                     (p[3] == (uint8_t)tag[3]));
  }

  return (uint8_t)((isdigit((int)p[0]) != 0) &&
                   (isdigit((int)p[1]) != 0) &&
                   (p[2] == (uint8_t)'d') &&
                   (p[3] == (uint8_t)'c'));
}

static void mjpeg_recover_jpeg_core(void)
{
  (void)HAL_JPEG_Abort(&hjpeg);
  (void)HAL_JPEG_DeInit(&hjpeg);
  if (HAL_JPEG_Init(&hjpeg) != HAL_OK)
  {
    mjpeg_log("MJPEG: HAL_JPEG_Init failed\r\n");
  }
}

typedef struct
{
  uint16_t width;
  uint16_t height;
  uint8_t components;
  uint8_t luma_sampling_h;
  uint8_t luma_sampling_v;
  uint8_t progressive;
  uint8_t valid;
} mjpeg_jpeg_header_info_t;

static uint16_t mjpeg_read_u16_be(const uint8_t *p)
{
  return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static int8_t mjpeg_parse_jpeg_header(const uint8_t *jpeg_data, uint32_t jpeg_len, mjpeg_jpeg_header_info_t *header)
{
  uint32_t pos;

  if ((jpeg_data == NULL) || (header == NULL) || (jpeg_len < 4U))
  {
    return MJPEG_PLAYER_ERR_PARAM;
  }

  memset(header, 0, sizeof(*header));

  if ((jpeg_data[0] != 0xFFU) || (jpeg_data[1] != 0xD8U))
  {
    return MJPEG_PLAYER_ERR_FORMAT;
  }

  pos = 2U;
  while ((pos + 3U) < jpeg_len)
  {
    uint8_t marker;
    uint32_t seg_len;

    while ((pos < jpeg_len) && (jpeg_data[pos] == 0xFFU))
    {
      pos++;
    }

    if (pos >= jpeg_len)
    {
      break;
    }

    marker = jpeg_data[pos++];
    if ((marker == 0xD8U) || (marker == 0xD9U) || (marker == 0x01U) ||
        ((marker >= 0xD0U) && (marker <= 0xD7U)))
    {
      continue;
    }

    if (marker == 0xDAU)
    {
      break;
    }

    if ((pos + 1U) >= jpeg_len)
    {
      return MJPEG_PLAYER_ERR_FORMAT;
    }

    seg_len = (uint32_t)mjpeg_read_u16_be(&jpeg_data[pos]);
    if ((seg_len < 2U) || ((pos + seg_len) > jpeg_len))
    {
      return MJPEG_PLAYER_ERR_FORMAT;
    }

    if ((marker == 0xC0U) || (marker == 0xC1U) || (marker == 0xC2U))
    {
      if (seg_len < 8U)
      {
        return MJPEG_PLAYER_ERR_FORMAT;
      }

      header->progressive = (marker == 0xC2U) ? 1U : 0U;
      header->height = mjpeg_read_u16_be(&jpeg_data[pos + 3U]);
      header->width = mjpeg_read_u16_be(&jpeg_data[pos + 5U]);
      header->components = jpeg_data[pos + 7U];

      if (header->components >= 1U)
      {
        uint8_t sampling = jpeg_data[pos + 9U];
        header->luma_sampling_h = (uint8_t)((sampling >> 4U) & 0x0FU);
        header->luma_sampling_v = (uint8_t)(sampling & 0x0FU);
      }

      header->valid = 1U;
      return MJPEG_PLAYER_OK;
    }

    pos += seg_len;
  }

  return MJPEG_PLAYER_ERR_FORMAT;
}

static int8_t mjpeg_prepare_decode_stream(
  uint8_t *jpeg_data,
  uint32_t jpeg_len,
  uint32_t jpeg_capacity,
  uint32_t *decode_len
)
{
  uint32_t pos;
  uint32_t soi_pos;
  uint32_t eoi_pos;
  uint8_t has_dht;
  uint8_t has_sos;

  if ((jpeg_data == NULL) || (decode_len == NULL) || (jpeg_len < 4U))
  {
    return MJPEG_PLAYER_ERR_PARAM;
  }

  soi_pos = jpeg_len;
  for (pos = 0U; (pos + 1U) < jpeg_len; ++pos)
  {
    if ((jpeg_data[pos] == 0xFFU) && (jpeg_data[pos + 1U] == 0xD8U))
    {
      soi_pos = pos;
      break;
    }
  }

  if (soi_pos >= jpeg_len)
  {
    return MJPEG_PLAYER_ERR_FORMAT;
  }

  if (soi_pos > 0U)
  {
    memmove(&jpeg_data[0], &jpeg_data[soi_pos], jpeg_len - soi_pos);
    jpeg_len -= soi_pos;
    mjpeg_log("MJPEG: SOI realigned\r\n");
  }

  if ((jpeg_data[0] != 0xFFU) || (jpeg_data[1] != 0xD8U))
  {
    return MJPEG_PLAYER_ERR_FORMAT;
  }

  eoi_pos = jpeg_len;
  for (pos = 2U; (pos + 1U) < jpeg_len; ++pos)
  {
    if ((jpeg_data[pos] == 0xFFU) && (jpeg_data[pos + 1U] == 0xD9U))
    {
      eoi_pos = pos;
      break;
    }
  }

  if (eoi_pos < jpeg_len)
  {
    jpeg_len = eoi_pos + 2U;
  }
  else
  {
    if ((jpeg_len + 2U) > jpeg_capacity)
    {
      return MJPEG_PLAYER_ERR_FRAME_TOO_LARGE;
    }

    jpeg_data[jpeg_len++] = 0xFFU;
    jpeg_data[jpeg_len++] = 0xD9U;
    mjpeg_log("MJPEG: EOI appended\r\n");
  }

  pos = 2U;
  has_dht = 0U;
  has_sos = 0U;

  while ((pos + 3U) < jpeg_len)
  {
    uint8_t marker;
    uint32_t seg_len;

    if (jpeg_data[pos] != 0xFFU)
    {
      pos++;
      continue;
    }

    while ((pos < jpeg_len) && (jpeg_data[pos] == 0xFFU))
    {
      pos++;
    }

    if (pos >= jpeg_len)
    {
      break;
    }

    marker = jpeg_data[pos++];

    if ((marker == 0xD8U) || (marker == 0xD9U) || (marker == 0x01U) ||
        ((marker >= 0xD0U) && (marker <= 0xD7U)))
    {
      continue;
    }

    if (marker == 0xDAU)
    {
      has_sos = 1U;
      break;
    }

    if ((pos + 1U) >= jpeg_len)
    {
      return MJPEG_PLAYER_ERR_FORMAT;
    }

    seg_len = ((uint32_t)jpeg_data[pos] << 8) | (uint32_t)jpeg_data[pos + 1U];
    if ((seg_len < 2U) || ((pos + seg_len) > jpeg_len))
    {
      return MJPEG_PLAYER_ERR_FORMAT;
    }

    if (marker == 0xC4U)
    {
      has_dht = 1U;
    }
    else if (marker == 0xC2U)
    {
      mjpeg_log("MJPEG: progressive JPEG not supported\r\n");
      return MJPEG_PLAYER_ERR_UNSUPPORTED;
    }

    pos += seg_len;
  }

  if (has_sos == 0U)
  {
    return MJPEG_PLAYER_ERR_FORMAT;
  }

  if ((has_sos != 0U) && (has_dht == 0U))
  {
    uint32_t patched_len = jpeg_len + (uint32_t)sizeof(s_mjpeg_default_dht);
    uint32_t pad_len;

    if (patched_len > jpeg_capacity)
    {
      return MJPEG_PLAYER_ERR_FRAME_TOO_LARGE;
    }

    memmove(
      &jpeg_data[2U + sizeof(s_mjpeg_default_dht)],
      &jpeg_data[2],
      jpeg_len - 2U
    );
    memcpy(&jpeg_data[2], s_mjpeg_default_dht, sizeof(s_mjpeg_default_dht));

    pad_len = (MJPEG_PLAYER_JPEG_DMA_ALIGN_BYTES -
               (patched_len & (MJPEG_PLAYER_JPEG_DMA_ALIGN_BYTES - 1U))) &
              (MJPEG_PLAYER_JPEG_DMA_ALIGN_BYTES - 1U);
    if ((patched_len + pad_len) > jpeg_capacity)
    {
      return MJPEG_PLAYER_ERR_FRAME_TOO_LARGE;
    }

    if (pad_len != 0U)
    {
      memset(&jpeg_data[patched_len], 0, pad_len);
      patched_len += pad_len;
    }

    *decode_len = patched_len;
    s_mjpeg_dht_inject_log_count++;
    if ((s_mjpeg_dht_inject_log_count <= 6U) || ((s_mjpeg_dht_inject_log_count & 0x7U) == 0U))
    {
      mjpeg_log("MJPEG: DHT injected\r\n");
    }
    return MJPEG_PLAYER_OK;
  }

  {
    uint32_t pad_len = (MJPEG_PLAYER_JPEG_DMA_ALIGN_BYTES -
                        (jpeg_len & (MJPEG_PLAYER_JPEG_DMA_ALIGN_BYTES - 1U))) &
                       (MJPEG_PLAYER_JPEG_DMA_ALIGN_BYTES - 1U);
    if ((jpeg_len + pad_len) > jpeg_capacity)
    {
      return MJPEG_PLAYER_ERR_FRAME_TOO_LARGE;
    }

    if (pad_len != 0U)
    {
      memset(&jpeg_data[jpeg_len], 0, pad_len);
      jpeg_len += pad_len;
    }
  }

  *decode_len = jpeg_len;
  return MJPEG_PLAYER_OK;
}

static int8_t mjpeg_wait_next_frame_tick(uint32_t wait_ms)
{
  uint32_t start_ms;

  start_ms = HAL_GetTick();

  for (;;)
  {
    if (mjpeg_stop_requested() != 0U)
    {
      return MJPEG_PLAYER_ERR_STOPPED;
    }

    if (MJPEG_Scheduler_ConsumeFrameTick() != 0U)
    {
      return MJPEG_PLAYER_OK;
    }

    if ((HAL_GetTick() - start_ms) >= wait_ms)
    {
      return MJPEG_PLAYER_OK;
    }

    HAL_Delay(1U);
  }
}

static int8_t mjpeg_dma2d_convert_and_show(const JPEG_ConfTypeDef *jpeg_info)
{
  uint16_t x;
  uint16_t y;
  uint32_t css;

  if (jpeg_info == NULL)
  {
    return MJPEG_PLAYER_ERR_PARAM;
  }

  if ((jpeg_info->ImageWidth == 0U) || (jpeg_info->ImageHeight == 0U) ||
      (jpeg_info->ImageWidth > LCD_Width) || (jpeg_info->ImageHeight > LCD_Height))
  {
    mjpeg_log("MJPEG: image size unsupported\r\n");
    return MJPEG_PLAYER_ERR_UNSUPPORTED;
  }

  if (jpeg_info->ChromaSubsampling == JPEG_420_SUBSAMPLING)
  {
    css = DMA2D_CSS_420;
  }
  else if (jpeg_info->ChromaSubsampling == JPEG_422_SUBSAMPLING)
  {
    css = DMA2D_CSS_422;
    mjpeg_log("MJPEG: chroma 4:2:2 -> DMA2D CSS422\r\n");
  }
  else
  {
    mjpeg_log("MJPEG: chroma unsupported (need 4:2:0 or 4:2:2)\r\n");
    return MJPEG_PLAYER_ERR_UNSUPPORTED;
  }

  hdma2d.Init.Mode = DMA2D_M2M_PFC;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_RGB565;
  hdma2d.Init.OutputOffset = 0U;
  hdma2d.LayerCfg[1].InputOffset = 0U;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_YCBCR;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0U;
  hdma2d.LayerCfg[1].AlphaInverted = DMA2D_REGULAR_ALPHA;
  hdma2d.LayerCfg[1].RedBlueSwap = DMA2D_RB_REGULAR;
  hdma2d.LayerCfg[1].ChromaSubSampling = css;

  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    mjpeg_log("MJPEG: HAL_DMA2D_Init failed\r\n");
    return MJPEG_PLAYER_ERR_DECODE;
  }

  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1U) != HAL_OK)
  {
    mjpeg_log("MJPEG: HAL_DMA2D_ConfigLayer failed\r\n");
    return MJPEG_PLAYER_ERR_DECODE;
  }

  if (HAL_DMA2D_Start(
        &hdma2d,
        (uint32_t)s_mjpeg_ycbcr_buffer,
        (uint32_t)s_mjpeg_frame_io_buffer,
        jpeg_info->ImageWidth,
        jpeg_info->ImageHeight
      ) != HAL_OK)
  {
    mjpeg_log("MJPEG: HAL_DMA2D_Start failed\r\n");
    return MJPEG_PLAYER_ERR_DECODE;
  }

  if (HAL_DMA2D_PollForTransfer(&hdma2d, MJPEG_PLAYER_DMA2D_TIMEOUT_MS) != HAL_OK)
  {
    mjpeg_log("MJPEG: HAL_DMA2D_PollForTransfer timeout\r\n");
    return MJPEG_PLAYER_ERR_DECODE;
  }

  x = (uint16_t)((LCD_Width - jpeg_info->ImageWidth) / 2U);
  y = (uint16_t)((LCD_Height - jpeg_info->ImageHeight) / 2U);
  LCD_CopyBuffer(x, y, (uint16_t)jpeg_info->ImageWidth, (uint16_t)jpeg_info->ImageHeight, (const uint16_t *)s_mjpeg_frame_io_buffer);

  return MJPEG_PLAYER_OK;
}

static int8_t mjpeg_gray_convert_and_show(const mjpeg_jpeg_header_info_t *header)
{
  uint32_t width;
  uint32_t height;
  uint32_t show_width;
  uint32_t show_height;
  uint32_t row;
  uint32_t col;
  uint16_t x;
  uint16_t y;
  uint8_t gray;
  uint16_t rgb565;

  if (header == NULL)
  {
    return MJPEG_PLAYER_ERR_PARAM;
  }

  width = header->width;
  height = header->height;
  if ((width == 0U) || (height == 0U))
  {
    mjpeg_log("MJPEG: gray image size invalid\r\n");
    return MJPEG_PLAYER_ERR_UNSUPPORTED;
  }

  show_width = (width > LCD_Width) ? LCD_Width : width;
  show_height = (height > LCD_Height) ? LCD_Height : height;

  if ((show_width != width) || (show_height != height))
  {
    mjpeg_log_info("MJPEG: gray crop", width, height);
  }

  for (row = 0U; row < show_height; ++row)
  {
    for (col = 0U; col < show_width; ++col)
    {
      gray = s_mjpeg_ycbcr_buffer[(row * width) + col];
      rgb565 = (uint16_t)(((uint16_t)(gray >> 3U) << 11U) |
                          ((uint16_t)(gray >> 2U) << 5U) |
                          ((uint16_t)(gray >> 3U)));
      ((uint16_t *)s_mjpeg_frame_io_buffer)[(row * show_width) + col] = rgb565;
    }
  }

  x = (uint16_t)((LCD_Width > show_width) ? ((LCD_Width - show_width) / 2U) : 0U);
  y = (uint16_t)((LCD_Height > show_height) ? ((LCD_Height - show_height) / 2U) : 0U);
  LCD_CopyBuffer(x, y, (uint16_t)show_width, (uint16_t)show_height, (const uint16_t *)s_mjpeg_frame_io_buffer);

  return MJPEG_PLAYER_OK;
}

static int8_t mjpeg_decode_one_frame(uint8_t *jpeg_data, uint32_t jpeg_len, uint16_t fallback_width, uint16_t fallback_height)
{
  uint32_t decode_len;
  JPEG_ConfTypeDef prefill_info;
  JPEG_ConfTypeDef info;
  mjpeg_jpeg_header_info_t header_info;
  uint8_t has_prefill_info;
  int8_t prepare_status;
  int8_t dma_status;
  int8_t header_status;

  if ((jpeg_data == NULL) || (jpeg_len == 0U))
  {
    return MJPEG_PLAYER_ERR_PARAM;
  }

  mjpeg_log_value("MJPEG: decode enter len=", (int32_t)jpeg_len);

  prepare_status = mjpeg_prepare_decode_stream(
    jpeg_data,
    jpeg_len,
    (uint32_t)sizeof(s_mjpeg_ycbcr_buffer),
    &decode_len
  );
  if (prepare_status != MJPEG_PLAYER_OK)
  {
    return prepare_status;
  }

  if (decode_len <= 4096U)
  {
    s_mjpeg_aligned_len_log_count++;
    if ((s_mjpeg_aligned_len_log_count <= 8U) || ((s_mjpeg_aligned_len_log_count & 0x3FU) == 0U))
    {
      mjpeg_log_value("MJPEG: dma aligned len=", (int32_t)decode_len);
    }
  }

  header_status = mjpeg_parse_jpeg_header(jpeg_data, decode_len, &header_info);
  if (header_status != MJPEG_PLAYER_OK)
  {
    memset(&header_info, 0, sizeof(header_info));
  }

  if (header_status == MJPEG_PLAYER_OK)
  {
    if (header_info.components == 3U)
    {
      if (!((header_info.luma_sampling_h == 2U) && (header_info.luma_sampling_v == 2U)) &&
          !((header_info.luma_sampling_h == 2U) && (header_info.luma_sampling_v == 1U)) &&
          !((header_info.luma_sampling_h == 1U) && (header_info.luma_sampling_v == 1U)))
      {
        return MJPEG_PLAYER_ERR_UNSUPPORTED;
      }
    }
    else if (header_info.components != 1U)
    {
      return MJPEG_PLAYER_ERR_UNSUPPORTED;
    }
  }

  has_prefill_info = 0U;
  memset(&prefill_info, 0, sizeof(prefill_info));
  if ((header_status == MJPEG_PLAYER_OK) && (header_info.width != 0U) && (header_info.height != 0U))
  {
    prefill_info.ImageWidth = header_info.width;
    prefill_info.ImageHeight = header_info.height;

    if (header_info.components == 1U)
    {
      prefill_info.ColorSpace = JPEG_GRAYSCALE_COLORSPACE;
      prefill_info.ChromaSubsampling = JPEG_444_SUBSAMPLING;
      has_prefill_info = 1U;
    }
    else if (header_info.components == 3U)
    {
      prefill_info.ColorSpace = JPEG_YCBCR_COLORSPACE;
      if ((header_info.luma_sampling_h == 2U) && (header_info.luma_sampling_v == 2U))
      {
        prefill_info.ChromaSubsampling = JPEG_420_SUBSAMPLING;
        has_prefill_info = 1U;
      }
      else if ((header_info.luma_sampling_h == 2U) && (header_info.luma_sampling_v == 1U))
      {
        prefill_info.ChromaSubsampling = JPEG_422_SUBSAMPLING;
        has_prefill_info = 1U;
      }
      else if ((header_info.luma_sampling_h == 1U) && (header_info.luma_sampling_v == 1U))
      {
        prefill_info.ChromaSubsampling = JPEG_444_SUBSAMPLING;
        has_prefill_info = 1U;
      }
    }
  }

  /* Keep JPEG HW state deterministic frame-by-frame, as in the reference flow. */
  mjpeg_recover_jpeg_core();

  dma_status = mjpeg_decode_frame_via_dma(
    jpeg_data,
    decode_len,
    (has_prefill_info != 0U) ? &prefill_info : NULL
  );
  if (dma_status != MJPEG_PLAYER_OK)
  {
    s_mjpeg_decode_fail_count++;
    if ((s_mjpeg_decode_fail_count <= 4U) || ((s_mjpeg_decode_fail_count & 0x7U) == 0U))
    {
      mjpeg_log_value("MJPEG: decode len=", (int32_t)decode_len);
      mjpeg_log_hex32("MJPEG: JPEG err=0x", hjpeg.ErrorCode);
      mjpeg_log_value("MJPEG: dma info_cb=", (int32_t)s_mjpeg_dma_ctx.info_cb_count);
      mjpeg_log_value("MJPEG: dma get_cb=", (int32_t)s_mjpeg_dma_ctx.get_cb_count);
      mjpeg_log_value("MJPEG: dma out_cb=", (int32_t)s_mjpeg_dma_ctx.out_cb_count);
      mjpeg_log_value("MJPEG: dma cvt_ready=", (int32_t)s_mjpeg_dma_ctx.convert_ready);
      if (s_mjpeg_dma_ctx.timeout != 0U)
      {
        mjpeg_log("MJPEG: dma timeout\r\n");
      }
      if (s_mjpeg_dma_ctx.input_exhausted != 0U)
      {
        mjpeg_log("MJPEG: dma input exhausted\r\n");
      }
      if (s_mjpeg_dma_ctx.out_overflow != 0U)
      {
        mjpeg_log("MJPEG: dma out overflow\r\n");
      }
      mjpeg_log("MJPEG: HAL_JPEG_Decode_DMA failed\r\n");
    }
    mjpeg_recover_jpeg_core();
    return MJPEG_PLAYER_ERR_DECODE;
  }

  info = s_mjpeg_dma_ctx.jpeg_info;
  s_mjpeg_decode_fail_count = 0U;

  if ((info.ImageWidth == 0U) || (info.ImageHeight == 0U))
  {
    info.ImageWidth = header_info.width;
    info.ImageHeight = header_info.height;
  }

  if ((info.ImageWidth == 0U) || (info.ImageHeight == 0U))
  {
    info.ImageWidth = fallback_width;
    info.ImageHeight = fallback_height;
  }

  if ((header_info.width == 0U) || (header_info.height == 0U))
  {
    header_info.width = (uint16_t)info.ImageWidth;
    header_info.height = (uint16_t)info.ImageHeight;
  }

  if ((info.ImageWidth == 0U) || (info.ImageHeight == 0U) ||
      (info.ImageWidth > LCD_Width) || (info.ImageHeight > LCD_Height))
  {
    mjpeg_log("MJPEG: image size unsupported\r\n");
    return MJPEG_PLAYER_ERR_UNSUPPORTED;
  }

  mjpeg_cache_clean_range(s_mjpeg_frame_io_buffer, (uint32_t)info.ImageWidth * (uint32_t)info.ImageHeight * 2U);
  LCD_CopyBuffer(
    (uint16_t)((LCD_Width - info.ImageWidth) / 2U),
    (uint16_t)((LCD_Height - info.ImageHeight) / 2U),
    (uint16_t)info.ImageWidth,
    (uint16_t)info.ImageHeight,
    (const uint16_t *)s_mjpeg_frame_io_buffer
  );

  return MJPEG_PLAYER_OK;
}

static mjpeg_read_status_t mjpeg_read_exact(FIL *file, uint8_t *buf, uint32_t bytes)
{
  FRESULT fr;
  UINT read_len;

  if ((file == NULL) || (buf == NULL))
  {
    return MJPEG_READ_STATUS_IO_ERR;
  }

  fr = f_read(file, buf, (UINT)bytes, &read_len);
  if ((fr != FR_OK) || (read_len != (UINT)bytes))
  {
    return MJPEG_READ_STATUS_IO_ERR;
  }

  return MJPEG_READ_STATUS_OK;
}

static mjpeg_read_status_t mjpeg_raw_read_next_frame(FIL *file, uint8_t *out_buf, uint32_t out_buf_size, uint32_t *frame_len)
{
  uint8_t stage[MJPEG_PLAYER_SCAN_CHUNK_SIZE];
  uint8_t prev = 0U;
  uint8_t has_prev = 0U;
  uint8_t capture = 0U;
  uint32_t out_len = 0U;
  FRESULT fr;
  UINT read_len;

  if ((file == NULL) || (out_buf == NULL) || (frame_len == NULL))
  {
    return MJPEG_READ_STATUS_IO_ERR;
  }

  *frame_len = 0U;

  while (1)
  {
    fr = f_read(file, stage, (UINT)sizeof(stage), &read_len);
    if (fr != FR_OK)
    {
      return MJPEG_READ_STATUS_IO_ERR;
    }

    if (read_len == 0U)
    {
      break;
    }

    for (UINT i = 0U; i < read_len; ++i)
    {
      uint8_t byte = stage[i];

      if (capture == 0U)
      {
        if ((has_prev != 0U) && (prev == 0xFFU) && (byte == 0xD8U))
        {
          if (out_buf_size < 2U)
          {
            return MJPEG_READ_STATUS_FRAME_TOO_LARGE;
          }

          out_buf[0] = 0xFFU;
          out_buf[1] = 0xD8U;
          out_len = 2U;
          capture = 1U;
        }
      }
      else
      {
        if (out_len >= out_buf_size)
        {
          return MJPEG_READ_STATUS_FRAME_TOO_LARGE;
        }

        out_buf[out_len++] = byte;

        if ((prev == 0xFFU) && (byte == 0xD9U))
        {
          *frame_len = out_len;
          return MJPEG_READ_STATUS_OK;
        }
      }

      prev = byte;
      has_prev = 1U;
    }
  }

  if (capture != 0U)
  {
    return MJPEG_READ_STATUS_FORMAT_ERR;
  }

  return MJPEG_READ_STATUS_EOF;
}

static uint8_t mjpeg_avi_is_video_chunk(uint32_t fourcc)
{
  char c2;
  char c3;

  c2 = (char)((fourcc >> 16) & 0xFFU);
  c3 = (char)((fourcc >> 24) & 0xFFU);

  return (uint8_t)((c2 == 'd') && (c3 == 'c'));
}

static int32_t mjpeg_chunk_find_soi(const uint8_t *buf, uint32_t len)
{
  uint32_t i;

  if ((buf == NULL) || (len < 2U))
  {
    return -1;
  }

  for (i = 0U; (i + 1U) < len; ++i)
  {
    if ((buf[i] == 0xFFU) && (buf[i + 1U] == 0xD8U))
    {
      return (int32_t)i;
    }
  }

  return -1;
}

static uint8_t mjpeg_fourcc_is_mjpg(uint32_t fourcc)
{
  return (uint8_t)((fourcc == AVI_FCC_MJPG) || (fourcc == AVI_FCC_mjpg));
}

static uint8_t mjpeg_avi_get_chunk_stream_id(uint32_t fourcc, uint8_t *stream_id)
{
  uint8_t c0;
  uint8_t c1;

  if (stream_id == NULL)
  {
    return 0U;
  }

  c0 = (uint8_t)(fourcc & 0xFFU);
  c1 = (uint8_t)((fourcc >> 8U) & 0xFFU);
  if ((isdigit((int)c0) == 0) || (isdigit((int)c1) == 0))
  {
    return 0U;
  }

  *stream_id = (uint8_t)(((uint8_t)(c0 - (uint8_t)'0') * 10U) + (uint8_t)(c1 - (uint8_t)'0'));
  return 1U;
}

static int8_t mjpeg_avi_parse_strl(
  FIL *file,
  uint32_t strl_start,
  uint32_t strl_end,
  uint16_t *stream_width,
  uint16_t *stream_height,
  uint32_t *stream_handler,
  uint32_t *stream_compression,
  uint8_t *is_video
)
{
  uint8_t hdr[8];
  uint8_t strh_buf[16];
  uint8_t strf_buf[20];
  uint32_t pos;
  uint32_t fcc_type;
  uint32_t fcc_handler;
  uint32_t compression;
  uint16_t width;
  uint16_t height;

  if ((file == NULL) || (is_video == NULL) || (strl_end <= strl_start))
  {
    return MJPEG_PLAYER_ERR_FORMAT;
  }

  *is_video = 0U;
  if (stream_width != NULL)
  {
    *stream_width = 0U;
  }
  if (stream_height != NULL)
  {
    *stream_height = 0U;
  }
  if (stream_handler != NULL)
  {
    *stream_handler = 0U;
  }
  if (stream_compression != NULL)
  {
    *stream_compression = 0U;
  }

  pos = strl_start;
  fcc_type = 0U;
  fcc_handler = 0U;
  compression = 0U;
  width = 0U;
  height = 0U;

  while ((pos + 8U) <= strl_end)
  {
    uint32_t chunk_id;
    uint32_t chunk_size;
    uint32_t chunk_next;

    if (f_lseek(file, (FSIZE_t)pos) != FR_OK)
    {
      return MJPEG_PLAYER_ERR_IO;
    }

    if (mjpeg_read_exact(file, hdr, sizeof(hdr)) != MJPEG_READ_STATUS_OK)
    {
      return MJPEG_PLAYER_ERR_IO;
    }

    chunk_id = mjpeg_read_u32_le(&hdr[0]);
    chunk_size = mjpeg_read_u32_le(&hdr[4]);
    chunk_next = pos + 8U + chunk_size + (chunk_size & 1U);
    if (chunk_next < pos)
    {
      return MJPEG_PLAYER_ERR_FORMAT;
    }

    if (chunk_id == AVI_FCC_STRH)
    {
      uint32_t read_len;

      memset(strh_buf, 0, sizeof(strh_buf));
      read_len = (chunk_size < (uint32_t)sizeof(strh_buf)) ? chunk_size : (uint32_t)sizeof(strh_buf);
      if ((read_len > 0U) && (mjpeg_read_exact(file, strh_buf, read_len) != MJPEG_READ_STATUS_OK))
      {
        return MJPEG_PLAYER_ERR_IO;
      }

      if (read_len >= 8U)
      {
        fcc_type = mjpeg_read_u32_le(&strh_buf[0]);
        fcc_handler = mjpeg_read_u32_le(&strh_buf[4]);
      }
    }
    else if (chunk_id == AVI_FCC_STRF)
    {
      uint32_t read_len;

      memset(strf_buf, 0, sizeof(strf_buf));
      read_len = (chunk_size < (uint32_t)sizeof(strf_buf)) ? chunk_size : (uint32_t)sizeof(strf_buf);
      if ((read_len > 0U) && (mjpeg_read_exact(file, strf_buf, read_len) != MJPEG_READ_STATUS_OK))
      {
        return MJPEG_PLAYER_ERR_IO;
      }

      if (read_len >= 20U)
      {
        uint32_t w;
        uint32_t h;

        w = mjpeg_read_u32_le(&strf_buf[4]);
        h = mjpeg_read_u32_le(&strf_buf[8]);
        compression = mjpeg_read_u32_le(&strf_buf[16]);

        if ((w > 0U) && (w <= 0xFFFFU))
        {
          width = (uint16_t)w;
        }
        if ((h > 0U) && (h <= 0xFFFFU))
        {
          height = (uint16_t)h;
        }
      }
    }

    if (chunk_next > strl_end)
    {
      break;
    }

    pos = chunk_next;
  }

  if (fcc_type == AVI_FCC_VIDS)
  {
    *is_video = 1U;
    if (stream_width != NULL)
    {
      *stream_width = width;
    }
    if (stream_height != NULL)
    {
      *stream_height = height;
    }
    if (stream_handler != NULL)
    {
      *stream_handler = fcc_handler;
    }
    if (stream_compression != NULL)
    {
      *stream_compression = compression;
    }
  }

  return MJPEG_PLAYER_OK;
}

static int8_t mjpeg_avi_parse_hdrl(
  FIL *file,
  uint32_t hdrl_start,
  uint32_t hdrl_end,
  uint32_t *frame_interval_ms,
  uint16_t *avi_width,
  uint16_t *avi_height,
  uint8_t *video_stream_id
)
{
  uint8_t hdr[8];
  uint8_t avih_buf[40];
  uint32_t pos;
  uint8_t first_video_stream_id;
  uint8_t stream_id;

  if ((file == NULL) || (frame_interval_ms == NULL) || (hdrl_end <= hdrl_start))
  {
    return MJPEG_PLAYER_ERR_FORMAT;
  }

  first_video_stream_id = 0xFFU;
  stream_id = 0U;
  if (video_stream_id != NULL)
  {
    *video_stream_id = 0xFFU;
  }

  pos = hdrl_start;
  while ((pos + 8U) <= hdrl_end)
  {
    uint32_t chunk_id;
    uint32_t chunk_size;
    uint32_t chunk_next;

    if (f_lseek(file, (FSIZE_t)pos) != FR_OK)
    {
      return MJPEG_PLAYER_ERR_IO;
    }

    if (mjpeg_read_exact(file, hdr, sizeof(hdr)) != MJPEG_READ_STATUS_OK)
    {
      return MJPEG_PLAYER_ERR_IO;
    }

    chunk_id = mjpeg_read_u32_le(&hdr[0]);
    chunk_size = mjpeg_read_u32_le(&hdr[4]);
    chunk_next = pos + 8U + chunk_size + (chunk_size & 1U);

    if (chunk_next < pos)
    {
      return MJPEG_PLAYER_ERR_FORMAT;
    }

    if (chunk_id == AVI_FCC_AVIH)
    {
      uint32_t read_len;

      if (chunk_size < 4U)
      {
        return MJPEG_PLAYER_ERR_FORMAT;
      }

      memset(avih_buf, 0, sizeof(avih_buf));
      read_len = (chunk_size < (uint32_t)sizeof(avih_buf)) ? chunk_size : (uint32_t)sizeof(avih_buf);

      if (mjpeg_read_exact(file, avih_buf, read_len) != MJPEG_READ_STATUS_OK)
      {
        return MJPEG_PLAYER_ERR_IO;
      }

      {
        uint32_t usec_per_frame = mjpeg_read_u32_le(avih_buf);
        if (usec_per_frame > 0U)
        {
          uint32_t ms = (usec_per_frame + 500U) / 1000U;
          if (ms == 0U)
          {
            ms = 1U;
          }

          *frame_interval_ms = ms;
        }
      }

      if ((read_len >= 40U) && (avi_width != NULL) && (avi_height != NULL))
      {
        uint32_t width = mjpeg_read_u32_le(&avih_buf[32]);
        uint32_t height = mjpeg_read_u32_le(&avih_buf[36]);
        if ((width > 0U) && (width <= 0xFFFFU) && (height > 0U) && (height <= 0xFFFFU))
        {
          *avi_width = (uint16_t)width;
          *avi_height = (uint16_t)height;
        }
      }
    }
    else if ((chunk_id == AVI_FCC_LIST) && (chunk_size >= 4U))
    {
      uint8_t list_type_buf[4];

      if (mjpeg_read_exact(file, list_type_buf, sizeof(list_type_buf)) != MJPEG_READ_STATUS_OK)
      {
        return MJPEG_PLAYER_ERR_IO;
      }

      if (mjpeg_read_u32_le(list_type_buf) == AVI_FCC_STRL)
      {
        uint16_t stream_w;
        uint16_t stream_h;
        uint32_t stream_handler;
        uint32_t stream_compression;
        uint8_t is_video;

        stream_w = 0U;
        stream_h = 0U;
        stream_handler = 0U;
        stream_compression = 0U;
        is_video = 0U;
        (void)mjpeg_avi_parse_strl(
          file,
          pos + 12U,
          pos + 8U + chunk_size,
          &stream_w,
          &stream_h,
          &stream_handler,
          &stream_compression,
          &is_video
        );

        if (is_video != 0U)
        {
          if (first_video_stream_id == 0xFFU)
          {
            first_video_stream_id = stream_id;
          }

          if ((avi_width != NULL) && (avi_height != NULL) &&
              ((*avi_width == 0U) || (*avi_height == 0U)) &&
              (stream_w > 0U) && (stream_h > 0U))
          {
            *avi_width = stream_w;
            *avi_height = stream_h;
          }

          if ((video_stream_id != NULL) && (*video_stream_id == 0xFFU) &&
              ((mjpeg_fourcc_is_mjpg(stream_handler) != 0U) ||
               (mjpeg_fourcc_is_mjpg(stream_compression) != 0U)))
          {
            *video_stream_id = stream_id;
          }
        }

        if (stream_id < 99U)
        {
          stream_id++;
        }
      }
    }

    if (chunk_next > hdrl_end)
    {
      break;
    }

    pos = chunk_next;
  }

  if ((video_stream_id != NULL) && (*video_stream_id == 0xFFU) && (first_video_stream_id != 0xFFU))
  {
    *video_stream_id = first_video_stream_id;
  }

  if ((video_stream_id != NULL) && (*video_stream_id != 0xFFU))
  {
    mjpeg_log_value("MJPEG: AVI stream=", (int32_t)*video_stream_id);
  }

  return MJPEG_PLAYER_OK;
}

static int8_t mjpeg_avi_find_movi(
  FIL *file,
  uint32_t file_size,
  uint32_t *movi_start,
  uint32_t *movi_end,
  uint32_t *frame_interval_ms,
  uint16_t *avi_width,
  uint16_t *avi_height,
  uint8_t *video_stream_id
)
{
  uint8_t riff_hdr[12];
  uint8_t chunk_hdr[8];
  uint8_t list_type_buf[4];
  uint32_t pos;

  if ((file == NULL) || (movi_start == NULL) || (movi_end == NULL) || (frame_interval_ms == NULL))
  {
    return MJPEG_PLAYER_ERR_PARAM;
  }

  if (f_lseek(file, 0U) != FR_OK)
  {
    return MJPEG_PLAYER_ERR_IO;
  }

  if (mjpeg_read_exact(file, riff_hdr, sizeof(riff_hdr)) != MJPEG_READ_STATUS_OK)
  {
    return MJPEG_PLAYER_ERR_FORMAT;
  }

  if ((mjpeg_read_u32_le(&riff_hdr[0]) != AVI_FCC_RIFF) ||
      (mjpeg_read_u32_le(&riff_hdr[8]) != AVI_FCC_AVI))
  {
    return MJPEG_PLAYER_ERR_FORMAT;
  }

  pos = 12U;
  while ((pos + 8U) <= file_size)
  {
    uint32_t chunk_id;
    uint32_t chunk_size;
    uint32_t data_pos;
    uint32_t chunk_next;

    if (f_lseek(file, (FSIZE_t)pos) != FR_OK)
    {
      return MJPEG_PLAYER_ERR_IO;
    }

    if (mjpeg_read_exact(file, chunk_hdr, sizeof(chunk_hdr)) != MJPEG_READ_STATUS_OK)
    {
      return MJPEG_PLAYER_ERR_IO;
    }

    chunk_id = mjpeg_read_u32_le(&chunk_hdr[0]);
    chunk_size = mjpeg_read_u32_le(&chunk_hdr[4]);
    data_pos = pos + 8U;
    chunk_next = data_pos + chunk_size + (chunk_size & 1U);

    if ((chunk_next < pos) || (chunk_next > file_size))
    {
      return MJPEG_PLAYER_ERR_FORMAT;
    }

    if ((chunk_id == AVI_FCC_LIST) && (chunk_size >= 4U))
    {
      uint32_t list_type;

      if (mjpeg_read_exact(file, list_type_buf, sizeof(list_type_buf)) != MJPEG_READ_STATUS_OK)
      {
        return MJPEG_PLAYER_ERR_IO;
      }

      list_type = mjpeg_read_u32_le(list_type_buf);
      if (list_type == AVI_FCC_HDRL)
      {
        (void)mjpeg_avi_parse_hdrl(file, data_pos + 4U, data_pos + chunk_size, frame_interval_ms, avi_width, avi_height, video_stream_id);
      }
      else if (list_type == AVI_FCC_MOVI)
      {
        *movi_start = data_pos + 4U;
        *movi_end = data_pos + chunk_size;
        return MJPEG_PLAYER_OK;
      }
    }

    pos = chunk_next;
  }

  return MJPEG_PLAYER_ERR_FORMAT;
}

static void mjpeg_avi_iter_init(mjpeg_avi_iter_t *iter, uint32_t movi_end, uint8_t target_stream_id)
{
  if (iter == NULL)
  {
    return;
  }

  memset(iter, 0, sizeof(*iter));
  iter->end_pos_stack[0] = movi_end;
  iter->next_pos_stack[0] = movi_end;
  iter->depth = 0U;
  if (target_stream_id != 0xFFU)
  {
    iter->has_target_stream = 1U;
    iter->target_stream_id = target_stream_id;
  }
  else
  {
    iter->has_target_stream = 0U;
    iter->target_stream_id = 0U;
  }
}

static mjpeg_read_status_t mjpeg_avi_read_next_frame(
  FIL *file,
  mjpeg_avi_iter_t *iter,
  uint8_t *out_buf,
  uint32_t out_buf_size,
  uint32_t *frame_len
)
{
  uint8_t chunk_hdr[8];
  uint8_t list_type_buf[4];

  if ((file == NULL) || (iter == NULL) || (out_buf == NULL) || (frame_len == NULL))
  {
    return MJPEG_READ_STATUS_IO_ERR;
  }

  *frame_len = 0U;

  while (1)
  {
    uint32_t pos = (uint32_t)f_tell(file);
    uint32_t container_end = iter->end_pos_stack[iter->depth];

    if (pos >= container_end)
    {
      if (iter->depth == 0U)
      {
        return MJPEG_READ_STATUS_EOF;
      }

      {
        uint8_t done_depth = iter->depth;
        uint32_t resume_pos = iter->next_pos_stack[done_depth];

        iter->depth = (uint8_t)(done_depth - 1U);
        if (f_lseek(file, (FSIZE_t)resume_pos) != FR_OK)
        {
          return MJPEG_READ_STATUS_IO_ERR;
        }
      }

      continue;
    }

    if (mjpeg_read_exact(file, chunk_hdr, sizeof(chunk_hdr)) != MJPEG_READ_STATUS_OK)
    {
      return MJPEG_READ_STATUS_IO_ERR;
    }

    {
      uint32_t chunk_id = mjpeg_read_u32_le(&chunk_hdr[0]);
      uint32_t chunk_size = mjpeg_read_u32_le(&chunk_hdr[4]);
      uint32_t data_pos = pos + 8U;
      uint32_t chunk_end = data_pos + chunk_size;
      uint32_t chunk_next = chunk_end + (chunk_size & 1U);

      if ((chunk_end < data_pos) || (chunk_next < chunk_end) || (chunk_next > container_end))
      {
        return MJPEG_READ_STATUS_FORMAT_ERR;
      }

      if ((chunk_id == AVI_FCC_LIST) && (chunk_size >= 4U))
      {
        uint32_t list_type;

        if (mjpeg_read_exact(file, list_type_buf, sizeof(list_type_buf)) != MJPEG_READ_STATUS_OK)
        {
          return MJPEG_READ_STATUS_IO_ERR;
        }

        list_type = mjpeg_read_u32_le(list_type_buf);
        if ((list_type == AVI_FCC_REC) || (list_type == AVI_FCC_MOVI))
        {
          if ((iter->depth + 1U) >= MJPEG_ITER_AVI_STACK_DEPTH)
          {
            if (f_lseek(file, (FSIZE_t)chunk_next) != FR_OK)
            {
              return MJPEG_READ_STATUS_IO_ERR;
            }

            continue;
          }

          iter->depth++;
          iter->end_pos_stack[iter->depth] = chunk_end;
          iter->next_pos_stack[iter->depth] = chunk_next;
          if (f_lseek(file, (FSIZE_t)(data_pos + 4U)) != FR_OK)
          {
            return MJPEG_READ_STATUS_IO_ERR;
          }

          continue;
        }

        if (f_lseek(file, (FSIZE_t)chunk_next) != FR_OK)
        {
          return MJPEG_READ_STATUS_IO_ERR;
        }

        continue;
      }

      if (mjpeg_avi_is_video_chunk(chunk_id) != 0U)
      {
        int32_t soi_pos;

        if (iter->has_target_stream != 0U)
        {
          uint8_t stream_id;

          stream_id = 0U;
          if ((mjpeg_avi_get_chunk_stream_id(chunk_id, &stream_id) == 0U) ||
              (stream_id != iter->target_stream_id))
          {
            if (f_lseek(file, (FSIZE_t)chunk_next) != FR_OK)
            {
              return MJPEG_READ_STATUS_IO_ERR;
            }

            continue;
          }
        }

        if (chunk_size > out_buf_size)
        {
          if (f_lseek(file, (FSIZE_t)chunk_next) != FR_OK)
          {
            return MJPEG_READ_STATUS_IO_ERR;
          }

          return MJPEG_READ_STATUS_FRAME_TOO_LARGE;
        }

        if (mjpeg_read_exact(file, out_buf, chunk_size) != MJPEG_READ_STATUS_OK)
        {
          return MJPEG_READ_STATUS_IO_ERR;
        }

        if (f_lseek(file, (FSIZE_t)chunk_next) != FR_OK)
        {
          return MJPEG_READ_STATUS_IO_ERR;
        }

        soi_pos = mjpeg_chunk_find_soi(out_buf, chunk_size);
        if ((soi_pos < 0) || (soi_pos > 16))
        {
          continue;
        }

        if (soi_pos > 0)
        {
          memmove(out_buf, &out_buf[(uint32_t)soi_pos], chunk_size - (uint32_t)soi_pos);
          chunk_size -= (uint32_t)soi_pos;
        }

        *frame_len = chunk_size;
        return MJPEG_READ_STATUS_OK;
      }

      if (f_lseek(file, (FSIZE_t)chunk_next) != FR_OK)
      {
        return MJPEG_READ_STATUS_IO_ERR;
      }
    }
  }
}

static mjpeg_read_status_t mjpeg_avi_read_next_frame_by_tag(
  FIL *file,
  uint32_t movi_end,
  const char *video_tag,
  uint8_t *out_buf,
  uint32_t out_buf_size,
  uint32_t *frame_len
)
{
  uint8_t chunk_hdr[8];
  uint8_t list_type_buf[4];
  uint32_t pos;

  if ((file == NULL) || (out_buf == NULL) || (frame_len == NULL))
  {
    return MJPEG_READ_STATUS_IO_ERR;
  }

  *frame_len = 0U;
  pos = (uint32_t)f_tell(file);

  while ((pos + 8U) <= movi_end)
  {
    uint32_t chunk_id;
    uint32_t chunk_size;
    uint32_t data_pos;
    uint32_t chunk_end;
    uint32_t chunk_next;
    int32_t soi_pos;

    if (f_lseek(file, (FSIZE_t)pos) != FR_OK)
    {
      return MJPEG_READ_STATUS_IO_ERR;
    }

    if (mjpeg_read_exact(file, chunk_hdr, sizeof(chunk_hdr)) != MJPEG_READ_STATUS_OK)
    {
      return MJPEG_READ_STATUS_IO_ERR;
    }

    chunk_id = mjpeg_read_u32_le(&chunk_hdr[0]);
    chunk_size = mjpeg_read_u32_le(&chunk_hdr[4]);
    data_pos = pos + 8U;
    chunk_end = data_pos + chunk_size;
    chunk_next = chunk_end + (chunk_size & 1U);

    if ((chunk_end < data_pos) || (chunk_next < chunk_end) || (chunk_next > movi_end))
    {
      pos += 1U;
      continue;
    }

    if ((chunk_id == AVI_FCC_LIST) && (chunk_size >= 4U))
    {
      uint32_t list_type;

      if (mjpeg_read_exact(file, list_type_buf, sizeof(list_type_buf)) != MJPEG_READ_STATUS_OK)
      {
        return MJPEG_READ_STATUS_IO_ERR;
      }

      list_type = mjpeg_read_u32_le(list_type_buf);
      if ((list_type == AVI_FCC_REC) || (list_type == AVI_FCC_MOVI))
      {
        pos = data_pos + 4U;
      }
      else
      {
        pos = chunk_next;
      }

      continue;
    }

    if (mjpeg_chunk_tag_match(chunk_hdr, video_tag) == 0U)
    {
      pos = chunk_next;
      continue;
    }

    if (chunk_size > out_buf_size)
    {
      if (f_lseek(file, (FSIZE_t)chunk_next) != FR_OK)
      {
        return MJPEG_READ_STATUS_IO_ERR;
      }

      return MJPEG_READ_STATUS_FRAME_TOO_LARGE;
    }

    if (f_lseek(file, (FSIZE_t)data_pos) != FR_OK)
    {
      return MJPEG_READ_STATUS_IO_ERR;
    }

    if (mjpeg_read_exact(file, out_buf, chunk_size) != MJPEG_READ_STATUS_OK)
    {
      return MJPEG_READ_STATUS_IO_ERR;
    }

    if (f_lseek(file, (FSIZE_t)chunk_next) != FR_OK)
    {
      return MJPEG_READ_STATUS_IO_ERR;
    }

    soi_pos = mjpeg_chunk_find_soi(out_buf, chunk_size);
    if ((soi_pos < 0) || (soi_pos > 16))
    {
      pos = chunk_next;
      continue;
    }

    if (soi_pos > 0)
    {
      memmove(out_buf, &out_buf[(uint32_t)soi_pos], chunk_size - (uint32_t)soi_pos);
      chunk_size -= (uint32_t)soi_pos;
    }

    *frame_len = chunk_size;
    return MJPEG_READ_STATUS_OK;
  }

  return MJPEG_READ_STATUS_EOF;
}

static int8_t mjpeg_avi_get_streaminfo(const uint8_t *hdr8, uint16_t *stream_id, uint32_t *stream_size, uint8_t *stream_no)
{
  uint8_t c0;
  uint8_t c1;

  if ((hdr8 == NULL) || (stream_id == NULL) || (stream_size == NULL))
  {
    return MJPEG_PLAYER_ERR_PARAM;
  }

  if (stream_no != NULL)
  {
    *stream_no = 0xFFU;
  }

  c0 = hdr8[0];
  c1 = hdr8[1];
  if ((isdigit((int)c0) != 0) && (isdigit((int)c1) != 0) && (stream_no != NULL))
  {
    *stream_no = (uint8_t)(((uint8_t)(c0 - (uint8_t)'0') * 10U) + (uint8_t)(c1 - (uint8_t)'0'));
  }

  *stream_id = (uint16_t)(((uint16_t)hdr8[2] << 8U) | (uint16_t)hdr8[3]);
  *stream_size = mjpeg_read_u32_le(&hdr8[4]);

  if (*stream_size > MJPEG_PLAYER_AVI_MAX_STREAM_SIZE)
  {
    mjpeg_log_value("MJPEG: stream size over=", (int32_t)*stream_size);
    return MJPEG_PLAYER_ERR_FORMAT;
  }

  if ((*stream_size & 1U) != 0U)
  {
    (*stream_size)++;
  }

  if ((*stream_id == AVI_VIDS_FLAG) || (*stream_id == AVI_AUDS_FLAG))
  {
    return MJPEG_PLAYER_OK;
  }

  return MJPEG_PLAYER_ERR_FORMAT;
}

int8_t MJPEG_Player_PlayFile(const char *file_path)
{
  FIL file;
  FRESULT fr;
  uint8_t probe_hdr[12];
  mjpeg_container_type_t container_type;
  uint32_t frame_interval_ms;
  uint32_t frame_len;
  int8_t status;
  uint8_t stream_hdr[8];
  uint8_t next_stream_hdr[8];
  uint16_t stream_id;
  uint32_t stream_size;
  uint8_t stream_no;

  uint32_t avi_movi_start = 0U;
  uint32_t avi_movi_end = 0U;
  uint16_t avi_width = 0U;
  uint16_t avi_height = 0U;
  uint8_t avi_video_stream_id = 0xFFU;
  uint8_t avi_video_stream_primary = 0xFFU;
  uint32_t stream_filter_skip_log_count = 0U;
  uint16_t stream_eval_seen = 0U;
  uint16_t stream_eval_ok = 0U;
  uint8_t stream_auto_switch_done = 0U;

  if (s_mjpeg_jpeg_tables_ready == 0U)
  {
    JPEG_InitColorTables();
    s_mjpeg_jpeg_tables_ready = 1U;
  }

  if ((file_path == NULL) || (file_path[0] == '\0'))
  {
    return MJPEG_PLAYER_ERR_PARAM;
  }

  fr = f_mount(&SDFatFS, (TCHAR const *)SDPath, 1U);
  if (fr != FR_OK)
  {
    return MJPEG_PLAYER_ERR_MOUNT;
  }

  fr = f_open(&file, file_path, FA_READ);
  if (fr != FR_OK)
  {
    (void)f_mount(NULL, (TCHAR const *)SDPath, 1U);
    return MJPEG_PLAYER_ERR_FILE;
  }

  frame_interval_ms = MJPEG_PLAYER_DEFAULT_FRAME_MS;
  status = MJPEG_PLAYER_OK;
  frame_len = 0U;

  if (mjpeg_read_exact(&file, probe_hdr, sizeof(probe_hdr)) != MJPEG_READ_STATUS_OK)
  {
    status = MJPEG_PLAYER_ERR_FORMAT;
    goto cleanup;
  }

  if ((mjpeg_read_u32_le(&probe_hdr[0]) == AVI_FCC_RIFF) &&
      (mjpeg_read_u32_le(&probe_hdr[8]) == AVI_FCC_AVI))
  {
    container_type = MJPEG_CONTAINER_AVI;
  }
  else if (mjpeg_is_ext(file_path, "AVI") != 0U)
  {
    container_type = MJPEG_CONTAINER_AVI;
  }
  else
  {
    container_type = MJPEG_CONTAINER_RAW;
  }

  if (container_type == MJPEG_CONTAINER_AVI)
  {
    status = mjpeg_avi_find_movi(
      &file,
      (uint32_t)f_size(&file),
      &avi_movi_start,
      &avi_movi_end,
      &frame_interval_ms,
      &avi_width,
      &avi_height,
      &avi_video_stream_id
    );
    if (status != MJPEG_PLAYER_OK)
    {
      goto cleanup;
    }

    if (f_lseek(&file, (FSIZE_t)avi_movi_start) != FR_OK)
    {
      status = MJPEG_PLAYER_ERR_IO;
      goto cleanup;
    }

    if ((avi_width > 0U) && (avi_height > 0U))
    {
      mjpeg_log_info("MJPEG: AVI wh", (uint32_t)avi_width, (uint32_t)avi_height);
    }

    if (avi_video_stream_id != 0xFFU)
    {
      mjpeg_log_value("MJPEG: AVI stream=", (int32_t)avi_video_stream_id);
      avi_video_stream_primary = avi_video_stream_id;
    }

    if (mjpeg_read_exact(&file, stream_hdr, sizeof(stream_hdr)) != MJPEG_READ_STATUS_OK)
    {
      status = MJPEG_PLAYER_ERR_IO;
      goto cleanup;
    }

    status = mjpeg_avi_get_streaminfo(stream_hdr, &stream_id, &stream_size, &stream_no);
    if (status != MJPEG_PLAYER_OK)
    {
      status = MJPEG_PLAYER_ERR_FORMAT;
      goto cleanup;
    }
  }
  else
  {
    if (f_lseek(&file, 0U) != FR_OK)
    {
      status = MJPEG_PLAYER_ERR_IO;
      goto cleanup;
    }
  }

  if (frame_interval_ms == 0U)
  {
    frame_interval_ms = 1U;
  }

  if (MJPEG_Scheduler_SetFrameIntervalMs(frame_interval_ms) != HAL_OK)
  {
    status = MJPEG_PLAYER_ERR_IO;
    goto cleanup;
  }

  LCD_SetBackColor(LCD_BLACK);
  LCD_Clear();

  while (status == MJPEG_PLAYER_OK)
  {
    if (mjpeg_stop_requested() != 0U)
    {
      status = MJPEG_PLAYER_ERR_STOPPED;
      break;
    }

    if (container_type == MJPEG_CONTAINER_AVI)
    {
      uint32_t cur_pos;
      uint32_t frame_payload_plus_header;

      cur_pos = (uint32_t)f_tell(&file);
      frame_payload_plus_header = stream_size + 8U;

      if ((cur_pos > avi_movi_end) || (frame_payload_plus_header > (avi_movi_end - cur_pos)))
      {
        status = MJPEG_PLAYER_OK;
        break;
      }

      if (stream_id == AVI_VIDS_FLAG)
      {
        int8_t frame_status;

        if (stream_filter_skip_log_count == 0U)
        {
          mjpeg_log_value("MJPEG: avi frame stream=", (int32_t)stream_no);
          mjpeg_log_value("MJPEG: avi frame len=", (int32_t)stream_size);
        }

        if (frame_payload_plus_header > (uint32_t)sizeof(s_mjpeg_ycbcr_buffer))
        {
          status = MJPEG_PLAYER_ERR_FRAME_TOO_LARGE;
          break;
        }

        if (mjpeg_read_exact(&file, s_mjpeg_frame_io_buffer, frame_payload_plus_header) != MJPEG_READ_STATUS_OK)
        {
          mjpeg_log("MJPEG: avi frame read fail\r\n");
          status = MJPEG_PLAYER_ERR_IO;
          break;
        }

        memcpy(next_stream_hdr, &s_mjpeg_frame_io_buffer[stream_size], sizeof(next_stream_hdr));

        if ((avi_video_stream_id != 0xFFU) && (stream_no != 0xFFU) && (stream_no != avi_video_stream_id))
        {
          stream_filter_skip_log_count++;
          if ((stream_filter_skip_log_count <= 6U) || ((stream_filter_skip_log_count & 0x1FU) == 0U))
          {
            mjpeg_log_value("MJPEG: skip stream=", (int32_t)stream_no);
          }

          if ((stream_auto_switch_done == 1U) && (avi_video_stream_primary != 0xFFU) && (stream_filter_skip_log_count >= 64U))
          {
            avi_video_stream_id = avi_video_stream_primary;
            stream_auto_switch_done = 2U;
            stream_filter_skip_log_count = 0U;
            stream_eval_seen = 0U;
            stream_eval_ok = 0U;
            mjpeg_log_value("MJPEG: switch stream back->", (int32_t)avi_video_stream_id);
          }

          status = mjpeg_avi_get_streaminfo(next_stream_hdr, &stream_id, &stream_size, &stream_no);
          if (status != MJPEG_PLAYER_OK)
          {
            status = MJPEG_PLAYER_ERR_FORMAT;
            break;
          }

          status = MJPEG_PLAYER_OK;
          continue;
        }

        frame_len = stream_size;
        memcpy(s_mjpeg_ycbcr_buffer, s_mjpeg_frame_io_buffer, frame_len);
        frame_status = mjpeg_decode_one_frame(
          s_mjpeg_ycbcr_buffer,
          frame_len,
          avi_width,
          avi_height
        );

        if ((frame_status == MJPEG_PLAYER_ERR_DECODE) ||
            (frame_status == MJPEG_PLAYER_ERR_UNSUPPORTED) ||
            (frame_status == MJPEG_PLAYER_ERR_FORMAT))
        {
          s_mjpeg_skip_log_count++;
          if ((s_mjpeg_skip_log_count <= 6U) || ((s_mjpeg_skip_log_count & 0x7U) == 0U))
          {
            mjpeg_log_value("MJPEG: frame skipped err=", (int32_t)frame_status);
          }
        }
        else if (frame_status != MJPEG_PLAYER_OK)
        {
          status = frame_status;
          break;
        }

        if ((avi_video_stream_id != 0xFFU) && (stream_no == avi_video_stream_id))
        {
          stream_eval_seen++;
          if (frame_status == MJPEG_PLAYER_OK)
          {
            stream_eval_ok++;
          }

          if (stream_eval_seen >= MJPEG_PLAYER_STREAM_EVAL_FRAMES)
          {
            if ((stream_eval_ok * 2U) < stream_eval_seen)
            {
              if ((stream_auto_switch_done == 0U) && (avi_video_stream_primary != 0xFFU))
              {
                uint8_t new_stream = (avi_video_stream_id == 0U) ? 1U : 0U;
                mjpeg_log_value("MJPEG: switch stream->", (int32_t)new_stream);
                avi_video_stream_id = new_stream;
                stream_auto_switch_done = 1U;
                stream_filter_skip_log_count = 0U;
              }
            }

            stream_eval_seen = 0U;
            stream_eval_ok = 0U;
          }
        }

        status = mjpeg_avi_get_streaminfo(next_stream_hdr, &stream_id, &stream_size, &stream_no);
        if (status != MJPEG_PLAYER_OK)
        {
          status = MJPEG_PLAYER_ERR_FORMAT;
          break;
        }

        status = mjpeg_wait_next_frame_tick(frame_interval_ms);
        continue;
      }
      else if (stream_id == AVI_AUDS_FLAG)
      {
        if (frame_payload_plus_header <= (uint32_t)sizeof(s_mjpeg_ycbcr_buffer))
        {
          if (mjpeg_read_exact(&file, s_mjpeg_frame_io_buffer, frame_payload_plus_header) != MJPEG_READ_STATUS_OK)
          {
            mjpeg_log("MJPEG: avi frame read fail\r\n");
            status = MJPEG_PLAYER_ERR_IO;
            break;
          }

          memcpy(next_stream_hdr, &s_mjpeg_frame_io_buffer[stream_size], sizeof(next_stream_hdr));


          status = mjpeg_avi_get_streaminfo(next_stream_hdr, &stream_id, &stream_size, &stream_no);
          if (status != MJPEG_PLAYER_OK)
          {
            status = MJPEG_PLAYER_ERR_FORMAT;
            break;
          }
        }
        else
        {
          if (f_lseek(&file, (FSIZE_t)(cur_pos + stream_size)) != FR_OK)
          {
            status = MJPEG_PLAYER_ERR_IO;
            break;
          }

          if (mjpeg_read_exact(&file, stream_hdr, sizeof(stream_hdr)) != MJPEG_READ_STATUS_OK)
          {
            status = MJPEG_PLAYER_ERR_IO;
            break;
          }

          status = mjpeg_avi_get_streaminfo(stream_hdr, &stream_id, &stream_size, &stream_no);
          if (status != MJPEG_PLAYER_OK)
          {
            status = MJPEG_PLAYER_ERR_FORMAT;
            break;
          }
        }

        status = MJPEG_PLAYER_OK;
        continue;
      }

      status = MJPEG_PLAYER_ERR_FORMAT;
      break;
    }
    else
    {
      int8_t frame_status;
      mjpeg_read_status_t read_status;

      read_status = mjpeg_raw_read_next_frame(
        &file,
        s_mjpeg_frame_io_buffer,
        sizeof(s_mjpeg_frame_io_buffer),
        &frame_len
      );

      if (read_status == MJPEG_READ_STATUS_EOF)
      {
        status = MJPEG_PLAYER_OK;
        break;
      }

      if (read_status == MJPEG_READ_STATUS_FRAME_TOO_LARGE)
      {
        status = MJPEG_PLAYER_ERR_FRAME_TOO_LARGE;
        break;
      }

      if (read_status == MJPEG_READ_STATUS_FORMAT_ERR)
      {
        status = MJPEG_PLAYER_ERR_FORMAT;
        break;
      }

      if (read_status != MJPEG_READ_STATUS_OK)
      {
        status = MJPEG_PLAYER_ERR_IO;
        break;
      }

      memcpy(s_mjpeg_ycbcr_buffer, s_mjpeg_frame_io_buffer, frame_len);
      frame_status = mjpeg_decode_one_frame(
        s_mjpeg_ycbcr_buffer,
        frame_len,
        avi_width,
        avi_height
      );

      if (frame_status == MJPEG_PLAYER_OK)
      {
        s_mjpeg_skip_log_count = 0U;
      }
      else if ((frame_status == MJPEG_PLAYER_ERR_DECODE) ||
               (frame_status == MJPEG_PLAYER_ERR_UNSUPPORTED) ||
               (frame_status == MJPEG_PLAYER_ERR_FORMAT))
      {
        s_mjpeg_skip_log_count++;
        if ((s_mjpeg_skip_log_count <= 6U) || ((s_mjpeg_skip_log_count & 0x7U) == 0U))
        {
          mjpeg_log_value("MJPEG: frame skipped err=", (int32_t)frame_status);
        }
      }
      else
      {
        status = frame_status;
        break;
      }

      status = mjpeg_wait_next_frame_tick(frame_interval_ms);
    }
  }

cleanup:
  (void)f_close(&file);
  (void)f_mount(NULL, (TCHAR const *)SDPath, 1U);
  return status;
}