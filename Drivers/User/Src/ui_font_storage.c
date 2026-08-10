#include "ui_font_storage.h"

#include "fatfs.h"
#include "qspi_partition.h"
#include "qspi_w25q64.h"
#include "ui_font_asset.h"
#include "src/font/lv_font_fmt_txt.h"

#include <stddef.h>
#include <string.h>

#define UI_FONT_PACKAGE_HEADER_SIZE       32U
#define UI_FONT_INSTALL_PATH              "0:/GB2312.FNT"
#define UI_FONT_IO_CHUNK                  4096U
#define UI_FONT_CACHE_SLOTS               8U
#define UI_FONT_CACHE_GLYPH_BYTES         128U
#define UI_FONT_MAP_RETRY_MS              1000U

typedef struct
{
  uint32_t unicode;
  uint32_t bitmap_offset;
  uint16_t size;
  uint8_t valid;
  uint8_t data[UI_FONT_CACHE_GLYPH_BYTES];
} UI_FontGlyphCache;

_Static_assert((UI_FONT_PACKAGE_HEADER_SIZE + UI_FONT_ASSET_BITMAP_SIZE) <=
                 QSPI_PARTITION_UI_FONT_SIZE,
               "GB2312 font package exceeds the W25Q64 partition");

static uint8_t s_font_ready;
static uint8_t s_cache_next;
static uint32_t s_map_retry_tick;
static UI_FontGlyphCache s_glyph_cache[UI_FONT_CACHE_SLOTS];
static uint8_t s_io_buffer[UI_FONT_IO_CHUNK]
  __attribute__((section(".ram_d2"), aligned(32)));

static uint16_t ui_font_read_u16_le(const uint8_t *data)
{
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static uint32_t ui_font_read_u32_le(const uint8_t *data)
{
  return (uint32_t)data[0] |
         ((uint32_t)data[1] << 8U) |
         ((uint32_t)data[2] << 16U) |
         ((uint32_t)data[3] << 24U);
}

static uint32_t ui_font_crc32_update(uint32_t crc,
                                     const uint8_t *data,
                                     uint32_t size)
{
  uint32_t index;
  uint32_t bit;

  for (index = 0U; index < size; ++index)
  {
    crc ^= data[index];
    for (bit = 0U; bit < 8U; ++bit)
    {
      crc = ((crc & 1U) != 0U) ?
              ((crc >> 1U) ^ 0xEDB88320U) : (crc >> 1U);
    }
  }
  return crc;
}

static uint8_t ui_font_header_valid(const uint8_t *header)
{
  return ((ui_font_read_u32_le(&header[0]) == UI_FONT_ASSET_MAGIC) &&
          (ui_font_read_u16_le(&header[4]) == UI_FONT_ASSET_VERSION) &&
          (ui_font_read_u16_le(&header[6]) == UI_FONT_PACKAGE_HEADER_SIZE) &&
          (ui_font_read_u32_le(&header[8]) == UI_FONT_ASSET_BITMAP_SIZE) &&
          (ui_font_read_u32_le(&header[12]) == UI_FONT_ASSET_BITMAP_CRC32) &&
          (ui_font_read_u32_le(&header[16]) == UI_FONT_ASSET_GLYPH_COUNT) &&
          (ui_font_read_u16_le(&header[20]) == UI_FONT_ASSET_PIXEL_SIZE) &&
          (header[22] == UI_FONT_ASSET_BPP)) ? 1U : 0U;
}

static void ui_font_cache_reset(void)
{
  memset(s_glyph_cache, 0, sizeof(s_glyph_cache));
  s_cache_next = 0U;
}

static int8_t ui_font_read_flash(uint8_t *buffer,
                                 uint32_t address,
                                 uint32_t size)
{
  if ((buffer == NULL) || (size == 0U) ||
      (address < QSPI_PARTITION_UI_FONT_OFFSET) ||
      ((address + size) > QSPI_PARTITION_UI_FONT_END))
  {
    return UI_FONT_STORAGE_ERR_PACKAGE;
  }

  if (QSPI_W25Qxx_IsMemoryMapped() != 0U)
  {
    memcpy(buffer,
           (const void *)(uintptr_t)(W25QXX_MEM_MAPPED_ADDR + address),
           size);
    return UI_FONT_STORAGE_OK;
  }

  return (QSPI_W25Qxx_ReadBuffer(buffer, address, size) ==
          QSPI_W25QXX_OK) ? UI_FONT_STORAGE_OK :
                            UI_FONT_STORAGE_ERR_QSPI;
}

static int8_t ui_font_validate_flash_payload(void)
{
  uint32_t address = QSPI_PARTITION_UI_FONT_OFFSET +
                     UI_FONT_PACKAGE_HEADER_SIZE;
  uint32_t remaining = UI_FONT_ASSET_BITMAP_SIZE;
  uint32_t chunk;
  uint32_t crc = 0xFFFFFFFFU;
  int8_t result;

  while (remaining != 0U)
  {
    chunk = (remaining > sizeof(s_io_buffer)) ? sizeof(s_io_buffer) :
                                                remaining;
    result = ui_font_read_flash(s_io_buffer, address, chunk);
    if (result != UI_FONT_STORAGE_OK)
    {
      return result;
    }

    crc = ui_font_crc32_update(crc, s_io_buffer, chunk);
    address += chunk;
    remaining -= chunk;
  }

  return ((crc ^ 0xFFFFFFFFU) == UI_FONT_ASSET_BITMAP_CRC32) ?
           UI_FONT_STORAGE_OK : UI_FONT_STORAGE_ERR_CRC;
}

int8_t UI_FontStorage_Init(void)
{
  uint8_t header[UI_FONT_PACKAGE_HEADER_SIZE];
  int8_t result;

  s_font_ready = 0U;
  s_map_retry_tick = 0U;
  ui_font_cache_reset();

  result = ui_font_read_flash(header, QSPI_PARTITION_UI_FONT_OFFSET,
                              sizeof(header));
  if (result != UI_FONT_STORAGE_OK)
  {
    return result;
  }

  if (ui_font_header_valid(header) == 0U)
  {
    return UI_FONT_STORAGE_ERR_NOT_READY;
  }

  /* The header is committed last during installation, but a full CRC check
   * also detects later corruption or an interrupted update whose old header
   * happened to survive.  This runs only once during boot. */
  result = ui_font_validate_flash_payload();
  if (result != UI_FONT_STORAGE_OK)
  {
    return result;
  }

  s_font_ready = 1U;
  return UI_FONT_STORAGE_OK;
}

int8_t UI_FontStorage_InstallFromSd(const char *path)
{
  FIL file;
  FRESULT fr;
  UINT bytes_read;
  uint8_t header[UI_FONT_PACKAGE_HEADER_SIZE];
  uint32_t remaining;
  uint32_t address;
  uint32_t chunk;
  uint32_t crc;
  uint32_t erase_address;
  uint32_t erase_end;
  uint8_t file_open = 0U;
  uint8_t mounted = 0U;
  int8_t result = UI_FONT_STORAGE_ERR_SD;

  if (path == NULL)
  {
    return UI_FONT_STORAGE_ERR_PACKAGE;
  }

  s_font_ready = 0U;
  ui_font_cache_reset();

  fr = f_mount(&SDFatFS, (TCHAR const *)SDPath, 1U);
  if (fr != FR_OK)
  {
    return UI_FONT_STORAGE_ERR_SD;
  }
  mounted = 1U;

  fr = f_open(&file, path, FA_READ);
  if (fr != FR_OK)
  {
    goto cleanup;
  }
  file_open = 1U;

  fr = f_read(&file, header, sizeof(header), &bytes_read);
  if ((fr != FR_OK) || (bytes_read != sizeof(header)) ||
      (ui_font_header_valid(header) == 0U) ||
      (f_size(&file) != (FSIZE_t)(UI_FONT_PACKAGE_HEADER_SIZE +
                                  UI_FONT_ASSET_BITMAP_SIZE)))
  {
    result = UI_FONT_STORAGE_ERR_PACKAGE;
    goto cleanup;
  }

  if ((UI_FONT_PACKAGE_HEADER_SIZE + UI_FONT_ASSET_BITMAP_SIZE) >
      QSPI_PARTITION_UI_FONT_SIZE)
  {
    result = UI_FONT_STORAGE_ERR_PACKAGE;
    goto cleanup;
  }

  if (QSPI_W25Qxx_ExitMemoryMappedMode() != QSPI_W25QXX_OK)
  {
    result = UI_FONT_STORAGE_ERR_QSPI;
    goto cleanup;
  }

  /* The header is committed last.  A reset or power loss during installation
   * therefore leaves the package invalid instead of exposing partial glyphs. */
  erase_end = (QSPI_PARTITION_UI_FONT_OFFSET +
               UI_FONT_PACKAGE_HEADER_SIZE +
               UI_FONT_ASSET_BITMAP_SIZE + 0xFFFFU) & ~0xFFFFU;
  for (erase_address = QSPI_PARTITION_UI_FONT_OFFSET;
       erase_address < erase_end;
       erase_address += 0x10000U)
  {
    if (QSPI_W25Qxx_BlockErase_64K(erase_address) != QSPI_W25QXX_OK)
    {
      result = UI_FONT_STORAGE_ERR_QSPI;
      goto cleanup;
    }
  }

  remaining = UI_FONT_ASSET_BITMAP_SIZE;
  address = QSPI_PARTITION_UI_FONT_OFFSET + UI_FONT_PACKAGE_HEADER_SIZE;
  crc = 0xFFFFFFFFU;
  while (remaining != 0U)
  {
    chunk = (remaining > sizeof(s_io_buffer)) ? sizeof(s_io_buffer) : remaining;
    fr = f_read(&file, s_io_buffer, chunk, &bytes_read);
    if ((fr != FR_OK) || (bytes_read != chunk))
    {
      result = UI_FONT_STORAGE_ERR_SD;
      goto cleanup;
    }

    crc = ui_font_crc32_update(crc, s_io_buffer, chunk);
    if (QSPI_W25Qxx_WriteBuffer(s_io_buffer, address, chunk) !=
        QSPI_W25QXX_OK)
    {
      result = UI_FONT_STORAGE_ERR_QSPI;
      goto cleanup;
    }
    address += chunk;
    remaining -= chunk;
  }

  if ((crc ^ 0xFFFFFFFFU) != UI_FONT_ASSET_BITMAP_CRC32)
  {
    result = UI_FONT_STORAGE_ERR_CRC;
    goto cleanup;
  }

  /* Read back the programmed payload before committing its header. */
  remaining = UI_FONT_ASSET_BITMAP_SIZE;
  address = QSPI_PARTITION_UI_FONT_OFFSET + UI_FONT_PACKAGE_HEADER_SIZE;
  crc = 0xFFFFFFFFU;
  while (remaining != 0U)
  {
    chunk = (remaining > sizeof(s_io_buffer)) ? sizeof(s_io_buffer) : remaining;
    if (QSPI_W25Qxx_ReadBuffer(s_io_buffer, address, chunk) !=
        QSPI_W25QXX_OK)
    {
      result = UI_FONT_STORAGE_ERR_QSPI;
      goto cleanup;
    }
    crc = ui_font_crc32_update(crc, s_io_buffer, chunk);
    address += chunk;
    remaining -= chunk;
  }

  if ((crc ^ 0xFFFFFFFFU) != UI_FONT_ASSET_BITMAP_CRC32)
  {
    result = UI_FONT_STORAGE_ERR_CRC;
    goto cleanup;
  }

  if (QSPI_W25Qxx_WriteBuffer(header,
                              QSPI_PARTITION_UI_FONT_OFFSET,
                              sizeof(header)) != QSPI_W25QXX_OK)
  {
    result = UI_FONT_STORAGE_ERR_QSPI;
    goto cleanup;
  }

  result = UI_FONT_STORAGE_OK;

cleanup:
  if (file_open != 0U)
  {
    (void)f_close(&file);
  }
  if (mounted != 0U)
  {
    (void)f_mount(NULL, (TCHAR const *)SDPath, 1U);
  }
  return result;
}

int8_t UI_FontStorage_Bootstrap(void)
{
  int8_t result = UI_FontStorage_Init();

  if (result == UI_FONT_STORAGE_OK)
  {
    return result;
  }

  result = UI_FontStorage_InstallFromSd(UI_FONT_INSTALL_PATH);
  if (result != UI_FONT_STORAGE_OK)
  {
    return result;
  }
  return UI_FontStorage_Init();
}

uint8_t UI_FontStorage_IsReady(void)
{
  return s_font_ready;
}

void UI_FontStorage_MaintainMappedRead(void)
{
  uint32_t now;

  if ((s_font_ready == 0U) ||
      (QSPI_W25Qxx_IsMemoryMapped() != 0U))
  {
    return;
  }

  now = HAL_GetTick();
  if ((s_map_retry_tick != 0U) &&
      ((int32_t)(now - s_map_retry_tick) < 0))
  {
    return;
  }

  if (QSPI_W25Qxx_MemoryMappedMode() == QSPI_W25QXX_OK)
  {
    s_map_retry_tick = 0U;
  }
  else
  {
    s_map_retry_tick = now + UI_FONT_MAP_RETRY_MS;
    if (s_map_retry_tick == 0U)
    {
      s_map_retry_tick = 1U;
    }
  }
}

bool UI_FontStorage_GetGlyphDsc(const lv_font_t *font,
                               lv_font_glyph_dsc_t *dsc_out,
                               uint32_t unicode_letter,
                               uint32_t unicode_letter_next)
{
  if ((s_font_ready == 0U) || (font == NULL) || (dsc_out == NULL))
  {
    return false;
  }

  return lv_font_get_glyph_dsc_fmt_txt(font, dsc_out,
                                        unicode_letter,
                                        unicode_letter_next);
}

const uint8_t *UI_FontStorage_GetBitmap(const lv_font_t *font,
                                        uint32_t unicode_letter)
{
  lv_font_fmt_txt_dsc_t *font_dsc;
  const lv_font_fmt_txt_glyph_dsc_t *glyph_dsc;
  lv_font_glyph_dsc_t public_dsc;
  UI_FontGlyphCache *slot;
  uint32_t glyph_id;
  uint32_t bitmap_size;
  uint32_t index;
  uint32_t flash_address;

  if ((s_font_ready == 0U) || (font == NULL))
  {
    return NULL;
  }

  font_dsc = (lv_font_fmt_txt_dsc_t *)font->dsc;
  if ((font_dsc == NULL) || (font_dsc->cache == NULL) ||
      (UI_FontStorage_GetGlyphDsc(font, &public_dsc,
                                 unicode_letter, 0U) == false))
  {
    return NULL;
  }

  glyph_id = font_dsc->cache->last_glyph_id;
  if (glyph_id == 0U)
  {
    return NULL;
  }
  glyph_dsc = &font_dsc->glyph_dsc[glyph_id];
  bitmap_size = (((uint32_t)glyph_dsc->box_w *
                  (uint32_t)glyph_dsc->box_h *
                  (uint32_t)font_dsc->bpp) + 7U) >> 3U;
  if (bitmap_size == 0U)
  {
    return NULL;
  }
  if ((bitmap_size > UI_FONT_CACHE_GLYPH_BYTES) ||
      ((glyph_dsc->bitmap_index + bitmap_size) >
       UI_FONT_ASSET_BITMAP_SIZE))
  {
    return NULL;
  }

  for (index = 0U; index < UI_FONT_CACHE_SLOTS; ++index)
  {
    slot = &s_glyph_cache[index];
    if ((slot->valid != 0U) &&
        (slot->unicode == unicode_letter) &&
        (slot->bitmap_offset == glyph_dsc->bitmap_index) &&
        (slot->size == bitmap_size))
    {
      return slot->data;
    }
  }

  slot = &s_glyph_cache[s_cache_next];
  s_cache_next = (uint8_t)((s_cache_next + 1U) % UI_FONT_CACHE_SLOTS);
  flash_address = QSPI_PARTITION_UI_FONT_OFFSET +
                  UI_FONT_PACKAGE_HEADER_SIZE + glyph_dsc->bitmap_index;

  if (QSPI_W25Qxx_IsMemoryMapped() != 0U)
  {
    memcpy(slot->data,
           (const void *)(uintptr_t)(W25QXX_MEM_MAPPED_ADDR + flash_address),
           bitmap_size);
  }
  else if (QSPI_W25Qxx_ReadBuffer(slot->data, flash_address, bitmap_size) !=
           QSPI_W25QXX_OK)
  {
    slot->valid = 0U;
    return NULL;
  }

  slot->unicode = unicode_letter;
  slot->bitmap_offset = glyph_dsc->bitmap_index;
  slot->size = (uint16_t)bitmap_size;
  slot->valid = 1U;
  return slot->data;
}
