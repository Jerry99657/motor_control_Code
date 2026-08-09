#include "nes_rom_cache.h"

#include "fatfs.h"
#include "qspi_partition.h"
#include "qspi_w25q64.h"
#include "sd_diskio.h"

#include <stddef.h>
#include <string.h>

#define NES_ROM_CACHE_PATH_SIZE          512U
#define NES_ROM_CACHE_IO_CHUNK           4096U
#define NES_ROM_CACHE_INES_HEADER_SIZE   16U
#define NES_ROM_CACHE_TRAINER_SIZE       512U
#define NES_ROM_CACHE_PRG_UNIT           16384U
#define NES_ROM_CACHE_CHR_UNIT           8192U

typedef struct
{
  NES_RomCacheSnapshot snapshot;
  NES_RomCacheMetadata metadata;
  FIL file;
  char path[NES_ROM_CACHE_PATH_SIZE];
  uint32_t file_position;
  uint32_t erase_address;
  uint32_t erase_end;
  uint32_t running_crc;
  uint32_t mapped_size;
  uint8_t mounted;
  uint8_t file_open;
  uint8_t cancel_requested;
  uint8_t polling_read_active;
} NES_RomCacheContext;

static NES_RomCacheContext s_cache;
static uint8_t s_io_buffer[NES_ROM_CACHE_IO_CHUNK]
  __attribute__((section(".ram_d2"), aligned(32)));

static uint8_t nes_cache_metadata_valid(const NES_RomCacheMetadata *metadata);

_Static_assert(sizeof(NES_RomCacheMetadata) == NES_ROM_CACHE_METADATA_SIZE,
               "NES cache metadata layout must remain 64 bytes");

static uint32_t nes_crc32_update(uint32_t crc, const uint8_t *data, uint32_t size)
{
  uint32_t i;
  uint32_t bit;

  for (i = 0U; i < size; ++i)
  {
    crc ^= data[i];
    for (bit = 0U; bit < 8U; ++bit)
    {
      crc = ((crc & 1U) != 0U) ?
              ((crc >> 1U) ^ 0xEDB88320U) : (crc >> 1U);
    }
  }

  return crc;
}

static uint32_t nes_crc32(const uint8_t *data, uint32_t size)
{
  return nes_crc32_update(0xFFFFFFFFU, data, size) ^ 0xFFFFFFFFU;
}

static uint32_t nes_align_up(uint32_t value, uint32_t alignment)
{
  return (value + alignment - 1U) & ~(alignment - 1U);
}

static uint8_t nes_cache_source_matches(
  const NES_RomCacheMetadata *cached,
  const NES_RomCacheMetadata *source
)
{
  if ((nes_cache_metadata_valid(cached) == 0U) || (source == NULL))
  {
    return 0U;
  }

  return ((cached->rom_size == source->rom_size) &&
          (cached->source_path_crc32 == source->source_path_crc32) &&
          (cached->source_fdate == source->source_fdate) &&
          (cached->source_ftime == source->source_ftime) &&
          (cached->source_tail_crc32 == source->source_tail_crc32) &&
          (memcmp(cached->ines_header, source->ines_header,
                  sizeof(cached->ines_header)) == 0)) ? 1U : 0U;
}

static void nes_cache_close_sd(void)
{
  if (s_cache.file_open != 0U)
  {
    (void)f_close(&s_cache.file);
    s_cache.file_open = 0U;
  }

  if (s_cache.mounted != 0U)
  {
    (void)f_mount(NULL, (TCHAR const *)SDPath, 1U);
    s_cache.mounted = 0U;
  }

  if (s_cache.polling_read_active != 0U)
  {
    SD_SetReadPollingMode(0U);
    s_cache.polling_read_active = 0U;
  }
}

static void nes_cache_set_polling_read(uint8_t enabled)
{
  enabled = (enabled != 0U) ? 1U : 0U;
  if (s_cache.polling_read_active != enabled)
  {
    SD_SetReadPollingMode(enabled);
    s_cache.polling_read_active = enabled;
  }
}

static void nes_cache_record_fs_error(FRESULT fr)
{
  s_cache.snapshot.last_fs_error = (uint8_t)fr;
  if (s_cache.snapshot.read_error_count < UINT8_MAX)
  {
    ++s_cache.snapshot.read_error_count;
  }
  if (s_cache.snapshot.read_retry_attempt < UINT8_MAX)
  {
    ++s_cache.snapshot.read_retry_attempt;
  }
}

/* FatFs latches a disk error in FIL.err.  Retrying f_read on the same FIL
 * therefore cannot recover even if the SD driver has already reinitialized
 * the card.  Close/unmount here; the next cooperative Process call remounts,
 * reopens and seeks to the last committed 4 KiB boundary. */
static int8_t nes_cache_defer_read_retry(FRESULT fr)
{
  nes_cache_record_fs_error(fr);
  nes_cache_close_sd();

  if (s_cache.snapshot.read_retry_attempt >=
      NES_ROM_CACHE_READ_RETRY_LIMIT)
  {
    return NES_ROM_CACHE_ERR_READ;
  }

  return NES_ROM_CACHE_OK;
}

static void nes_cache_read_recovered(void)
{
  if (s_cache.snapshot.read_retry_attempt != 0U)
  {
    if (s_cache.snapshot.read_recovery_count < UINT8_MAX)
    {
      ++s_cache.snapshot.read_recovery_count;
    }
    s_cache.snapshot.read_retry_attempt = 0U;
  }

  /* Polling is only a recovery path.  Return normal chunks to DMA after the
   * complete chunk has been read successfully. */
  nes_cache_set_polling_read(0U);
}

static FRESULT nes_cache_open_at(uint32_t position)
{
  FRESULT fr;

  nes_cache_close_sd();
  nes_cache_set_polling_read(
    (s_cache.snapshot.read_retry_attempt != 0U) ? 1U : 0U
  );

  fr = f_mount(&SDFatFS, (TCHAR const *)SDPath, 1U);
  if (fr != FR_OK)
  {
    nes_cache_close_sd();
    return fr;
  }
  s_cache.mounted = 1U;

  fr = f_open(&s_cache.file, s_cache.path, FA_READ);
  if (fr != FR_OK)
  {
    nes_cache_close_sd();
    return fr;
  }
  s_cache.file_open = 1U;

  if (position != 0U)
  {
    fr = f_lseek(&s_cache.file, position);
    if ((fr != FR_OK) || ((uint32_t)f_tell(&s_cache.file) != position))
    {
      if (fr == FR_OK)
      {
        fr = FR_DISK_ERR;
      }
      nes_cache_close_sd();
      return fr;
    }
  }

  return FR_OK;
}

static void nes_cache_finish(NES_RomCachePhase phase, int8_t result)
{
  nes_cache_close_sd();
  s_cache.snapshot.phase = phase;
  s_cache.snapshot.result = result;
  s_cache.cancel_requested = 0U;
}

static uint8_t nes_cache_metadata_valid(const NES_RomCacheMetadata *metadata)
{
  uint32_t expected_crc;
  uint32_t expected_minimum;
  uint32_t expected_prg;
  uint32_t expected_chr;
  uint16_t expected_mapper;

  if (metadata == NULL)
  {
    return 0U;
  }

  if ((metadata->magic != NES_ROM_CACHE_METADATA_MAGIC) ||
      (metadata->version != NES_ROM_CACHE_METADATA_VERSION) ||
      (metadata->header_size != NES_ROM_CACHE_METADATA_SIZE) ||
      (metadata->rom_offset != QSPI_PARTITION_NES_ROM_OFFSET) ||
      (metadata->rom_size == 0U) ||
      (metadata->rom_size > QSPI_PARTITION_NES_ROM_SIZE))
  {
    return 0U;
  }

  if ((metadata->ines_header[0] != 'N') ||
      (metadata->ines_header[1] != 'E') ||
      (metadata->ines_header[2] != 'S') ||
      (metadata->ines_header[3] != 0x1AU) ||
      ((metadata->ines_header[7] & 0x0CU) == 0x08U))
  {
    return 0U;
  }

  expected_prg = (uint32_t)metadata->ines_header[4] * NES_ROM_CACHE_PRG_UNIT;
  expected_chr = (uint32_t)metadata->ines_header[5] * NES_ROM_CACHE_CHR_UNIT;
  expected_mapper = (uint16_t)(((uint16_t)(metadata->ines_header[6] >> 4U)) |
                               ((uint16_t)metadata->ines_header[7] & 0xF0U));
  expected_minimum = NES_ROM_CACHE_INES_HEADER_SIZE +
                     (((metadata->ines_header[6] & 0x04U) != 0U) ?
                        NES_ROM_CACHE_TRAINER_SIZE : 0U) +
                     expected_prg + expected_chr;
  if ((expected_prg == 0U) || (metadata->rom_size < expected_minimum) ||
      (metadata->prg_size != expected_prg) ||
      (metadata->chr_size != expected_chr) ||
      (metadata->mapper != expected_mapper) ||
      (metadata->flags6 != metadata->ines_header[6]) ||
      (metadata->flags7 != metadata->ines_header[7]))
  {
    return 0U;
  }

  expected_crc = nes_crc32((const uint8_t *)metadata,
                           offsetof(NES_RomCacheMetadata, metadata_crc32));
  return (expected_crc == metadata->metadata_crc32) ? 1U : 0U;
}

static int8_t nes_cache_parse_header(const uint8_t *header, uint32_t file_size)
{
  uint32_t expected_minimum;

  if ((header[0] != 'N') || (header[1] != 'E') ||
      (header[2] != 'S') || (header[3] != 0x1AU))
  {
    return NES_ROM_CACHE_ERR_HEADER;
  }

  /* NES 2.0 uses extended size encoding.  Cache it only after that format is
     explicitly supported, rather than silently deriving the wrong layout. */
  if ((header[7] & 0x0CU) == 0x08U)
  {
    return NES_ROM_CACHE_ERR_UNSUPPORTED;
  }

  if (header[4] == 0U)
  {
    return NES_ROM_CACHE_ERR_HEADER;
  }

  s_cache.metadata.prg_size = (uint32_t)header[4] * NES_ROM_CACHE_PRG_UNIT;
  s_cache.metadata.chr_size = (uint32_t)header[5] * NES_ROM_CACHE_CHR_UNIT;
  expected_minimum = NES_ROM_CACHE_INES_HEADER_SIZE +
                     (((header[6] & 0x04U) != 0U) ? NES_ROM_CACHE_TRAINER_SIZE : 0U) +
                     s_cache.metadata.prg_size + s_cache.metadata.chr_size;
  if (file_size < expected_minimum)
  {
    return NES_ROM_CACHE_ERR_SIZE;
  }

  s_cache.metadata.mapper = (uint16_t)(((uint16_t)(header[6] >> 4U)) |
                                      ((uint16_t)header[7] & 0xF0U));
  s_cache.metadata.flags6 = header[6];
  s_cache.metadata.flags7 = header[7];
  (void)memcpy(s_cache.metadata.ines_header, header,
               sizeof(s_cache.metadata.ines_header));

  s_cache.snapshot.prg_size = s_cache.metadata.prg_size;
  s_cache.snapshot.chr_size = s_cache.metadata.chr_size;
  s_cache.snapshot.mapper = s_cache.metadata.mapper;
  s_cache.snapshot.flags6 = s_cache.metadata.flags6;
  s_cache.snapshot.flags7 = s_cache.metadata.flags7;
  return NES_ROM_CACHE_OK;
}

static int8_t nes_cache_prepare(void)
{
  FRESULT fr;
  FILINFO file_info;
  NES_RomCacheMetadata cached;
  UINT bytes_read = 0U;
  uint32_t file_size;
  uint32_t tail_position;
  uint32_t tail_size;
  int8_t result;

  if (NES_RomCache_Unmap() != NES_ROM_CACHE_OK)
  {
    return NES_ROM_CACHE_ERR_QSPI;
  }

  /* Header retries also use the polling fallback. */
  nes_cache_set_polling_read(
    (s_cache.snapshot.read_retry_attempt != 0U) ? 1U : 0U
  );
  fr = f_mount(&SDFatFS, (TCHAR const *)SDPath, 1U);
  if (fr != FR_OK)
  {
    if (s_cache.snapshot.read_retry_attempt != 0U)
    {
      return nes_cache_defer_read_retry(fr);
    }
    s_cache.snapshot.last_fs_error = (uint8_t)fr;
    return NES_ROM_CACHE_ERR_MOUNT;
  }
  s_cache.mounted = 1U;

  fr = f_open(&s_cache.file, s_cache.path, FA_READ);
  if (fr != FR_OK)
  {
    if (s_cache.snapshot.read_retry_attempt != 0U)
    {
      return nes_cache_defer_read_retry(fr);
    }
    s_cache.snapshot.last_fs_error = (uint8_t)fr;
    return NES_ROM_CACHE_ERR_OPEN;
  }
  s_cache.file_open = 1U;

  file_size = (uint32_t)f_size(&s_cache.file);
  if ((file_size < NES_ROM_CACHE_INES_HEADER_SIZE) ||
      (file_size > QSPI_PARTITION_NES_ROM_SIZE))
  {
    return NES_ROM_CACHE_ERR_SIZE;
  }

  fr = f_read(&s_cache.file, s_io_buffer,
              NES_ROM_CACHE_INES_HEADER_SIZE, &bytes_read);
  if ((fr != FR_OK) || (bytes_read != NES_ROM_CACHE_INES_HEADER_SIZE))
  {
    return nes_cache_defer_read_retry(
      (fr != FR_OK) ? fr : FR_DISK_ERR
    );
  }
  nes_cache_read_recovered();

  result = nes_cache_parse_header(s_io_buffer, file_size);
  if (result != NES_ROM_CACHE_OK)
  {
    return result;
  }

  tail_size = (file_size > sizeof(s_io_buffer)) ?
                sizeof(s_io_buffer) : file_size;
  tail_position = file_size - tail_size;
  fr = f_lseek(&s_cache.file, tail_position);
  if (fr != FR_OK)
  {
    return nes_cache_defer_read_retry(fr);
  }
  bytes_read = 0U;
  fr = f_read(&s_cache.file, s_io_buffer, (UINT)tail_size, &bytes_read);
  if ((fr != FR_OK) || (bytes_read != tail_size))
  {
    return nes_cache_defer_read_retry(
      (fr != FR_OK) ? fr : FR_DISK_ERR
    );
  }
  nes_cache_read_recovered();

  (void)memset(&file_info, 0, sizeof(file_info));
  fr = f_stat(s_cache.path, &file_info);
  s_cache.metadata.magic = NES_ROM_CACHE_METADATA_MAGIC;
  s_cache.metadata.version = NES_ROM_CACHE_METADATA_VERSION;
  s_cache.metadata.header_size = NES_ROM_CACHE_METADATA_SIZE;
  s_cache.metadata.rom_offset = QSPI_PARTITION_NES_ROM_OFFSET;
  s_cache.metadata.rom_size = file_size;
  s_cache.metadata.rom_crc32 = 0U;
  s_cache.metadata.source_path_crc32 =
    nes_crc32((const uint8_t *)s_cache.path, (uint32_t)strlen(s_cache.path));
  s_cache.metadata.source_fdate = (fr == FR_OK) ? file_info.fdate : 0U;
  s_cache.metadata.source_ftime = (fr == FR_OK) ? file_info.ftime : 0U;
  s_cache.metadata.source_tail_crc32 = nes_crc32(s_io_buffer, tail_size);
  s_cache.metadata.metadata_crc32 = 0U;

  s_cache.snapshot.rom_size = file_size;
  s_cache.snapshot.rom_crc32 = 0U;
  s_cache.snapshot.completed_bytes = 0U;
  s_cache.snapshot.total_bytes =
    nes_align_up(QSPI_PARTITION_NES_ROM_OFFSET + file_size, 4096U) -
    QSPI_PARTITION_NES_METADATA_OFFSET;
  s_cache.erase_address = QSPI_PARTITION_NES_METADATA_OFFSET;
  s_cache.erase_end = nes_align_up(QSPI_PARTITION_NES_ROM_OFFSET + file_size,
                                   4096U);
  s_cache.file_position = 0U;
  s_cache.running_crc = 0xFFFFFFFFU;

  /* A path/date/size/header/tail match is a fast cache hit.  The metadata
   * itself is CRC protected, and the tail guard catches replaced files that
   * retained the same name and size. */
  if ((QSPI_W25Qxx_ReadBuffer((uint8_t *)&cached,
                              QSPI_PARTITION_NES_METADATA_OFFSET,
                              sizeof(cached)) == QSPI_W25QXX_OK) &&
      (nes_cache_source_matches(&cached, &s_cache.metadata) != 0U))
  {
    s_cache.metadata = cached;
    s_cache.snapshot.rom_crc32 = cached.rom_crc32;
    s_cache.snapshot.completed_bytes = file_size;
    s_cache.snapshot.total_bytes = file_size;
    s_cache.snapshot.cache_hit = 1U;
    nes_cache_finish(NES_ROM_CACHE_PHASE_READY, NES_ROM_CACHE_OK);
    return NES_ROM_CACHE_OK;
  }

  /* Do not keep a FatFs object open while QSPI sectors are being erased.
   * Reopen it immediately before COPY so a long erase phase cannot leave a
   * stale SD/FatFs state behind. */
  nes_cache_close_sd();
  s_cache.snapshot.phase = NES_ROM_CACHE_PHASE_ERASING;
  return NES_ROM_CACHE_OK;
}

static int8_t nes_cache_erase_one(void)
{
  uint32_t remaining;
  uint32_t erase_size;
  int8_t result;

  if (s_cache.erase_address >= s_cache.erase_end)
  {
    s_cache.file_position = 0U;
    s_cache.running_crc = 0xFFFFFFFFU;
    s_cache.snapshot.completed_bytes = 0U;
    s_cache.snapshot.total_bytes = s_cache.metadata.rom_size;
    s_cache.snapshot.phase = NES_ROM_CACHE_PHASE_COPYING;
    return NES_ROM_CACHE_OK;
  }

  remaining = s_cache.erase_end - s_cache.erase_address;
  if (((s_cache.erase_address & 0xFFFFU) == 0U) && (remaining >= 0x10000U))
  {
    erase_size = 0x10000U;
    result = QSPI_W25Qxx_BlockErase_64K(s_cache.erase_address);
  }
  else if (((s_cache.erase_address & 0x7FFFU) == 0U) && (remaining >= 0x8000U))
  {
    erase_size = 0x8000U;
    result = QSPI_W25Qxx_BlockErase_32K(s_cache.erase_address);
  }
  else
  {
    erase_size = 0x1000U;
    result = QSPI_W25Qxx_SectorErase(s_cache.erase_address);
  }

  if (result != QSPI_W25QXX_OK)
  {
    return NES_ROM_CACHE_ERR_QSPI;
  }

  s_cache.erase_address += erase_size;
  s_cache.snapshot.completed_bytes =
    s_cache.erase_address - QSPI_PARTITION_NES_METADATA_OFFSET;
  if (s_cache.snapshot.completed_bytes > s_cache.snapshot.total_bytes)
  {
    s_cache.snapshot.completed_bytes = s_cache.snapshot.total_bytes;
  }
  return NES_ROM_CACHE_OK;
}

static int8_t nes_cache_copy_one(void)
{
  UINT bytes_read = 0U;
  uint32_t remaining;
  uint32_t chunk;
  FRESULT fr;

  remaining = s_cache.metadata.rom_size - s_cache.file_position;
  if (remaining == 0U)
  {
    s_cache.metadata.rom_crc32 = s_cache.running_crc ^ 0xFFFFFFFFU;
    s_cache.snapshot.rom_crc32 = s_cache.metadata.rom_crc32;
    nes_cache_close_sd();
    s_cache.file_position = 0U;
    s_cache.running_crc = 0xFFFFFFFFU;
    s_cache.snapshot.completed_bytes = 0U;
    s_cache.snapshot.total_bytes = s_cache.metadata.rom_size;
    s_cache.snapshot.phase = NES_ROM_CACHE_PHASE_VERIFYING;
    return NES_ROM_CACHE_OK;
  }

  chunk = (remaining > sizeof(s_io_buffer)) ? sizeof(s_io_buffer) : remaining;
  if (s_cache.file_open == 0U)
  {
    fr = nes_cache_open_at(s_cache.file_position);
    if (fr != FR_OK)
    {
      return nes_cache_defer_read_retry(fr);
    }
  }

  fr = f_read(&s_cache.file, s_io_buffer, (UINT)chunk, &bytes_read);
  if ((fr != FR_OK) || (bytes_read != chunk))
  {
    return nes_cache_defer_read_retry(
      (fr != FR_OK) ? fr : FR_DISK_ERR
    );
  }
  nes_cache_read_recovered();

  s_cache.running_crc = nes_crc32_update(s_cache.running_crc,
                                         s_io_buffer, chunk);
  if (QSPI_W25Qxx_WriteBuffer(s_io_buffer,
                              QSPI_PARTITION_NES_ROM_OFFSET +
                              s_cache.file_position,
                              chunk) != QSPI_W25QXX_OK)
  {
    return NES_ROM_CACHE_ERR_QSPI;
  }

  s_cache.file_position += chunk;
  s_cache.snapshot.completed_bytes = s_cache.file_position;
  return NES_ROM_CACHE_OK;
}

static int8_t nes_cache_verify_one(void)
{
  uint32_t remaining;
  uint32_t chunk;
  uint32_t crc;

  remaining = s_cache.metadata.rom_size - s_cache.file_position;
  if (remaining == 0U)
  {
    crc = s_cache.running_crc ^ 0xFFFFFFFFU;
    if (crc != s_cache.metadata.rom_crc32)
    {
      return NES_ROM_CACHE_ERR_CRC;
    }

    s_cache.snapshot.completed_bytes = 0U;
    s_cache.snapshot.total_bytes = NES_ROM_CACHE_METADATA_SIZE;
    s_cache.snapshot.phase = NES_ROM_CACHE_PHASE_COMMITTING;
    return NES_ROM_CACHE_OK;
  }

  chunk = (remaining > sizeof(s_io_buffer)) ? sizeof(s_io_buffer) : remaining;
  if (QSPI_W25Qxx_ReadBuffer(s_io_buffer,
                             QSPI_PARTITION_NES_ROM_OFFSET +
                             s_cache.file_position,
                             chunk) != QSPI_W25QXX_OK)
  {
    return NES_ROM_CACHE_ERR_QSPI;
  }

  s_cache.running_crc = nes_crc32_update(s_cache.running_crc,
                                         s_io_buffer, chunk);
  s_cache.file_position += chunk;
  s_cache.snapshot.completed_bytes = s_cache.file_position;
  return NES_ROM_CACHE_OK;
}

static int8_t nes_cache_commit(void)
{
  NES_RomCacheMetadata readback;

  s_cache.metadata.metadata_crc32 =
    nes_crc32((const uint8_t *)&s_cache.metadata,
              offsetof(NES_RomCacheMetadata, metadata_crc32));
  if (QSPI_W25Qxx_WriteBuffer((uint8_t *)&s_cache.metadata,
                              QSPI_PARTITION_NES_METADATA_OFFSET,
                              sizeof(s_cache.metadata)) != QSPI_W25QXX_OK)
  {
    return NES_ROM_CACHE_ERR_QSPI;
  }

  if (QSPI_W25Qxx_ReadBuffer((uint8_t *)&readback,
                             QSPI_PARTITION_NES_METADATA_OFFSET,
                             sizeof(readback)) != QSPI_W25QXX_OK)
  {
    return NES_ROM_CACHE_ERR_QSPI;
  }

  if ((memcmp(&readback, &s_cache.metadata, sizeof(readback)) != 0) ||
      (nes_cache_metadata_valid(&readback) == 0U))
  {
    return NES_ROM_CACHE_ERR_METADATA;
  }

  s_cache.snapshot.completed_bytes = NES_ROM_CACHE_METADATA_SIZE;
  return NES_ROM_CACHE_OK;
}

int8_t NES_RomCache_Start(const char *fatfs_path)
{
  size_t path_length;

  if (fatfs_path == NULL)
  {
    return NES_ROM_CACHE_ERR_PARAM;
  }

  path_length = strlen(fatfs_path);
  if ((path_length == 0U) || (path_length >= sizeof(s_cache.path)))
  {
    return NES_ROM_CACHE_ERR_PARAM;
  }

  if (NES_RomCache_IsBusy() != 0U)
  {
    return NES_ROM_CACHE_ERR_BUSY;
  }

  if (NES_RomCache_Unmap() != NES_ROM_CACHE_OK)
  {
    return NES_ROM_CACHE_ERR_QSPI;
  }

  nes_cache_close_sd();
  (void)memset(&s_cache, 0, sizeof(s_cache));
  (void)memcpy(s_cache.path, fatfs_path, path_length + 1U);
  s_cache.snapshot.phase = NES_ROM_CACHE_PHASE_PREPARING;
  s_cache.snapshot.result = NES_ROM_CACHE_OK;
  return NES_ROM_CACHE_OK;
}

void NES_RomCache_Process(void)
{
  int8_t result = NES_ROM_CACHE_OK;

  if (s_cache.cancel_requested != 0U)
  {
    nes_cache_finish(NES_ROM_CACHE_PHASE_CANCELLED,
                     NES_ROM_CACHE_ERR_CANCELLED);
    return;
  }

  switch (s_cache.snapshot.phase)
  {
    case NES_ROM_CACHE_PHASE_PREPARING:
      result = nes_cache_prepare();
      break;
    case NES_ROM_CACHE_PHASE_ERASING:
      result = nes_cache_erase_one();
      break;
    case NES_ROM_CACHE_PHASE_COPYING:
      result = nes_cache_copy_one();
      break;
    case NES_ROM_CACHE_PHASE_VERIFYING:
      result = nes_cache_verify_one();
      break;
    case NES_ROM_CACHE_PHASE_COMMITTING:
      result = nes_cache_commit();
      if (result == NES_ROM_CACHE_OK)
      {
        nes_cache_finish(NES_ROM_CACHE_PHASE_READY, NES_ROM_CACHE_OK);
      }
      break;
    default:
      return;
  }

  if (result != NES_ROM_CACHE_OK)
  {
    nes_cache_finish(NES_ROM_CACHE_PHASE_ERROR, result);
  }
}

void NES_RomCache_Cancel(void)
{
  if (NES_RomCache_IsBusy() != 0U)
  {
    s_cache.cancel_requested = 0U;
    nes_cache_finish(NES_ROM_CACHE_PHASE_CANCELLED,
                     NES_ROM_CACHE_ERR_CANCELLED);
  }
}

uint8_t NES_RomCache_IsBusy(void)
{
  return ((s_cache.snapshot.phase >= NES_ROM_CACHE_PHASE_PREPARING) &&
          (s_cache.snapshot.phase <= NES_ROM_CACHE_PHASE_COMMITTING)) ? 1U : 0U;
}

void NES_RomCache_GetSnapshot(NES_RomCacheSnapshot *snapshot)
{
  if (snapshot != NULL)
  {
    *snapshot = s_cache.snapshot;
  }
}

const char *NES_RomCache_GetPath(void)
{
  return s_cache.path;
}

const char *NES_RomCache_PhaseText(NES_RomCachePhase phase)
{
  switch (phase)
  {
    case NES_ROM_CACHE_PHASE_PREPARING: return "Inspecting iNES";
    case NES_ROM_CACHE_PHASE_ERASING: return "Erasing QSPI";
    case NES_ROM_CACHE_PHASE_COPYING: return "Copying SD to QSPI";
    case NES_ROM_CACHE_PHASE_VERIFYING: return "Verifying CRC32";
    case NES_ROM_CACHE_PHASE_COMMITTING: return "Committing metadata";
    case NES_ROM_CACHE_PHASE_READY: return "Cache ready";
    case NES_ROM_CACHE_PHASE_ERROR: return "Cache failed";
    case NES_ROM_CACHE_PHASE_CANCELLED: return "Cancelled";
    default: return "Idle";
  }
}

int8_t NES_RomCache_LoadMetadata(NES_RomCacheMetadata *metadata)
{
  NES_RomCacheMetadata local;

  if (metadata == NULL)
  {
    return NES_ROM_CACHE_ERR_PARAM;
  }
  if (NES_RomCache_IsBusy() != 0U)
  {
    return NES_ROM_CACHE_ERR_BUSY;
  }
  if (NES_RomCache_Unmap() != NES_ROM_CACHE_OK)
  {
    return NES_ROM_CACHE_ERR_QSPI;
  }
  if (QSPI_W25Qxx_ReadBuffer((uint8_t *)&local,
                             QSPI_PARTITION_NES_METADATA_OFFSET,
                             sizeof(local)) != QSPI_W25QXX_OK)
  {
    return NES_ROM_CACHE_ERR_QSPI;
  }
  if (nes_cache_metadata_valid(&local) == 0U)
  {
    return NES_ROM_CACHE_ERR_METADATA;
  }

  *metadata = local;
  return NES_ROM_CACHE_OK;
}

int8_t NES_RomCache_Map(const uint8_t **rom, NES_RomCacheMetadata *metadata)
{
  NES_RomCacheMetadata local;
  uintptr_t start;
  uintptr_t aligned_start;
  uint32_t invalidate_size;
  int8_t result;

  if (rom == NULL)
  {
    return NES_ROM_CACHE_ERR_PARAM;
  }

  result = NES_RomCache_LoadMetadata(&local);
  if (result != NES_ROM_CACHE_OK)
  {
    return result;
  }
  if (QSPI_W25Qxx_MemoryMappedMode() != QSPI_W25QXX_OK)
  {
    return NES_ROM_CACHE_ERR_QSPI;
  }

  start = (uintptr_t)W25QXX_MEM_MAPPED_ADDR + local.rom_offset;
  aligned_start = start & ~(uintptr_t)31U;
  invalidate_size = (uint32_t)(start - aligned_start) + local.rom_size;
  invalidate_size = nes_align_up(invalidate_size, 32U);
  SCB_InvalidateDCache_by_Addr((uint32_t *)aligned_start,
                              (int32_t)invalidate_size);
  __DSB();
  __ISB();

  s_cache.mapped_size = local.rom_size;
  *rom = (const uint8_t *)start;
  if (metadata != NULL)
  {
    *metadata = local;
  }
  return NES_ROM_CACHE_OK;
}

int8_t NES_RomCache_Unmap(void)
{
  uintptr_t start;
  uintptr_t aligned_start;
  uint32_t invalidate_size;

  if (QSPI_W25Qxx_IsMemoryMapped() == 0U)
  {
    s_cache.mapped_size = 0U;
    return NES_ROM_CACHE_OK;
  }

  if (s_cache.mapped_size != 0U)
  {
    start = (uintptr_t)W25QXX_MEM_MAPPED_ADDR +
            QSPI_PARTITION_NES_ROM_OFFSET;
    aligned_start = start & ~(uintptr_t)31U;
    invalidate_size = (uint32_t)(start - aligned_start) +
                      s_cache.mapped_size;
    invalidate_size = nes_align_up(invalidate_size, 32U);
    SCB_InvalidateDCache_by_Addr((uint32_t *)aligned_start,
                                (int32_t)invalidate_size);
    __DSB();
  }

  if (QSPI_W25Qxx_ExitMemoryMappedMode() != QSPI_W25QXX_OK)
  {
    return NES_ROM_CACHE_ERR_QSPI;
  }
  s_cache.mapped_size = 0U;
  return NES_ROM_CACHE_OK;
}
