#ifndef NES_ROM_CACHE_H
#define NES_ROM_CACHE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NES_ROM_CACHE_OK                    0
#define NES_ROM_CACHE_ERR_PARAM            -1
#define NES_ROM_CACHE_ERR_BUSY             -2
#define NES_ROM_CACHE_ERR_MOUNT            -3
#define NES_ROM_CACHE_ERR_OPEN             -4
#define NES_ROM_CACHE_ERR_READ             -5
#define NES_ROM_CACHE_ERR_HEADER           -6
#define NES_ROM_CACHE_ERR_SIZE             -7
#define NES_ROM_CACHE_ERR_QSPI             -8
#define NES_ROM_CACHE_ERR_CRC              -9
#define NES_ROM_CACHE_ERR_METADATA        -10
#define NES_ROM_CACHE_ERR_UNSUPPORTED     -11
#define NES_ROM_CACHE_ERR_CANCELLED       -12

#define NES_ROM_CACHE_METADATA_MAGIC       0x3153454EU /* "NES1" */
#define NES_ROM_CACHE_METADATA_VERSION     1U
#define NES_ROM_CACHE_METADATA_SIZE        64U

typedef enum
{
  NES_ROM_CACHE_PHASE_IDLE = 0,
  NES_ROM_CACHE_PHASE_PREPARING,
  NES_ROM_CACHE_PHASE_ERASING,
  NES_ROM_CACHE_PHASE_COPYING,
  NES_ROM_CACHE_PHASE_VERIFYING,
  NES_ROM_CACHE_PHASE_COMMITTING,
  NES_ROM_CACHE_PHASE_READY,
  NES_ROM_CACHE_PHASE_ERROR,
  NES_ROM_CACHE_PHASE_CANCELLED
} NES_RomCachePhase;

typedef struct
{
  uint32_t magic;
  uint16_t version;
  uint16_t header_size;
  uint32_t rom_offset;
  uint32_t rom_size;
  uint32_t rom_crc32;
  uint32_t prg_size;
  uint32_t chr_size;
  uint16_t mapper;
  uint8_t flags6;
  uint8_t flags7;
  uint8_t ines_header[16];
  uint8_t reserved[12];
  uint32_t metadata_crc32;
} NES_RomCacheMetadata;

typedef struct
{
  NES_RomCachePhase phase;
  int8_t result;
  uint32_t completed_bytes;
  uint32_t total_bytes;
  uint32_t rom_size;
  uint32_t rom_crc32;
  uint32_t prg_size;
  uint32_t chr_size;
  uint16_t mapper;
  uint8_t flags6;
  uint8_t flags7;
} NES_RomCacheSnapshot;

/* Start and Process form a cooperative state machine.  Process performs at
 * most one flash erase or one 4 KiB I/O chunk per call. */
int8_t NES_RomCache_Start(const char *fatfs_path);
void NES_RomCache_Process(void);
void NES_RomCache_Cancel(void);
uint8_t NES_RomCache_IsBusy(void);
void NES_RomCache_GetSnapshot(NES_RomCacheSnapshot *snapshot);
const char *NES_RomCache_GetPath(void);
const char *NES_RomCache_PhaseText(NES_RomCachePhase phase);

int8_t NES_RomCache_LoadMetadata(NES_RomCacheMetadata *metadata);
int8_t NES_RomCache_Map(const uint8_t **rom, NES_RomCacheMetadata *metadata);
int8_t NES_RomCache_Unmap(void);

#ifdef __cplusplus
}
#endif

#endif /* NES_ROM_CACHE_H */
