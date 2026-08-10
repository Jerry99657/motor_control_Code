#ifndef QSPI_PARTITION_H
#define QSPI_PARTITION_H

#include "qspi_w25q64.h"

/*
 * W25Q64 persistent layout
 *
 * 0x000000 .. 0x3FFFFF  Start-up animation (4 MiB)
 * 0x400000 .. 0x400FFF  NES cache metadata (one erase sector)
 * 0x401000 .. 0x77FFFF  NES ROM image
 * 0x780000 .. 0x7FFFFF  LVGL GB2312 font package (512 KiB)
 *
 * Metadata is deliberately separated from the ROM payload.  It is committed
 * only after the copied ROM has passed a full read-back CRC check, so a power
 * loss during erase/program cannot expose a partially written ROM as valid.
 */
#define QSPI_PARTITION_START_ANIM_OFFSET      0x000000U
#define QSPI_PARTITION_START_ANIM_SIZE        0x400000U
#define QSPI_PARTITION_START_ANIM_END         \
  (QSPI_PARTITION_START_ANIM_OFFSET + QSPI_PARTITION_START_ANIM_SIZE)

#define QSPI_PARTITION_NES_METADATA_OFFSET    0x400000U
#define QSPI_PARTITION_NES_METADATA_SIZE      0x001000U
#define QSPI_PARTITION_NES_ROM_OFFSET         0x401000U

#define QSPI_PARTITION_UI_FONT_OFFSET         0x780000U
#define QSPI_PARTITION_UI_FONT_SIZE           0x080000U
#define QSPI_PARTITION_UI_FONT_END            \
  (QSPI_PARTITION_UI_FONT_OFFSET + QSPI_PARTITION_UI_FONT_SIZE)

#define QSPI_PARTITION_NES_ROM_SIZE           \
  (QSPI_PARTITION_UI_FONT_OFFSET - QSPI_PARTITION_NES_ROM_OFFSET)
#define QSPI_PARTITION_NES_ROM_END            \
  (QSPI_PARTITION_NES_ROM_OFFSET + QSPI_PARTITION_NES_ROM_SIZE)

#if (QSPI_PARTITION_START_ANIM_END > QSPI_PARTITION_NES_METADATA_OFFSET)
#error "QSPI start animation overlaps NES metadata"
#endif

#if ((QSPI_PARTITION_NES_METADATA_OFFSET + QSPI_PARTITION_NES_METADATA_SIZE) > \
     QSPI_PARTITION_NES_ROM_OFFSET)
#error "QSPI NES metadata overlaps NES ROM payload"
#endif

#if (QSPI_PARTITION_NES_ROM_END > QSPI_PARTITION_UI_FONT_OFFSET)
#error "QSPI NES ROM partition overlaps the UI font"
#endif

#if (QSPI_PARTITION_UI_FONT_END > W25QXX_FLASH_SIZE_BYTES)
#error "QSPI UI font partition exceeds W25Q64 capacity"
#endif

#if ((QSPI_PARTITION_START_ANIM_OFFSET & 0xFFFU) != 0U) || \
    ((QSPI_PARTITION_NES_METADATA_OFFSET & 0xFFFU) != 0U) || \
    ((QSPI_PARTITION_NES_ROM_OFFSET & 0xFFFU) != 0U) || \
    ((QSPI_PARTITION_UI_FONT_OFFSET & 0xFFFU) != 0U)
#error "QSPI partitions must be aligned to 4 KiB erase sectors"
#endif

#endif /* QSPI_PARTITION_H */
