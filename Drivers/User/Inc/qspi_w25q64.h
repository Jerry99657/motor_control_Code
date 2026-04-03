#ifndef QSPI_W25Q64_H
#define QSPI_W25Q64_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define QSPI_W25QXX_OK                     0
#define W25QXX_ERROR_INIT                 -1
#define W25QXX_ERROR_WRITE_ENABLE         -2
#define W25QXX_ERROR_AUTOPOLLING          -3
#define W25QXX_ERROR_ERASE                -4
#define W25QXX_ERROR_TRANSMIT             -5
#define W25QXX_ERROR_MEMORY_MAPPED        -6
#define W25QXX_ERROR_PARAM                -7
#define W25QXX_ERROR_DEINIT               -8
#define W25QXX_ERROR_RESET                -9
#define W25QXX_ERROR_READID              -10

#define W25QXX_CMD_ENABLE_RESET           0x66U
#define W25QXX_CMD_RESET_DEVICE           0x99U
#define W25QXX_CMD_JEDEC_ID               0x9FU
#define W25QXX_CMD_WRITE_ENABLE           0x06U

#define W25QXX_CMD_SECTOR_ERASE_4K        0x20U
#define W25QXX_CMD_BLOCK_ERASE_32K        0x52U
#define W25QXX_CMD_BLOCK_ERASE_64K        0xD8U
#define W25QXX_CMD_CHIP_ERASE             0xC7U

#define W25QXX_CMD_QUAD_PAGE_PROGRAM      0x32U
#define W25QXX_CMD_FAST_READ_QUAD_IO      0xEBU
#define W25QXX_CMD_PAGE_PROGRAM           0x02U
#define W25QXX_CMD_FAST_READ_1_LINE       0x0BU
#define W25QXX_CMD_READ_STATUS_REG1       0x05U

#define W25QXX_STATUS_BUSY_MASK           0x01U
#define W25QXX_STATUS_WEL_MASK            0x02U

#define W25QXX_PAGE_SIZE                  256U
#define W25QXX_FLASH_SIZE_BYTES           0x800000U
#define W25QXX_FLASH_ID                   0x684017U
#define W25QXX_CHIP_ERASE_TIMEOUT_MS      100000U
#define W25QXX_OP_TIMEOUT_MS              5000U
#define W25QXX_MEM_MAPPED_ADDR            0x90000000U

int8_t QSPI_W25Qxx_Init(uint32_t *pDeviceId);
int8_t QSPI_W25Qxx_Reset(void);
uint32_t QSPI_W25Qxx_ReadID(void);
int8_t QSPI_W25Qxx_MemoryMappedMode(void);

int8_t QSPI_W25Qxx_SectorErase(uint32_t sectorAddress);
int8_t QSPI_W25Qxx_BlockErase_32K(uint32_t blockAddress);
int8_t QSPI_W25Qxx_BlockErase_64K(uint32_t blockAddress);
int8_t QSPI_W25Qxx_ChipErase(void);

int8_t QSPI_W25Qxx_WritePage(uint8_t *pBuffer, uint32_t writeAddr, uint16_t numByteToWrite);
int8_t QSPI_W25Qxx_WriteBuffer(uint8_t *pBuffer, uint32_t writeAddr, uint32_t size);
int8_t QSPI_W25Qxx_ReadBuffer(uint8_t *pBuffer, uint32_t readAddr, uint32_t numByteToRead);

#ifdef __cplusplus
}
#endif

#endif
