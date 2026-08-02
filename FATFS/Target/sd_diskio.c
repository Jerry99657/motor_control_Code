/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sd_diskio.c
  * @brief   SD Disk I/O driver
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Note: code generation based on sd_diskio_dma_template_bspv1.c v2.1.4
   as "Use dma template" is enabled. */

/* USER CODE BEGIN firstSection */
/* can be used to modify / undefine following code or add new definitions */
/* USER CODE END firstSection*/

/* Includes ------------------------------------------------------------------*/
#include "ff_gen_drv.h"
#include "sd_diskio.h"

#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

 /*
 * the following Timeout is useful to give the control back to the applications
 * in case of errors in either BSP_SD_ReadCpltCallback() or BSP_SD_WriteCpltCallback()
 * the value by default is as defined in the BSP platform driver otherwise 30 secs
 */
#define SD_TIMEOUT 3000U

#define SD_DEFAULT_BLOCK_SIZE 512
#define SD_SCRATCH_BLOCK_COUNT 8U

/*
 * Depending on the use case, the SD card initialization could be done at the
 * application level: if it is the case define the flag below to disable
 * the BSP_SD_Init() call in the SD_Initialize() and add a call to
 * BSP_SD_Init() elsewhere in the application.
 */
/* USER CODE BEGIN disableSDInit */
/* #define DISABLE_SD_INIT */
/* USER CODE END disableSDInit */

/*
 * when using cacheable memory region, it may be needed to maintain the cache
 * validity. Enable the define below to activate a cache maintenance at each
 * read and write operation.
 * Notice: This is applicable only for cortex M7 based platform.
 */
/* USER CODE BEGIN enableSDDmaCacheMaintenance */
#define ENABLE_SD_DMA_CACHE_MAINTENANCE  1
/* USER CODE END enableSDDmaCacheMaintenance */

/*
* Some DMA requires 4-Byte aligned address buffer to correctly read/write data,
* in FatFs some accesses aren't thus we need a 4-byte aligned scratch buffer to correctly
* transfer data
*/
/* USER CODE BEGIN enableScratchBuffer */
#define ENABLE_SCRATCH_BUFFER
/* USER CODE END enableScratchBuffer */

/* Private variables ---------------------------------------------------------*/
#if defined(ENABLE_SCRATCH_BUFFER)
#if defined (ENABLE_SD_DMA_CACHE_MAINTENANCE)
ALIGN_32BYTES(static uint8_t scratch[BLOCKSIZE * SD_SCRATCH_BLOCK_COUNT]);
#else
__ALIGN_BEGIN static uint8_t scratch[BLOCKSIZE * SD_SCRATCH_BLOCK_COUNT] __ALIGN_END;
#endif
#endif
/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;

static volatile  UINT  WriteStatus = 0, ReadStatus = 0;
static volatile uint8_t SD_ForcePollingRead = 0U;
extern SD_HandleTypeDef hsd1;
/* Private function prototypes -----------------------------------------------*/
static DSTATUS SD_CheckStatus(BYTE lun);
DSTATUS SD_initialize (BYTE);
DSTATUS SD_status (BYTE);
DRESULT SD_read (BYTE, BYTE*, DWORD, UINT);
#if _USE_WRITE == 1
DRESULT SD_write (BYTE, const BYTE*, DWORD, UINT);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
DRESULT SD_ioctl (BYTE, BYTE, void*);
#endif  /* _USE_IOCTL == 1 */

const Diskio_drvTypeDef  SD_Driver =
{
  SD_initialize,
  SD_status,
  SD_read,
#if  _USE_WRITE == 1
  SD_write,
#endif /* _USE_WRITE == 1 */

#if  _USE_IOCTL == 1
  SD_ioctl,
#endif /* _USE_IOCTL == 1 */
};

/* USER CODE BEGIN beforeFunctionSection */
/* can be used to modify / undefine following code or add new code */
/* USER CODE END beforeFunctionSection */

/* Private functions ---------------------------------------------------------*/

static int SD_CheckStatusWithTimeout(uint32_t timeout)
{
  uint32_t timer = HAL_GetTick();
  /* block until SDIO IP is ready again or a timeout occur */
  while(HAL_GetTick() - timer < timeout)
  {
    if (BSP_SD_GetCardState() == SD_TRANSFER_OK)
    {
      return 0;
    }
  }

  return -1;
}

static uint8_t SD_ReadBlocksDMAOnce(uint32_t *data, uint32_t sector, uint32_t count)
{
  uint32_t timeout;

  /* Arm the completion flag before starting DMA.  Clearing it after the
     start races a fast completion IRQ and can lose the event forever. */
  ReadStatus = 0U;
  if (BSP_SD_ReadBlocks_DMA(data, sector, count) == MSD_OK)
  {
    timeout = HAL_GetTick();
    while ((ReadStatus == 0U) && ((HAL_GetTick() - timeout) < SD_TIMEOUT))
    {
    }

    if ((ReadStatus == 1U) && (SD_CheckStatusWithTimeout(SD_TIMEOUT) == 0))
    {
      ReadStatus = 0U;
      return MSD_OK;
    }
  }

  ReadStatus = 0U;
  return MSD_ERROR;
}

static uint8_t SD_ReadBlocksPollingOnce(uint32_t *data, uint32_t sector, uint32_t count)
{
  if (BSP_SD_ReadBlocks(data, sector, count, SD_TIMEOUT) != MSD_OK)
  {
    return MSD_ERROR;
  }

  return (SD_CheckStatusWithTimeout(SD_TIMEOUT) == 0) ? MSD_OK : MSD_ERROR;
}

static uint8_t SD_ReinitializeCard(void)
{
  /* HAL_SD_Abort() only resets the current data path.  After an IDMA/SDMMC
     error the HAL handle or the card can remain BUSY/ERROR, making every
     later FatFs access fail until reset.  Recreate the complete peripheral
     and card state before retrying the sector read. */
  (void)HAL_SD_Abort(&hsd1);
  ReadStatus = 0U;
  WriteStatus = 0U;

  if (HAL_SD_DeInit(&hsd1) != HAL_OK)
  {
    Stat |= STA_NOINIT;
    return MSD_ERROR;
  }

  HAL_NVIC_ClearPendingIRQ(SDMMC1_IRQn);
  __HAL_RCC_SDMMC1_FORCE_RESET();
  __DSB();
  __HAL_RCC_SDMMC1_RELEASE_RESET();
  HAL_Delay(1U);
  if (BSP_SD_Init() != MSD_OK)
  {
    Stat |= STA_NOINIT;
    return MSD_ERROR;
  }

  if (SD_CheckStatusWithTimeout(SD_TIMEOUT) < 0)
  {
    Stat |= STA_NOINIT;
    return MSD_ERROR;
  }

  Stat &= (DSTATUS)~STA_NOINIT;
  return MSD_OK;
}

static uint8_t SD_ReadBlocksWithRecovery(uint32_t *data, uint32_t sector, uint32_t count)
{
  if (SD_ReadBlocksDMAOnce(data, sector, count) == MSD_OK)
  {
    return MSD_OK;
  }

  /* Do not repeat the IDMA operation that just failed.  Reinitialize the card
     and finish this request in polling mode.  Clean the CPU-written data to
     RAM because the aligned caller performs a DMA-style cache invalidate. */
  if (SD_ReinitializeCard() == MSD_OK)
  {
    if (SD_ReadBlocksPollingOnce(data, sector, count) == MSD_OK)
    {
#if (ENABLE_SD_DMA_CACHE_MAINTENANCE == 1)
      SCB_CleanDCache_by_Addr((uint32_t *)data, count * BLOCKSIZE);
#endif
      return MSD_OK;
    }
  }

  (void)HAL_SD_Abort(&hsd1);
  Stat |= STA_NOINIT;
  ReadStatus = 0U;
  return MSD_ERROR;
}

static uint8_t SD_ReadBlocksPollingWithRecovery(uint32_t *data, uint32_t sector, uint32_t count)
{
  if (SD_ReadBlocksPollingOnce(data, sector, count) == MSD_OK)
  {
    return MSD_OK;
  }

  if ((SD_ReinitializeCard() == MSD_OK) &&
      (SD_ReadBlocksPollingOnce(data, sector, count) == MSD_OK))
  {
    return MSD_OK;
  }

  (void)HAL_SD_Abort(&hsd1);
  Stat |= STA_NOINIT;
  return MSD_ERROR;
}

#if defined(ENABLE_SCRATCH_BUFFER)
static uint8_t SD_ReadUnalignedBlocks(uint8_t *data, uint32_t sector, uint32_t count)
{
  uint32_t completed = 0U;

  while (completed < count)
  {
    uint32_t block_count = count - completed;

    if (block_count > SD_SCRATCH_BLOCK_COUNT)
    {
      block_count = SD_SCRATCH_BLOCK_COUNT;
    }

    /* AVI frame payloads are commonly unaligned.  Starting one IDMA transfer
       per 512-byte sector creates hundreds of short transfers per second and
       is the failure mode seen during MJPEG playback. */
    if (SD_ReadBlocksPollingWithRecovery(
          (uint32_t *)scratch,
          sector + completed,
          block_count
        ) != MSD_OK)
    {
      return MSD_ERROR;
    }

    memcpy(data, scratch, block_count * BLOCKSIZE);
    data += block_count * BLOCKSIZE;
    completed += block_count;
  }

  return MSD_OK;
}
#endif

static DSTATUS SD_CheckStatus(BYTE lun)
{
  Stat = STA_NOINIT;

  if(BSP_SD_GetCardState() == MSD_OK)
  {
    Stat &= ~STA_NOINIT;
  }

  return Stat;
}

/**
  * @brief  Initializes a Drive
  * @param  lun : not used
  * @retval DSTATUS: Operation status
  */
DSTATUS SD_initialize(BYTE lun)
{

#if !defined(DISABLE_SD_INIT)

  if(BSP_SD_Init() == MSD_OK)
  {
    Stat = SD_CheckStatus(lun);
  }

#else
  Stat = SD_CheckStatus(lun);
#endif

  return Stat;
}

/**
  * @brief  Gets Disk Status
  * @param  lun : not used
  * @retval DSTATUS: Operation status
  */
DSTATUS SD_status(BYTE lun)
{
  return SD_CheckStatus(lun);
}

void SD_SetReadPollingMode(uint8_t enabled)
{
  SD_ForcePollingRead = (enabled != 0U) ? 1U : 0U;
}

/* USER CODE BEGIN beforeReadSection */
/* can be used to modify previous code / undefine following code / add new code */
/* USER CODE END beforeReadSection */
/**
  * @brief  Reads Sector(s)
  * @param  lun : not used
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */

DRESULT SD_read(BYTE lun, BYTE *buff, DWORD sector, UINT count)
{
  DRESULT res = RES_ERROR;
#if (ENABLE_SD_DMA_CACHE_MAINTENANCE == 1)
  uint32_t alignedAddr;
#endif

  /*
  * ensure the SDCard is ready for a new operation
  */

  if (SD_CheckStatusWithTimeout(SD_TIMEOUT) < 0)
  {
    if (SD_ReinitializeCard() != MSD_OK)
    {
      return res;
    }
  }

#if defined(ENABLE_SCRATCH_BUFFER)
  if (!((uint32_t)buff & 0x1F))
  {
#endif
    if (SD_ForcePollingRead != 0U)
    {
      if (SD_ReadBlocksPollingWithRecovery((uint32_t *)buff, (uint32_t)sector, count) == MSD_OK)
      {
        res = RES_OK;
      }
    }
    else
    {
#if (ENABLE_SD_DMA_CACHE_MAINTENANCE == 1)
      alignedAddr = (uint32_t)buff & ~0x1F;
      SCB_CleanInvalidateDCache_by_Addr(
        (uint32_t *)alignedAddr,
        count * BLOCKSIZE + ((uint32_t)buff - alignedAddr)
      );
#endif
      if (SD_ReadBlocksWithRecovery((uint32_t *)buff, (uint32_t)sector, count) == MSD_OK)
      {
        res = RES_OK;
#if (ENABLE_SD_DMA_CACHE_MAINTENANCE == 1)
        alignedAddr = (uint32_t)buff & ~0x1F;
        SCB_InvalidateDCache_by_Addr((uint32_t*)alignedAddr, count*BLOCKSIZE + ((uint32_t)buff - alignedAddr));
#endif
      }
    }
#if defined(ENABLE_SCRATCH_BUFFER)
  }
    else
    {
      if (SD_ReadUnalignedBlocks(buff, (uint32_t)sector, count) == MSD_OK)
      {
        res = RES_OK;
      }
    }
#endif

  return res;
}

/* USER CODE BEGIN beforeWriteSection */
/* can be used to modify previous code / undefine following code / add new code */
/* USER CODE END beforeWriteSection */
/**
  * @brief  Writes Sector(s)
  * @param  lun : not used
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
#if _USE_WRITE == 1

DRESULT SD_write(BYTE lun, const BYTE *buff, DWORD sector, UINT count)
{
  DRESULT res = RES_ERROR;
  uint32_t timeout;
#if defined(ENABLE_SCRATCH_BUFFER)
  uint8_t ret;
  int i;
#endif

   WriteStatus = 0;
#if (ENABLE_SD_DMA_CACHE_MAINTENANCE == 1)
  uint32_t alignedAddr;
#endif

  if (SD_CheckStatusWithTimeout(SD_TIMEOUT) < 0)
  {
    return res;
  }

#if defined(ENABLE_SCRATCH_BUFFER)
  if (!((uint32_t)buff & 0x1F))
  {
#endif
#if (ENABLE_SD_DMA_CACHE_MAINTENANCE == 1)

    /*
    the SCB_CleanDCache_by_Addr() requires a 32-Byte aligned address
    adjust the address and the D-Cache size to clean accordingly.
    */
    alignedAddr = (uint32_t)buff &  ~0x1F;
    SCB_CleanDCache_by_Addr((uint32_t*)alignedAddr, count*BLOCKSIZE + ((uint32_t)buff - alignedAddr));
#endif

    WriteStatus = 0U;
    if(BSP_SD_WriteBlocks_DMA((uint32_t*)buff,
                              (uint32_t)(sector),
                              count) == MSD_OK)
    {
      /* Wait that writing process is completed or a timeout occurs */

      timeout = HAL_GetTick();
      while((WriteStatus == 0) && ((HAL_GetTick() - timeout) < SD_TIMEOUT))
      {
      }
      /* in case of a timeout return error */
      if (WriteStatus != 1U)
      {
        (void)HAL_SD_Abort(&hsd1);
        WriteStatus = 0U;
        res = RES_ERROR;
      }
      else
      {
        WriteStatus = 0;
        timeout = HAL_GetTick();

        while((HAL_GetTick() - timeout) < SD_TIMEOUT)
        {
          if (BSP_SD_GetCardState() == SD_TRANSFER_OK)
          {
            res = RES_OK;
            break;
          }
        }
      }
    }
#if defined(ENABLE_SCRATCH_BUFFER)
  }
    else
    {
      /* Slow path, fetch each sector a part and memcpy to destination buffer */
#if (ENABLE_SD_DMA_CACHE_MAINTENANCE == 1)
      /*
      * invalidate the scratch buffer before the next write to get the actual data instead of the cached one
      */
      SCB_InvalidateDCache_by_Addr((uint32_t*)scratch, BLOCKSIZE);
#endif

      for (i = 0; i < count; i++)
      {
        WriteStatus = 0;

        memcpy((void *)scratch, (void *)buff, BLOCKSIZE);
        buff += BLOCKSIZE;

#if (ENABLE_SD_DMA_CACHE_MAINTENANCE == 1)
        SCB_CleanDCache_by_Addr((uint32_t *)scratch, BLOCKSIZE);
#endif

        ret = BSP_SD_WriteBlocks_DMA((uint32_t*)scratch, (uint32_t)sector++, 1);
        if (ret == MSD_OK) {
          /* wait for a message from the queue or a timeout */
          timeout = HAL_GetTick();
          while((WriteStatus == 0) && ((HAL_GetTick() - timeout) < SD_TIMEOUT))
          {
          }
          if (WriteStatus != 1U)
          {
            (void)HAL_SD_Abort(&hsd1);
            WriteStatus = 0U;
            break;
          }

        }
        else
        {
          break;
        }
      }
      if ((i == count) && (ret == MSD_OK))
        res = RES_OK;
    }
#endif
  return res;
}
#endif /* _USE_WRITE == 1 */

/* USER CODE BEGIN beforeIoctlSection */
/* can be used to modify previous code / undefine following code / add new code */
/* USER CODE END beforeIoctlSection */
/**
  * @brief  I/O control operation
  * @param  lun : not used
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1
DRESULT SD_ioctl(BYTE lun, BYTE cmd, void *buff)
{
  DRESULT res = RES_ERROR;
  BSP_SD_CardInfo CardInfo;

  if (Stat & STA_NOINIT) return RES_NOTRDY;

  switch (cmd)
  {
  /* Make sure that no pending write process */
  case CTRL_SYNC :
    res = RES_OK;
    break;

  /* Get number of sectors on the disk (DWORD) */
  case GET_SECTOR_COUNT :
    BSP_SD_GetCardInfo(&CardInfo);
    *(DWORD*)buff = CardInfo.LogBlockNbr;
    res = RES_OK;
    break;

  /* Get R/W sector size (WORD) */
  case GET_SECTOR_SIZE :
    BSP_SD_GetCardInfo(&CardInfo);
    *(WORD*)buff = CardInfo.LogBlockSize;
    res = RES_OK;
    break;

  /* Get erase block size in unit of sector (DWORD) */
  case GET_BLOCK_SIZE :
    BSP_SD_GetCardInfo(&CardInfo);
    *(DWORD*)buff = CardInfo.LogBlockSize / SD_DEFAULT_BLOCK_SIZE;
    res = RES_OK;
    break;

  default:
    res = RES_PARERR;
  }

  return res;
}
#endif /* _USE_IOCTL == 1 */

/* USER CODE BEGIN afterIoctlSection */
/* can be used to modify previous code / undefine following code / add new code */
/* USER CODE END afterIoctlSection */

/* USER CODE BEGIN callbackSection */
/* can be used to modify / following code or add new code */
/* USER CODE END callbackSection */
/**
  * @brief Tx Transfer completed callbacks
  * @param hsd: SD handle
  * @retval None
  */
void BSP_SD_WriteCpltCallback(void)
{

  WriteStatus = 1;
}

/**
  * @brief Rx Transfer completed callbacks
  * @param hsd: SD handle
  * @retval None
  */
void BSP_SD_ReadCpltCallback(void)
{
  ReadStatus = 1;
}

/* USER CODE BEGIN ErrorAbortCallbacks */
void BSP_SD_AbortCallback(void)
{
  ReadStatus = 2U;
  WriteStatus = 2U;
}

void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd)
{
  if (hsd == &hsd1)
  {
    ReadStatus = 2U;
    WriteStatus = 2U;
  }
}
/* USER CODE END ErrorAbortCallbacks */

/* USER CODE BEGIN lastSection */
/* can be used to modify / undefine previous code or add new code */
/* USER CODE END lastSection */
