#include "qspi_w25q64.h"

extern QSPI_HandleTypeDef hqspi;

static int8_t QSPI_W25Qxx_AutoPollingMemReady(uint32_t timeoutMs)
{
  QSPI_CommandTypeDef sCommand = {0};
  QSPI_AutoPollingTypeDef sConfig = {0};

  sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  sCommand.AddressMode = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode = QSPI_DATA_1_LINE;
  sCommand.DummyCycles = 0;
  sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
  sCommand.Instruction = W25QXX_CMD_READ_STATUS_REG1;
  sCommand.NbData = 1;

  sConfig.Match = 0;
  sConfig.Mask = W25QXX_STATUS_BUSY_MASK;
  sConfig.MatchMode = QSPI_MATCH_MODE_AND;
  sConfig.StatusBytesSize = 1;
  sConfig.Interval = 0x10;
  sConfig.AutomaticStop = QSPI_AUTOMATIC_STOP_ENABLE;

  if (HAL_QSPI_AutoPolling(&hqspi, &sCommand, &sConfig, timeoutMs) != HAL_OK)
  {
    return W25QXX_ERROR_AUTOPOLLING;
  }

  return QSPI_W25QXX_OK;
}

static int8_t QSPI_W25Qxx_WriteEnable(void)
{
  QSPI_CommandTypeDef sCommand = {0};
  QSPI_AutoPollingTypeDef sConfig = {0};

  sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  sCommand.AddressMode = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode = QSPI_DATA_NONE;
  sCommand.DummyCycles = 0;
  sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
  sCommand.Instruction = W25QXX_CMD_WRITE_ENABLE;

  if (HAL_QSPI_Command(&hqspi, &sCommand, W25QXX_OP_TIMEOUT_MS) != HAL_OK)
  {
    return W25QXX_ERROR_WRITE_ENABLE;
  }

  sCommand.Instruction = W25QXX_CMD_READ_STATUS_REG1;
  sCommand.DataMode = QSPI_DATA_1_LINE;
  sCommand.NbData = 1;

  sConfig.Match = W25QXX_STATUS_WEL_MASK;
  sConfig.Mask = W25QXX_STATUS_WEL_MASK;
  sConfig.MatchMode = QSPI_MATCH_MODE_AND;
  sConfig.StatusBytesSize = 1;
  sConfig.Interval = 0x10;
  sConfig.AutomaticStop = QSPI_AUTOMATIC_STOP_ENABLE;

  if (HAL_QSPI_AutoPolling(&hqspi, &sCommand, &sConfig, W25QXX_OP_TIMEOUT_MS) != HAL_OK)
  {
    return W25QXX_ERROR_AUTOPOLLING;
  }

  return QSPI_W25QXX_OK;
}

static int8_t QSPI_W25Qxx_EraseByCommand(uint8_t instruction, uint32_t address)
{
  QSPI_CommandTypeDef sCommand = {0};

  if (QSPI_W25Qxx_WriteEnable() != QSPI_W25QXX_OK)
  {
    return W25QXX_ERROR_WRITE_ENABLE;
  }

  sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  sCommand.AddressMode = (instruction == W25QXX_CMD_CHIP_ERASE) ? QSPI_ADDRESS_NONE : QSPI_ADDRESS_1_LINE;
  sCommand.AddressSize = QSPI_ADDRESS_24_BITS;
  sCommand.Address = address;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode = QSPI_DATA_NONE;
  sCommand.DummyCycles = 0;
  sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
  sCommand.Instruction = instruction;

  if (HAL_QSPI_Command(&hqspi, &sCommand, W25QXX_OP_TIMEOUT_MS) != HAL_OK)
  {
    return W25QXX_ERROR_ERASE;
  }

  return QSPI_W25Qxx_AutoPollingMemReady(
      (instruction == W25QXX_CMD_CHIP_ERASE) ? W25QXX_CHIP_ERASE_TIMEOUT_MS : W25QXX_OP_TIMEOUT_MS);
}

int8_t QSPI_W25Qxx_Init(uint32_t *pDeviceId)
{
  uint32_t deviceId;
  uint8_t attempt;

  if (pDeviceId != NULL)
  {
    *pDeviceId = 0U;
  }

  if (HAL_QSPI_DeInit(&hqspi) != HAL_OK)
  {
    return W25QXX_ERROR_DEINIT;
  }

  hqspi.Init.ClockPrescaler = 7;
  hqspi.Init.FifoThreshold = 32;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hqspi.Init.FlashSize = 22;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_3;
  hqspi.Init.FlashID = QSPI_FLASH_ID_1;
  hqspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;

  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    return W25QXX_ERROR_INIT;
  }

  if (QSPI_W25Qxx_Reset() != QSPI_W25QXX_OK)
  {
    return W25QXX_ERROR_RESET;
  }

  HAL_Delay(10U);

  for (attempt = 0U; attempt < 3U; ++attempt)
  {
    deviceId = QSPI_W25Qxx_ReadID();
    if (pDeviceId != NULL)
    {
      *pDeviceId = deviceId;
    }

    if (deviceId == W25QXX_FLASH_ID)
    {
      return QSPI_W25QXX_OK;
    }

    HAL_Delay(2U);
  }

  return W25QXX_ERROR_READID;
}

int8_t QSPI_W25Qxx_Reset(void)
{
  QSPI_CommandTypeDef sCommand = {0};

  sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  sCommand.AddressMode = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode = QSPI_DATA_NONE;
  sCommand.DummyCycles = 0;
  sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
  sCommand.Instruction = W25QXX_CMD_ENABLE_RESET;

  if (HAL_QSPI_Command(&hqspi, &sCommand, W25QXX_OP_TIMEOUT_MS) != HAL_OK)
  {
    return W25QXX_ERROR_INIT;
  }

  HAL_Delay(1U);

  sCommand.Instruction = W25QXX_CMD_RESET_DEVICE;
  if (HAL_QSPI_Command(&hqspi, &sCommand, W25QXX_OP_TIMEOUT_MS) != HAL_OK)
  {
    return W25QXX_ERROR_INIT;
  }

  HAL_Delay(1U);

  return QSPI_W25QXX_OK;
}

uint32_t QSPI_W25Qxx_ReadID(void)
{
  QSPI_CommandTypeDef sCommand = {0};
  uint8_t recv[3] = {0};

  sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  sCommand.AddressMode = QSPI_ADDRESS_NONE;
  sCommand.AddressSize = QSPI_ADDRESS_24_BITS;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode = QSPI_DATA_1_LINE;
  sCommand.DummyCycles = 0;
  sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
  sCommand.NbData = 3;
  sCommand.Instruction = W25QXX_CMD_JEDEC_ID;

  if (HAL_QSPI_Command(&hqspi, &sCommand, W25QXX_OP_TIMEOUT_MS) != HAL_OK)
  {
    return 0;
  }

  if (HAL_QSPI_Receive(&hqspi, recv, W25QXX_OP_TIMEOUT_MS) != HAL_OK)
  {
    return 0;
  }

  return ((uint32_t)recv[0] << 16) | ((uint32_t)recv[1] << 8) | recv[2];
}

int8_t QSPI_W25Qxx_MemoryMappedMode(void)
{
  QSPI_CommandTypeDef sCommand = {0};
  QSPI_MemoryMappedTypeDef sMemMapped = {0};

  if (QSPI_W25Qxx_Reset() != QSPI_W25QXX_OK)
  {
    return W25QXX_ERROR_INIT;
  }

  sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  sCommand.AddressMode = QSPI_ADDRESS_1_LINE;
  sCommand.AddressSize = QSPI_ADDRESS_24_BITS;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode = QSPI_DATA_1_LINE;
  sCommand.DummyCycles = 8;
  sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
  sCommand.Instruction = W25QXX_CMD_FAST_READ_1_LINE;

  sMemMapped.TimeOutActivation = QSPI_TIMEOUT_COUNTER_DISABLE;
  sMemMapped.TimeOutPeriod = 0;

  if (HAL_QSPI_MemoryMapped(&hqspi, &sCommand, &sMemMapped) != HAL_OK)
  {
    return W25QXX_ERROR_MEMORY_MAPPED;
  }

  return QSPI_W25QXX_OK;
}

int8_t QSPI_W25Qxx_SectorErase(uint32_t sectorAddress)
{
  return QSPI_W25Qxx_EraseByCommand(W25QXX_CMD_SECTOR_ERASE_4K, sectorAddress);
}

int8_t QSPI_W25Qxx_BlockErase_32K(uint32_t blockAddress)
{
  return QSPI_W25Qxx_EraseByCommand(W25QXX_CMD_BLOCK_ERASE_32K, blockAddress);
}

int8_t QSPI_W25Qxx_BlockErase_64K(uint32_t blockAddress)
{
  return QSPI_W25Qxx_EraseByCommand(W25QXX_CMD_BLOCK_ERASE_64K, blockAddress);
}

int8_t QSPI_W25Qxx_ChipErase(void)
{
  return QSPI_W25Qxx_EraseByCommand(W25QXX_CMD_CHIP_ERASE, 0);
}

int8_t QSPI_W25Qxx_WritePage(uint8_t *pBuffer, uint32_t writeAddr, uint16_t numByteToWrite)
{
  QSPI_CommandTypeDef sCommand = {0};

  if (pBuffer == NULL || numByteToWrite == 0 || numByteToWrite > W25QXX_PAGE_SIZE)
  {
    return W25QXX_ERROR_PARAM;
  }

  if ((writeAddr + numByteToWrite) > W25QXX_FLASH_SIZE_BYTES)
  {
    return W25QXX_ERROR_PARAM;
  }

  if (QSPI_W25Qxx_WriteEnable() != QSPI_W25QXX_OK)
  {
    return W25QXX_ERROR_WRITE_ENABLE;
  }

  sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  sCommand.AddressMode = QSPI_ADDRESS_1_LINE;
  sCommand.AddressSize = QSPI_ADDRESS_24_BITS;
  sCommand.Address = writeAddr;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode = QSPI_DATA_1_LINE;
  sCommand.NbData = numByteToWrite;
  sCommand.DummyCycles = 0;
  sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
  sCommand.Instruction = W25QXX_CMD_PAGE_PROGRAM;

  if (HAL_QSPI_Command(&hqspi, &sCommand, W25QXX_OP_TIMEOUT_MS) != HAL_OK)
  {
    return W25QXX_ERROR_TRANSMIT;
  }

  if (HAL_QSPI_Transmit(&hqspi, pBuffer, W25QXX_OP_TIMEOUT_MS) != HAL_OK)
  {
    return W25QXX_ERROR_TRANSMIT;
  }

  return QSPI_W25Qxx_AutoPollingMemReady(W25QXX_OP_TIMEOUT_MS);
}

int8_t QSPI_W25Qxx_WriteBuffer(uint8_t *pBuffer, uint32_t writeAddr, uint32_t size)
{
  uint32_t currentAddr;
  uint32_t endAddr;
  uint32_t currentSize;
  uint8_t *writeData;

  if (pBuffer == NULL || size == 0)
  {
    return W25QXX_ERROR_PARAM;
  }

  if ((writeAddr + size) > W25QXX_FLASH_SIZE_BYTES)
  {
    return W25QXX_ERROR_PARAM;
  }

  currentAddr = writeAddr;
  endAddr = writeAddr + size;
  writeData = pBuffer;

  currentSize = W25QXX_PAGE_SIZE - (currentAddr % W25QXX_PAGE_SIZE);
  if (currentSize > size)
  {
    currentSize = size;
  }

  while (currentAddr < endAddr)
  {
    if (QSPI_W25Qxx_WritePage(writeData, currentAddr, (uint16_t)currentSize) != QSPI_W25QXX_OK)
    {
      return W25QXX_ERROR_TRANSMIT;
    }

    currentAddr += currentSize;
    writeData += currentSize;

    if ((currentAddr + W25QXX_PAGE_SIZE) > endAddr)
    {
      currentSize = endAddr - currentAddr;
    }
    else
    {
      currentSize = W25QXX_PAGE_SIZE;
    }
  }

  return QSPI_W25QXX_OK;
}

int8_t QSPI_W25Qxx_ReadBuffer(uint8_t *pBuffer, uint32_t readAddr, uint32_t numByteToRead)
{
  QSPI_CommandTypeDef sCommand = {0};
  uint8_t attempt;

  if (pBuffer == NULL || numByteToRead == 0)
  {
    return W25QXX_ERROR_PARAM;
  }

  if ((readAddr + numByteToRead) > W25QXX_FLASH_SIZE_BYTES)
  {
    return W25QXX_ERROR_PARAM;
  }

  sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  sCommand.AddressMode = QSPI_ADDRESS_1_LINE;
  sCommand.AddressSize = QSPI_ADDRESS_24_BITS;
  sCommand.Address = readAddr;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode = QSPI_DATA_1_LINE;
  sCommand.NbData = numByteToRead;
  sCommand.DummyCycles = 8;
  sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
  sCommand.Instruction = W25QXX_CMD_FAST_READ_1_LINE;

  for (attempt = 0U; attempt < 3U; ++attempt)
  {
    if (HAL_QSPI_Command(&hqspi, &sCommand, W25QXX_OP_TIMEOUT_MS) == HAL_OK)
    {
      if (HAL_QSPI_Receive(&hqspi, pBuffer, W25QXX_OP_TIMEOUT_MS) == HAL_OK)
      {
        /* Read does not need busy polling; command + receive success is enough. */
        return QSPI_W25QXX_OK;
      }
    }

    (void)QSPI_W25Qxx_Reset();
    HAL_Delay(1U);
  }

  return W25QXX_ERROR_TRANSMIT;
}
