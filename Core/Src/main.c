/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "bsp_driver_sd.h"
#include "sd_service.h"
#include "sd_bench_config.h"
#include "lcd_spi_154.h"
#include "logo_image.h"
#include "qspi_w25q64.h"
#include "qspi_start_anim.h"
#include "qspi_anim_loader.h"
#include "sd_start_anim.h"
#include "mjpeg_scheduler.h"
#include "dc_motor_ol.h"
#include "lvgl.h"
#include "lv_port_disp.h"
#include "lv_port_indev.h"
#include "lvgl_app.h"
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SD_TEST_STAGE_INIT          1U
#define SD_TEST_STAGE_MOUNT         2U
#define SD_TEST_STAGE_OPEN_WRITE    3U
#define SD_TEST_STAGE_WRITE         4U
#define SD_TEST_STAGE_CLOSE_WRITE   5U
#define SD_TEST_STAGE_OPEN_READ     6U
#define SD_TEST_STAGE_READ          7U
#define SD_TEST_STAGE_VERIFY        8U
#define STAGE_LOG_BUFFER_SIZE       2048U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac1;

DMA2D_HandleTypeDef hdma2d;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c4;

JPEG_HandleTypeDef hjpeg;
MDMA_HandleTypeDef hmdma_jpeg_infifo_th;
MDMA_HandleTypeDef hmdma_jpeg_outfifo_th;

QSPI_HandleTypeDef hqspi;

SD_HandleTypeDef hsd1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi6;
DMA_HandleTypeDef hdma_spi6_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static int8_t g_qspi_init_status = W25QXX_ERROR_INIT;
static uint32_t g_qspi_jedec_id = 0;
static int8_t g_start_anim_status = QSPI_START_ANIM_ERR_HEADER;
static int8_t g_sd_start_anim_status = SD_START_ANIM_ERR_FILE;
static int8_t g_start_anim_download_status = 0;
static uint8_t g_cdc_welcome_sent = 0U;
static uint32_t g_cdc_welcome_last_tick = 0U;
static uint8_t g_sd_self_test_done = 0U;
static int8_t g_sd_self_test_status = -1;
static uint8_t g_sd_self_test_reported = 0U;
static uint8_t g_sd_self_test_stage = 0U;
static FRESULT g_sd_self_test_fresult = FR_OK;
static uint8_t g_sd_detect_status = SD_NOT_PRESENT;
static uint8_t g_sd_card_state = SD_TRANSFER_BUSY;
static uint32_t g_sd_hal_error_code = 0U;
static uint8_t g_sd_service_demo_done = 0U;
#if SD_BENCH_ENABLE
static uint8_t g_sd_bench_done = 0U;
static uint8_t g_sd_bench_reported = 0U;
static sd_bench_result_t g_sd_seq_write_result = {0};
static sd_bench_result_t g_sd_rand_read_result = {0};
#endif
static FRESULT g_sd_service_demo_result = FR_OK;
static char g_stage_log_buffer[STAGE_LOG_BUFFER_SIZE];
static uint16_t g_stage_log_len = 0U;
static uint16_t g_stage_log_flush_pos = 0U;
static uint8_t g_stage_log_overflow = 0U;
static uint8_t g_jpeg_init_ok = 0U;
static uint8_t g_dma2d_init_ok = 0U;
static uint8_t g_tim7_init_ok = 0U;
static uint8_t g_tim7_start_ok = 0U;
volatile uint32_t g_tim7_frame_tick = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_BDMA_Init(void);
static void MX_MDMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_I2C4_Init(void);
static void MX_SPI6_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM15_Init(void);
static void MX_UART4_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART5_Init(void);
static void MX_ADC1_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_JPEG_Init(void);
static void MX_TIM6_Init(void);
static void MX_DMA2D_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */
static void LCD_ShowStartupScreen(void);
static void LCD_ShowDownloadScreen(void);
static const char *QSPI_GetInitErrorText(int8_t status);
static const char *QSPI_GetStartAnimErrorText(int8_t status);
static const char *SD_GetStartAnimErrorText(int8_t status);
static void QSPI_FormatJedecId(char *line, uint32_t jedecId);
static void QSPI_BootInit(void);
static uint8_t Boot_ShouldEnterAnimDownloadMode(void);
static void Boot_TryDownloadStartAnimViaCdc(void);
static int8_t SD_RunFatFsRwSelfTest(void);
static void Boot_TryReportSdSelfTestViaCdc(void);
static const char *SD_GetSelfTestStageText(uint8_t stage);
static const char *SD_GetFresultText(FRESULT fr);
static void SD_RunServiceDemo(void);
#if SD_BENCH_ENABLE
static void SD_RunBenchmarks(void);
static void Boot_TryReportSdBenchViaCdc(void);
#endif
static void Boot_LogText(const char *text);
static void Boot_LogStatus(const char *prefix, int32_t value);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Boot_DebugStageLog(const char *text)
{
  uint16_t textLen;

  if (text == NULL)
  {
    return;
  }

  textLen = (uint16_t)strlen(text);
  if (textLen == 0U)
  {
    return;
  }

  if ((uint16_t)(g_stage_log_len + textLen) >= STAGE_LOG_BUFFER_SIZE)
  {
    if (g_stage_log_overflow == 0U)
    {
      static const char overflowMsg[] = "LOG: buffer overflow\\r\\n";
      uint16_t remain = (uint16_t)(STAGE_LOG_BUFFER_SIZE - g_stage_log_len - 1U);
      uint16_t copyLen = (uint16_t)(sizeof(overflowMsg) - 1U);

      if (copyLen > remain)
      {
        copyLen = remain;
      }

      if (copyLen > 0U)
      {
        memcpy(&g_stage_log_buffer[g_stage_log_len], overflowMsg, copyLen);
        g_stage_log_len = (uint16_t)(g_stage_log_len + copyLen);
      }

      g_stage_log_overflow = 1U;
    }
    return;
  }

  memcpy(&g_stage_log_buffer[g_stage_log_len], text, textLen);
  g_stage_log_len = (uint16_t)(g_stage_log_len + textLen);
}

void Boot_DebugFlushStageLogsViaCdc(void)
{
  uint16_t start;
  uint16_t end;

  if (g_stage_log_flush_pos >= g_stage_log_len)
  {
    return;
  }

  start = g_stage_log_flush_pos;
  end = start;

  while ((end < g_stage_log_len) && (g_stage_log_buffer[end] != '\n'))
  {
    end++;
  }

  if (end < g_stage_log_len)
  {
    end++;
  }

  if (end <= start)
  {
    return;
  }

  if (CDC_Transmit_FS((uint8_t *)&g_stage_log_buffer[start], (uint16_t)(end - start)) == USBD_OK)
  {
    g_stage_log_flush_pos = end;
  }
}

static void QSPI_BootInit(void)
{
  g_qspi_init_status = QSPI_W25Qxx_Init(&g_qspi_jedec_id);
}

static void Boot_LogText(const char *text)
{
  Boot_DebugStageLog(text);
}

static void Boot_LogStatus(const char *prefix, int32_t value)
{
  char line[64];
  int len;

  len = snprintf(line, sizeof(line), "%s%ld\r\n", prefix, (long)value);
  if (len > 0)
  {
    Boot_LogText(line);
  }
}

static const char *QSPI_GetInitErrorText(int8_t status)
{
  switch (status)
  {
    case W25QXX_ERROR_DEINIT:
      return "QSPI DEINIT ERROR";
    case W25QXX_ERROR_RESET:
      return "QSPI RESET ERROR";
    case W25QXX_ERROR_READID:
      return "QSPI READID ERROR";
    case W25QXX_ERROR_INIT:
      return "QSPI HAL INIT ERROR";
    default:
      return "QSPI INIT ERROR";
  }
}

static const char *QSPI_GetStartAnimErrorText(int8_t status)
{
  switch (status)
  {
    case QSPI_START_ANIM_ERR_PARAM:
      return "ANIM PARAM ERROR";
    case QSPI_START_ANIM_ERR_HEADER:
      return "ANIM HEADER ERROR";
    case QSPI_START_ANIM_ERR_QSPI:
      return "ANIM QSPI ERROR";
    default:
      return "ANIM PLAY ERROR";
  }
}

static const char *SD_GetStartAnimErrorText(int8_t status)
{
  switch (status)
  {
    case SD_START_ANIM_ERR_MOUNT:
      return "SD MOUNT ERROR";
    case SD_START_ANIM_ERR_FILE:
      return "SD ANIM MISSING";
    case SD_START_ANIM_ERR_HEADER:
      return "SD ANIM HEADER";
    case SD_START_ANIM_ERR_IO:
      return "SD ANIM IO ERR";
    default:
      return "SD ANIM ERROR";
  }
}

static void QSPI_FormatJedecId(char *line, uint32_t jedecId)
{
  static const char hexDigits[] = "0123456789ABCDEF";
  uint8_t shift;

  line[0] = 'R';
  line[1] = 'E';
  line[2] = 'A';
  line[3] = 'D';
  line[4] = 'I';
  line[5] = 'D';
  line[6] = '=';
  line[7] = '0';
  line[8] = 'x';

  for (shift = 0U; shift < 6U; ++shift)
  {
    line[9U + shift] = hexDigits[(jedecId >> (20U - (shift * 4U))) & 0x0FU];
  }

  line[15] = '\0';
}

static uint8_t Boot_ShouldEnterAnimDownloadMode(void)
{
  HAL_Delay(20);
  return (HAL_GPIO_ReadPin(Key_OK_GPIO_Port, Key_OK_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
}

static void Boot_TryDownloadStartAnimViaCdc(void)
{
  int8_t dl_status;

  dl_status = QSPI_StartAnim_DownloadViaCdc(QSPI_START_ANIM_BASE_ADDR, 60000U);
  if (dl_status == QSPI_ANIM_LOADER_OK)
  {
    g_start_anim_download_status = 1;
  }
  else
  {
    g_start_anim_download_status = -1;
  }
}

static void LCD_ShowStartupScreen(void)
{
  uint16_t x;
  uint16_t y;
  char line1[] = "STM32H743ZIT6 KIT";
  char line2_dl_ok[] = "ANIM DOWNLOAD OK";
  char line2_dl_err[] = "ANIM DOWNLOAD FAIL";
  char line2_sd_ready[] = "SD ANIM READY";
  char line2_ready[] = "QSPI+ANIM READY";
  char *line2;
  char line3_ready[] = "Designed by JerryXie";
  char line3_sd_file[] = "FILE: STARTANI.BIN";
  char line3_readid[] = "READID=0x000000";
  char line3_anim_err[] = "CHECK W25Q64 DATA";
  char *line3 = line3_ready;

  if (g_sd_start_anim_status == SD_START_ANIM_OK)
  {
    line2 = line2_sd_ready;
    line3 = line3_sd_file;
  }
  else if (g_qspi_init_status == QSPI_W25QXX_OK)
  {
    if (g_start_anim_download_status > 0)
    {
      if (g_start_anim_status == QSPI_START_ANIM_OK)
      {
        line2 = line2_dl_ok;
      }
      else
      {
        line2 = (char *)QSPI_GetStartAnimErrorText(g_start_anim_status);
        line3 = line3_anim_err;
      }
    }
    else if (g_start_anim_download_status < 0)
    {
      line2 = line2_dl_err;
    }
    else if (g_start_anim_status == QSPI_START_ANIM_OK)
    {
      line2 = line2_ready;
    }
    else
    {
      line2 = (char *)QSPI_GetStartAnimErrorText(g_start_anim_status);
      line3 = line3_anim_err;
    }
  }
  else
  {
    line2 = (char *)SD_GetStartAnimErrorText(g_sd_start_anim_status);
    if (g_qspi_init_status == QSPI_W25QXX_OK)
    {
      line3 = line3_anim_err;
    }
    else
    {
      line3 = line3_readid;
      QSPI_FormatJedecId(line3, g_qspi_jedec_id);
    }
  }

  LCD_SetBackColor(LCD_BLACK);
  LCD_Clear();

  LCD_SetAsciiFont(&ASCII_Font16);
  LCD_SetColor(LCD_WHITE);
  LCD_DisplayString(10, 20, line1);
  LCD_DisplayString(10, 42, line2);
  LCD_DisplayString(10, 64, line3);
  LCD_SetColor(LCD_CYAN);
  LCD_DrawRect(6, 12, 228, 72);

  HAL_Delay(1500);

  LCD_Clear();

  x = (LCD_Width - LOGO_IMAGE_WIDTH) / 2;
  y = (LCD_Height - LOGO_IMAGE_HEIGHT) / 2;
  LCD_CopyBuffer(x, y, LOGO_IMAGE_WIDTH, LOGO_IMAGE_HEIGHT, g_logo_image_rgb565);
}

static void LCD_ShowDownloadScreen(void)
{
  LCD_SetBackColor(LCD_BLACK);
  LCD_Clear();

  LCD_SetAsciiFont(&ASCII_Font16);
  LCD_SetColor(LCD_CYAN);
  LCD_DisplayString(10, 18, "STM32H743ZIT6 KIT");

  LCD_SetColor(LCD_WHITE);
  LCD_DisplayString(10, 42, "USB CDC DOWNLOAD MODE");

  LCD_SetColor(LCD_YELLOW);
  LCD_DisplayString(10, 66, "OPEN COM8, SEND GIF BIN");

  LCD_SetColor(LCD_GREEN);
  LCD_DrawRect(6, 12, 228, 72);
}

static int8_t SD_RunFatFsRwSelfTest(void)
{
  const uint32_t mountRetryMax = 5U;
  static const char testFileName[] = "SDTEST.TXT";
  static const uint8_t writeData[] = "STM32H743 SDIO FatFs RW OK\r\n";
  uint8_t readData[sizeof(writeData)] = {0};
  FRESULT fresult;
  UINT bytesWritten = 0U;
  UINT bytesRead = 0U;
  uint8_t fileOpened = 0U;
  uint32_t retry;
  int8_t status = -1;

  g_sd_self_test_stage = SD_TEST_STAGE_INIT;
  g_sd_self_test_fresult = FR_OK;
  g_sd_detect_status = BSP_SD_IsDetected();
  g_sd_card_state = BSP_SD_GetCardState();
  g_sd_hal_error_code = hsd1.ErrorCode;

  g_sd_self_test_stage = SD_TEST_STAGE_MOUNT;
  fresult = FR_NOT_READY;
  for (retry = 0U; retry < mountRetryMax; ++retry)
  {
    fresult = f_mount(&SDFatFS, (TCHAR const *)SDPath, 1U);
    if (fresult == FR_OK)
    {
      break;
    }
    HAL_Delay(50U);
  }
  if (fresult != FR_OK)
  {
    g_sd_self_test_fresult = fresult;
    goto exit;
  }

  g_sd_self_test_stage = SD_TEST_STAGE_OPEN_WRITE;
  fresult = f_open(&SDFile, testFileName, FA_CREATE_ALWAYS | FA_WRITE);
  if (fresult != FR_OK)
  {
    g_sd_self_test_fresult = fresult;
    goto exit;
  }
  fileOpened = 1U;

  g_sd_self_test_stage = SD_TEST_STAGE_WRITE;
  fresult = f_write(&SDFile, writeData, (UINT)(sizeof(writeData) - 1U), &bytesWritten);
  if ((fresult != FR_OK) || (bytesWritten != (UINT)(sizeof(writeData) - 1U)))
  {
    g_sd_self_test_fresult = fresult;
    goto exit;
  }

  g_sd_self_test_stage = SD_TEST_STAGE_CLOSE_WRITE;
  fresult = f_close(&SDFile);
  fileOpened = 0U;
  if (fresult != FR_OK)
  {
    g_sd_self_test_fresult = fresult;
    goto exit;
  }

  g_sd_self_test_stage = SD_TEST_STAGE_OPEN_READ;
  fresult = f_open(&SDFile, testFileName, FA_READ);
  if (fresult != FR_OK)
  {
    g_sd_self_test_fresult = fresult;
    goto exit;
  }
  fileOpened = 1U;

  g_sd_self_test_stage = SD_TEST_STAGE_READ;
  fresult = f_read(&SDFile, readData, (UINT)(sizeof(writeData) - 1U), &bytesRead);
  if ((fresult != FR_OK) || (bytesRead != (UINT)(sizeof(writeData) - 1U)))
  {
    g_sd_self_test_fresult = fresult;
    goto exit;
  }

  g_sd_self_test_stage = SD_TEST_STAGE_VERIFY;
  if (memcmp(readData, writeData, sizeof(writeData) - 1U) != 0)
  {
    g_sd_self_test_fresult = FR_INVALID_OBJECT;
    goto exit;
  }

  g_sd_self_test_fresult = FR_OK;
  status = 0;

exit:
  g_sd_detect_status = BSP_SD_IsDetected();
  g_sd_card_state = BSP_SD_GetCardState();
  g_sd_hal_error_code = hsd1.ErrorCode;

  if (fileOpened != 0U)
  {
    (void)f_close(&SDFile);
  }
  (void)f_mount(NULL, (TCHAR const *)SDPath, 1U);
  return status;
}

static const char *SD_GetSelfTestStageText(uint8_t stage)
{
  switch (stage)
  {
    case SD_TEST_STAGE_INIT:
      return "init";
    case SD_TEST_STAGE_MOUNT:
      return "mount";
    case SD_TEST_STAGE_OPEN_WRITE:
      return "open_write";
    case SD_TEST_STAGE_WRITE:
      return "write";
    case SD_TEST_STAGE_CLOSE_WRITE:
      return "close_write";
    case SD_TEST_STAGE_OPEN_READ:
      return "open_read";
    case SD_TEST_STAGE_READ:
      return "read";
    case SD_TEST_STAGE_VERIFY:
      return "verify";
    default:
      return "unknown";
  }
}

static const char *SD_GetFresultText(FRESULT fr)
{
  switch (fr)
  {
    case FR_OK:
      return "FR_OK";
    case FR_DISK_ERR:
      return "FR_DISK_ERR";
    case FR_INT_ERR:
      return "FR_INT_ERR";
    case FR_NOT_READY:
      return "FR_NOT_READY";
    case FR_NO_FILE:
      return "FR_NO_FILE";
    case FR_NO_PATH:
      return "FR_NO_PATH";
    case FR_INVALID_NAME:
      return "FR_INVALID_NAME";
    case FR_DENIED:
      return "FR_DENIED";
    case FR_EXIST:
      return "FR_EXIST";
    case FR_INVALID_OBJECT:
      return "FR_INVALID_OBJECT";
    case FR_WRITE_PROTECTED:
      return "FR_WRITE_PROTECTED";
    case FR_INVALID_DRIVE:
      return "FR_INVALID_DRIVE";
    case FR_NOT_ENABLED:
      return "FR_NOT_ENABLED";
    case FR_NO_FILESYSTEM:
      return "FR_NO_FILESYSTEM";
    case FR_MKFS_ABORTED:
      return "FR_MKFS_ABORTED";
    case FR_TIMEOUT:
      return "FR_TIMEOUT";
    case FR_LOCKED:
      return "FR_LOCKED";
    case FR_NOT_ENOUGH_CORE:
      return "FR_NOT_ENOUGH_CORE";
    case FR_TOO_MANY_OPEN_FILES:
      return "FR_TOO_MANY_OPEN_FILES";
    case FR_INVALID_PARAMETER:
      return "FR_INVALID_PARAMETER";
    default:
      return "FR_UNKNOWN";
  }
}

static void Boot_TryReportSdSelfTestViaCdc(void)
{
  static uint8_t okMsg[] = "SD self-test OK: mount/write/read/verify passed.\r\n";
  char failMsg[224];
  int msgLen;

  if ((g_sd_self_test_done == 0U) || (g_sd_self_test_reported != 0U))
  {
    return;
  }

  if (g_sd_self_test_status == 0)
  {
    if (CDC_Transmit_FS(okMsg, (uint16_t)(sizeof(okMsg) - 1U)) == USBD_OK)
    {
      g_sd_self_test_reported = 1U;
    }
  }
  else
  {
    msgLen = snprintf(
      failMsg,
      sizeof(failMsg),
      "SD self-test FAIL: stage=%s fresult=%s(%d) detect=%u card=%u hal=0x%08lX%s\r\n",
      SD_GetSelfTestStageText(g_sd_self_test_stage),
      SD_GetFresultText(g_sd_self_test_fresult),
      (int)g_sd_self_test_fresult,
      (unsigned int)g_sd_detect_status,
      (unsigned int)g_sd_card_state,
      (unsigned long)g_sd_hal_error_code,
      (g_sd_self_test_fresult == FR_NO_FILESYSTEM) ? " hint=format FAT32" : ""
    );
    if ((msgLen > 0) && (CDC_Transmit_FS((uint8_t *)failMsg, (uint16_t)msgLen) == USBD_OK))
    {
      g_sd_self_test_reported = 1U;
    }
  }
}

static void SD_RunServiceDemo(void)
{
  static const char demoFile[] = "DEMO.TXT";
  static const char demoText[] = "STM32 SD service demo line\r\n";
  char readBack[96];
  uint32_t readLen = 0U;
  FRESULT fr;

  fr = SD_Service_WriteTextFile(demoFile, demoText);
  if (fr != FR_OK)
  {
    g_sd_service_demo_result = fr;
    return;
  }

  fr = SD_Service_ReadTextFile(demoFile, readBack, sizeof(readBack), &readLen);
  if (fr != FR_OK)
  {
    g_sd_service_demo_result = fr;
    return;
  }

  if ((readLen == 0U) || (strncmp(readBack, demoText, strlen(demoText)) != 0))
  {
    g_sd_service_demo_result = FR_INVALID_OBJECT;
    return;
  }

  fr = SD_Service_DeleteFile(demoFile);
  g_sd_service_demo_result = fr;
}

#if SD_BENCH_ENABLE
static void SD_RunBenchmarks(void)
{
  FRESULT fr;

  fr = SD_Service_BenchSequentialWrite(
    SD_BENCH_WRITE_FILE,
    SD_BENCH_WRITE_SIZE,
    SD_BENCH_WRITE_CHUNK,
    &g_sd_seq_write_result
  );
  if (fr != FR_OK)
  {
    g_sd_rand_read_result.fresult = fr;
    return;
  }

  fr = SD_Service_BenchRandomRead(
    SD_BENCH_WRITE_FILE,
    SD_BENCH_READ_COUNT,
    SD_BENCH_READ_CHUNK,
    &g_sd_rand_read_result
  );

  if (fr == FR_OK)
  {
    (void)SD_Service_DeleteFile(SD_BENCH_WRITE_FILE);
  }
}

static void Boot_TryReportSdBenchViaCdc(void)
{
  char msg[224];
  int msgLen;

  if ((g_sd_bench_done == 0U) || (g_sd_bench_reported != 0U))
  {
    return;
  }

  msgLen = snprintf(
    msg,
    sizeof(msg),
    "SD service=%s, seqW=%s %luB %lums %luB/s, randR=%s %luB %lums %luB/s\r\n",
    SD_GetFresultText(g_sd_service_demo_result),
    SD_GetFresultText(g_sd_seq_write_result.fresult),
    (unsigned long)g_sd_seq_write_result.bytes,
    (unsigned long)g_sd_seq_write_result.elapsed_ms,
    (unsigned long)g_sd_seq_write_result.bytes_per_sec,
    SD_GetFresultText(g_sd_rand_read_result.fresult),
    (unsigned long)g_sd_rand_read_result.bytes,
    (unsigned long)g_sd_rand_read_result.elapsed_ms,
    (unsigned long)g_sd_rand_read_result.bytes_per_sec
  );

  if ((msgLen > 0) && (CDC_Transmit_FS((uint8_t *)msg, (uint16_t)msgLen) == USBD_OK))
  {
    g_sd_bench_reported = 1U;
  }
}
#endif

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_BDMA_Init();
  MX_MDMA_Init();
  MX_DAC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_I2C4_Init();
  MX_SPI6_Init();
  MX_QUADSPI_Init();
  MX_USART1_UART_Init();
  MX_TIM15_Init();
  MX_UART4_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_UART5_Init();
  MX_ADC1_Init();
  MX_USB_DEVICE_Init();
  MX_FATFS_Init();
  MX_SDMMC1_SD_Init();
  MX_JPEG_Init();
  MX_TIM6_Init();
  MX_DMA2D_Init();
  MX_TIM7_Init();
  MX_TIM13_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  SPI_LCD_Init();
  Boot_LogStatus("BOOT: JPEG init=", g_jpeg_init_ok);
  Boot_LogStatus("BOOT: DMA2D init=", g_dma2d_init_ok);
  Boot_LogStatus("BOOT: TIM7 init=", g_tim7_init_ok);
  QSPI_BootInit();
  Boot_LogStatus("BOOT: QSPI init=", g_qspi_init_status);
  if (Boot_ShouldEnterAnimDownloadMode() == 1U)
  {
    LCD_ShowDownloadScreen();
    if (g_qspi_init_status == QSPI_W25QXX_OK)
    {
      Boot_TryDownloadStartAnimViaCdc();
    }
    else
    {
      char readIdLine[] = "READID=0x000000";

      QSPI_FormatJedecId(readIdLine, g_qspi_jedec_id);
      g_start_anim_download_status = -1;
      LCD_Clear();
      LCD_SetAsciiFont(&ASCII_Font16);
      LCD_SetColor(LCD_RED);
      LCD_DisplayString(10, 42, readIdLine);
      LCD_DisplayString(10, 66, (char *)QSPI_GetInitErrorText(g_qspi_init_status));
      HAL_Delay(1500);
    }
    if (g_start_anim_download_status > 0)
    {
      CDC_SetDownloadMode(0U);
    }
  }

  g_sd_start_anim_status = SD_StartAnim_Play();
  Boot_LogStatus("BOOT: SD status=", g_sd_start_anim_status);
  if (g_sd_start_anim_status == SD_START_ANIM_OK)
  {
    g_start_anim_status = QSPI_START_ANIM_OK;
  }
  else if (g_qspi_init_status == QSPI_W25QXX_OK)
  {
    Boot_LogText("BOOT: fallback to QSPI\r\n");
    /* Ensure LCD/SPI state is clean before fallback playback. */
    LCD_ResetTransferState();
    g_start_anim_status = QSPI_StartAnim_Play();
    Boot_LogStatus("BOOT: QSPI status=", g_start_anim_status);
  }
  LCD_ShowStartupScreen();

  lv_init();
  lv_port_disp_init();
  lv_port_indev_init();
  LVGL_App_Init();

  if (DCMotor_OL_Init() != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_Base_Start_IT(&htim13) != HAL_OK)
  {
    Error_Handler();
  }

  g_tim7_start_ok = 0U;
  if (g_tim7_init_ok != 0U)
  {
    if ((TIM7->CR1 & TIM_CR1_CEN) != 0U)
    {
      g_tim7_start_ok = 1U;
    }
    else if (HAL_TIM_Base_Start_IT(&htim7) == HAL_OK)
    {
      g_tim7_start_ok = 1U;
    }
    else if ((TIM7->CR1 & TIM_CR1_CEN) != 0U)
    {
      g_tim7_start_ok = 1U;
    }
  }
  Boot_LogStatus("BOOT: TIM7 start=", g_tim7_start_ok);

  HAL_TIM_Base_Start_IT(&htim16);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if SD_SELF_TEST_ENABLE
    if (g_sd_self_test_done == 0U)
    {
      g_sd_self_test_status = SD_RunFatFsRwSelfTest();
      g_sd_self_test_done = 1U;

      if (g_sd_self_test_status == 0)
      {
        SD_RunServiceDemo();
        g_sd_service_demo_done = 1U;
#if SD_BENCH_ENABLE
        SD_RunBenchmarks();
        g_sd_bench_done = 1U;
#endif
      }
    }
#endif

    LVGL_App_Process();

    if (g_cdc_welcome_sent == 0U)
    {
      uint32_t now = HAL_GetTick();
      static uint8_t cdc_welcome_msg[] = "USB CDC ready, type to echo.\r\n";

      if ((now - g_cdc_welcome_last_tick) >= 500U)
      {
        if (CDC_Transmit_FS(cdc_welcome_msg, (uint16_t)(sizeof(cdc_welcome_msg) - 1U)) == USBD_OK)
        {
          g_cdc_welcome_sent = 1U;
        }
        g_cdc_welcome_last_tick = now;
      }
    }

    Boot_DebugFlushStageLogsViaCdc();

#if SD_SELF_TEST_ENABLE
    Boot_TryReportSdSelfTestViaCdc();
#if SD_BENCH_ENABLE
    if ((g_sd_self_test_status == 0) && (g_sd_service_demo_done != 0U) && (g_sd_bench_done != 0U))
    {
      Boot_TryReportSdBenchViaCdc();
    }
#endif
#endif

    HAL_Delay(5);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_ENABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void)
{

  /* USER CODE BEGIN DMA2D_Init 0 */

  /* USER CODE END DMA2D_Init 0 */

  /* USER CODE BEGIN DMA2D_Init 1 */

  /* USER CODE END DMA2D_Init 1 */
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M_PFC;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_RGB565;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_YCBCR;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  hdma2d.LayerCfg[1].AlphaInverted = DMA2D_REGULAR_ALPHA;
  hdma2d.LayerCfg[1].RedBlueSwap = DMA2D_RB_REGULAR;
  hdma2d.LayerCfg[1].ChromaSubSampling = DMA2D_CSS_420;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DMA2D_Init 2 */

  /* USER CODE END DMA2D_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x307075B1;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x307075B1;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C4_Init(void)
{

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x307075B1;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */

}

/**
  * @brief JPEG Initialization Function
  * @param None
  * @retval None
  */
static void MX_JPEG_Init(void)
{

  /* USER CODE BEGIN JPEG_Init 0 */

  /* USER CODE END JPEG_Init 0 */

  /* USER CODE BEGIN JPEG_Init 1 */

  /* USER CODE END JPEG_Init 1 */
  hjpeg.Instance = JPEG;
  if (HAL_JPEG_Init(&hjpeg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN JPEG_Init 2 */

  /* USER CODE END JPEG_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 3;
  hqspi.Init.FifoThreshold = 32;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hqspi.Init.FlashSize = 22;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_3;
  hqspi.Init.FlashID = QSPI_FLASH_ID_1;
  hqspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 6;
  if (HAL_SD_Init(&hsd1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI6_Init(void)
{

  /* USER CODE BEGIN SPI6_Init 0 */

  /* USER CODE END SPI6_Init 0 */

  /* USER CODE BEGIN SPI6_Init 1 */

  /* USER CODE END SPI6_Init 1 */
  /* SPI6 parameter configuration*/
  hspi6.Instance = SPI6;
  hspi6.Init.Mode = SPI_MODE_MASTER;
  hspi6.Init.Direction = SPI_DIRECTION_1LINE;
  hspi6.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi6.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi6.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi6.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi6.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi6.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi6.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi6.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi6.Init.CRCPolynomial = 0x0;
  hspi6.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi6.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi6.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi6.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi6.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi6.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi6.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi6.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi6.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi6.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI6_Init 2 */

  /* USER CODE END SPI6_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 239;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 239;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 24000-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 332;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 239;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 19999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 2399;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 999;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 65535;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 24000-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 10000-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_BDMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_BDMA_CLK_ENABLE();

  /* DMA interrupt init */
  /* BDMA_Channel0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(BDMA_Channel0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(BDMA_Channel0_IRQn);

}

/**
  * Enable MDMA controller clock
  */
static void MX_MDMA_Init(void)
{

  /* MDMA controller clock enable */
  __HAL_RCC_MDMA_CLK_ENABLE();
  /* Local variables */

  /* MDMA interrupt initialization */
  /* MDMA_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MDMA_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(MDMA_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED1_Pin|LED2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAMERAPWDN_GPIO_Port, CAMERAPWDN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, M1_PH_Pin|M2_PH_Pin|M3_PH_Pin|M4_PH_Pin
                          |LCD_BL_Pin|LCD_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : Key_Up_Pin Key_Down_Pin Key_Left_Pin Key_Right_Pin
                           Key_OK_Pin Key1_Pin */
  GPIO_InitStruct.Pin = Key_Up_Pin|Key_Down_Pin|Key_Left_Pin|Key_Right_Pin
                          |Key_OK_Pin|Key1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SD_Pin Key2_Pin Key3_Pin */
  GPIO_InitStruct.Pin = SD_Pin|Key2_Pin|Key3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CAMERAPWDN_Pin */
  GPIO_InitStruct.Pin = CAMERAPWDN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CAMERAPWDN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : M1_PH_Pin M2_PH_Pin M3_PH_Pin M4_PH_Pin */
  GPIO_InitStruct.Pin = M1_PH_Pin|M2_PH_Pin|M3_PH_Pin|M4_PH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : LED0_Pin */
  GPIO_InitStruct.Pin = LED0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_BL_Pin LCD_DC_Pin */
  GPIO_InitStruct.Pin = LCD_BL_Pin|LCD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*AnalogSwitch Config */
  HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC2, SYSCFG_SWITCH_PC2_CLOSE);

  /*AnalogSwitch Config */
  HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC3, SYSCFG_SWITCH_PC3_CLOSE);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
    lv_tick_inc(1);
  }
  else if (htim->Instance == TIM7)
  {
    MJPEG_Scheduler_OnTim7Tick();
    g_tim7_frame_tick++;
  }
  else if (htim->Instance == TIM13)
  {
    DCMotor_OL_Tick10ms();
  }
  else if (htim->Instance == TIM16)
  {
    extern volatile uint8_t g_adc_update_flag;
    extern volatile float g_adc_voltage;
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
    {
      uint32_t val = HAL_ADC_GetValue(&hadc1);
      g_adc_voltage = ((float)val / 65535.0f) * 3.3f * 6.0f;
      g_adc_update_flag = 1;
    }
  }
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin,GPIO_PIN_RESET);
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
