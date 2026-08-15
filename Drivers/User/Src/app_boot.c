#include "app_boot.h"
#include "adc_sampler.h"
#include "app_context.h"
#include "app_hal_bridge.h"
#include "battery_monitor.h"
#include "bsp_driver_sd.h"
#include "camera_service.h"
#include "comm_service.h"
#include "dc_motor_ol.h"
#include "fatfs.h"
#include "foc_link.h"
#include "imu_service.h"
#include "lcd_spi_154.h"
#include "logo_image.h"
#include "lv_port_disp.h"
#include "lv_port_indev.h"
#include "lvgl.h"
#include "lvgl_app.h"
#include "mecanum_odometry.h"
#include "qspi_anim_loader.h"
#include "qspi_start_anim.h"
#include "qspi_w25q64.h"
#include "runtime_monitor.h"
#include "safety_manager.h"
#include "sd_bench_config.h"
#include "sd_service.h"
#include "sd_start_anim.h"
#include "ui_settings.h"
#include "usbd_cdc_if.h"
#include "ws2812.h"
#include <stdio.h>
#include <string.h>

#define STAGE_LOG_BUFFER_SIZE       2048U

#if SD_SELF_TEST_ENABLE
#define SD_TEST_STAGE_INIT          1U
#define SD_TEST_STAGE_MOUNT         2U
#define SD_TEST_STAGE_OPEN_WRITE    3U
#define SD_TEST_STAGE_WRITE         4U
#define SD_TEST_STAGE_CLOSE_WRITE   5U
#define SD_TEST_STAGE_OPEN_READ     6U
#define SD_TEST_STAGE_READ          7U
#define SD_TEST_STAGE_VERIFY        8U
#endif

#ifndef APP_CACHE_STAGE
#define APP_CACHE_STAGE             2U
#endif

static int8_t g_qspi_init_status = W25QXX_ERROR_INIT;
static uint32_t g_qspi_jedec_id;
static int8_t g_start_anim_status = QSPI_START_ANIM_ERR_HEADER;
static int8_t g_sd_start_anim_status = SD_START_ANIM_ERR_FILE;
static int8_t g_start_anim_download_status;
static uint8_t g_cdc_welcome_sent;
static uint32_t g_cdc_welcome_last_tick;
#if SD_SELF_TEST_ENABLE
static uint8_t g_sd_self_test_done;
static int8_t g_sd_self_test_status = -1;
static uint8_t g_sd_self_test_reported;
static uint8_t g_sd_self_test_stage;
static FRESULT g_sd_self_test_fresult = FR_OK;
static uint8_t g_sd_detect_status = SD_NOT_PRESENT;
static uint8_t g_sd_card_state = SD_TRANSFER_BUSY;
static uint32_t g_sd_hal_error_code;
static uint8_t g_sd_service_demo_done;
#if SD_BENCH_ENABLE
static uint8_t g_sd_bench_done;
static uint8_t g_sd_bench_reported;
static sd_bench_result_t g_sd_seq_write_result;
static sd_bench_result_t g_sd_rand_read_result;
#endif
static FRESULT g_sd_service_demo_result = FR_OK;
#endif
static char g_stage_log_buffer[STAGE_LOG_BUFFER_SIZE];
static uint16_t g_stage_log_len;
static uint16_t g_stage_log_flush_pos;
static uint8_t g_stage_log_overflow;
static uint8_t g_jpeg_init_ok;
static uint8_t g_dma2d_init_ok;
static uint8_t g_tim7_init_ok;
static uint8_t g_tim7_start_ok;

void AppBoot_EnableCache(void)
{
#if (APP_CACHE_STAGE >= 2U)
  MPU_Region_InitTypeDef region = {0};

  HAL_MPU_Disable();

  /* QSPI memory-mapped window: cached, read-only and never executable.  The
     NES ROM cache invalidates this range when its mapped lifetime changes. */
  region.Enable = MPU_REGION_ENABLE;
  region.Number = MPU_REGION_NUMBER0;
  region.BaseAddress = W25QXX_MEM_MAPPED_ADDR;
  region.Size = MPU_REGION_SIZE_8MB;
  region.SubRegionDisable = 0x00U;
  region.TypeExtField = MPU_TEX_LEVEL0;
  region.AccessPermission = MPU_REGION_PRIV_RO_URO;
  region.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  region.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  region.IsCacheable = MPU_ACCESS_CACHEABLE;
  region.IsBufferable = MPU_ACCESS_BUFFERABLE;
  HAL_MPU_ConfigRegion(&region);

  /* D2 SRAM is shared by SDMMC, USB and JPEG/MDMA.  Split its 288 KiB into
     MPU-compatible regions and keep it non-cacheable. */
  region.Enable = MPU_REGION_ENABLE;
  region.Number = MPU_REGION_NUMBER1;
  region.BaseAddress = 0x30000000U;
  region.Size = MPU_REGION_SIZE_256KB;
  region.SubRegionDisable = 0x00U;
  region.TypeExtField = MPU_TEX_LEVEL1;
  region.AccessPermission = MPU_REGION_FULL_ACCESS;
  region.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  region.IsShareable = MPU_ACCESS_SHAREABLE;
  region.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  region.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  HAL_MPU_ConfigRegion(&region);

  region.Number = MPU_REGION_NUMBER2;
  region.BaseAddress = 0x30040000U;
  region.Size = MPU_REGION_SIZE_32KB;
  HAL_MPU_ConfigRegion(&region);

  /* SPI6 BDMA staging lives in D3 SRAM4. */
  region.Number = MPU_REGION_NUMBER3;
  region.BaseAddress = 0x38000000U;
  region.Size = MPU_REGION_SIZE_64KB;
  HAL_MPU_ConfigRegion(&region);

  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
#endif

#if (APP_CACHE_STAGE >= 1U)
  SCB_EnableICache();
#endif
#if (APP_CACHE_STAGE >= 2U)
  SCB_EnableDCache();
#endif
}


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
    case QSPI_START_ANIM_ERR_BUSY:
      return "ANIM MEMORY BUSY";
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
    case SD_START_ANIM_ERR_BUSY:
      return "SD ANIM MEM BUSY";
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

#if SD_SELF_TEST_ENABLE
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
  g_sd_hal_error_code = ((AppContext_Get()->sd1 != NULL) ? AppContext_Get()->sd1->ErrorCode : 0U);

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
  g_sd_hal_error_code = ((AppContext_Get()->sd1 != NULL) ? AppContext_Get()->sd1->ErrorCode : 0U);

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

#endif /* SD_SELF_TEST_ENABLE */


void AppBoot_Run(void)
{
  const AppContext *context = AppContext_Get();

  if ((context->adc1 == NULL) || (context->iwdg1 == NULL) ||
      (context->tim6 == NULL) || (context->tim13 == NULL) ||
      (context->tim16 == NULL) || (context->uart5 == NULL))
  {
    Error_Handler();
  }

  g_jpeg_init_ok = context->jpeg_init_ok;
  g_dma2d_init_ok = context->dma2d_init_ok;
  g_tim7_init_ok = context->tim7_init_ok;

  RuntimeMonitor_AttachWatchdog(context->iwdg1);
  RuntimeMonitor_StackInit();
  RuntimeMonitor_BootProgress();
  Camera_Service_BootHold();

  AdcSampler_Init(context->adc1);
  (void)HAL_ADCEx_Calibration_Start(
      context->adc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
  CommService_Init();

  SPI_LCD_Init();
  Boot_LogStatus("BOOT: JPEG init=", g_jpeg_init_ok);
  Boot_LogStatus("BOOT: DMA2D init=", g_dma2d_init_ok);
  Boot_LogStatus("BOOT: TIM7 init=", g_tim7_init_ok);
  QSPI_BootInit();
  RuntimeMonitor_BootProgress();
  Boot_LogStatus("BOOT: QSPI init=", g_qspi_init_status);

  UI_Settings_Init((g_qspi_init_status == QSPI_W25QXX_OK) ? 1U : 0U);
  RuntimeMonitor_BootProgress();
  if (Boot_ShouldEnterAnimDownloadMode() == 1U)
  {
    LCD_ShowDownloadScreen();
    if (g_qspi_init_status == QSPI_W25QXX_OK)
    {
      Boot_TryDownloadStartAnimViaCdc();
    }
    else
    {
      char read_id_line[] = "READID=0x000000";

      QSPI_FormatJedecId(read_id_line, g_qspi_jedec_id);
      g_start_anim_download_status = -1;
      LCD_Clear();
      LCD_SetAsciiFont(&ASCII_Font16);
      LCD_SetColor(LCD_RED);
      LCD_DisplayString(10, 42, read_id_line);
      LCD_DisplayString(
          10, 66, (char *)QSPI_GetInitErrorText(g_qspi_init_status));
      HAL_Delay(1500);
    }
    if (g_start_anim_download_status > 0)
    {
      CDC_SetDownloadMode(0U);
    }
  }

  g_sd_start_anim_status = SD_StartAnim_Play();
  RuntimeMonitor_BootProgress();
  Boot_LogStatus("BOOT: SD status=", g_sd_start_anim_status);
  if (g_sd_start_anim_status == SD_START_ANIM_OK)
  {
    g_start_anim_status = QSPI_START_ANIM_OK;
  }
  else if (g_qspi_init_status == QSPI_W25QXX_OK)
  {
    Boot_LogText("BOOT: fallback to QSPI\r\n");
    LCD_ResetTransferState();
    g_start_anim_status = QSPI_StartAnim_Play();
    RuntimeMonitor_BootProgress();
    Boot_LogStatus("BOOT: QSPI status=", g_start_anim_status);
  }
  LCD_ShowStartupScreen();

  lv_init();
  lv_port_disp_init();
  lv_port_indev_init();
  LVGL_App_Init();
  RuntimeMonitor_BootProgress();
  IMU_Service_Init((g_qspi_init_status == QSPI_W25QXX_OK) ? 1U : 0U);
  RuntimeMonitor_BootProgress();

  if (DCMotor_OL_Init() != HAL_OK)
  {
    Error_Handler();
  }
  MecanumOdometry_Init();
  Safety_Init();
  BatteryMonitor_Init();

  /* Start continuous UART reception only after the blocking display/QSPI/SD
   * boot sequence has finished.  Starting it before the animation allowed the
   * ESP32 stream to fill the UART5 ring while the foreground could not drain
   * it, producing overflow, bad-frame and rearm warnings on every boot. */
  FOC_Link_Init();
  AppHalBridge_Init();

  if (HAL_TIM_Base_Start_IT(context->tim6) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_Base_Start_IT(context->tim13) != HAL_OK)
  {
    Error_Handler();
  }

  g_tim7_start_ok = 0U;
  if ((g_tim7_init_ok != 0U) && (context->tim7 != NULL))
  {
    if ((context->tim7->Instance->CR1 & TIM_CR1_CEN) != 0U)
    {
      g_tim7_start_ok = 1U;
    }
    else if (HAL_TIM_Base_Start_IT(context->tim7) == HAL_OK)
    {
      g_tim7_start_ok = 1U;
    }
    else if ((context->tim7->Instance->CR1 & TIM_CR1_CEN) != 0U)
    {
      g_tim7_start_ok = 1U;
    }
  }
  Boot_LogStatus("BOOT: TIM7 start=", g_tim7_start_ok);

  (void)HAL_TIM_Base_Start_IT(context->tim16);
  ws2812_Init();
  RuntimeMonitor_StartRuntime();
}

void AppBoot_ProcessPreUi(void)
{
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
}

void AppBoot_ProcessPostUi(void)
{
  uint32_t now = HAL_GetTick();

  if (g_cdc_welcome_sent == 0U)
  {
    static uint8_t welcome_message[] = "USB CDC ready, type to echo.\r\n";

    if ((now - g_cdc_welcome_last_tick) >= 500U)
    {
      if (CDC_Transmit_FS(
              welcome_message,
              (uint16_t)(sizeof(welcome_message) - 1U)) == USBD_OK)
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
  if ((g_sd_self_test_status == 0) &&
      (g_sd_service_demo_done != 0U) &&
      (g_sd_bench_done != 0U))
  {
    Boot_TryReportSdBenchViaCdc();
  }
#endif
#endif
}

uint8_t AppBoot_IsCdcReady(void)
{
  return g_cdc_welcome_sent;
}
