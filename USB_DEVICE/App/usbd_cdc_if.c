/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v1.0_Cube
  * @brief          : Usb device for Virtual Com Port.
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
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */
#include "main.h"
#include "runtime_monitor.h"
#include <string.h>

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_CDC_IF
  * @{
  */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

typedef struct
{
  uint16_t length;
  uint8_t data[APP_TX_DATA_SIZE];
} CDC_TxPacket_t;

typedef struct
{
  uint16_t length;
  uint8_t data[128U];
} CDC_LatestPacket_t;

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */
#define CDC_APP_RX_BUFFER_SIZE 512U
#define CDC_TX_QUEUE_DEPTH     3U
#define CDC_TX_PACKET_SIZE     APP_TX_DATA_SIZE
#define CDC_TX_BUSY_NONE       0U
#define CDC_TX_BUSY_QUEUE      1U
#define CDC_TX_BUSY_LATEST     2U
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
  * @brief Private variables.
  * @{
  */
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */
static uint8_t g_line_coding[7] = {0x00U, 0xC2U, 0x01U, 0x00U, 0x00U, 0x00U, 0x08U};
static volatile uint8_t g_cdc_download_mode = 0U;
static volatile uint32_t g_cdc_rx_head = 0U;
static volatile uint32_t g_cdc_rx_tail = 0U;
static volatile uint8_t g_cdc_rx_overflow = 0U;
static uint8_t g_cdc_rx_buffer[131072U];
static volatile uint16_t g_cdc_app_rx_head = 0U;
static volatile uint16_t g_cdc_app_rx_tail = 0U;
static volatile uint32_t g_cdc_app_rx_overflow_count = 0U;
static volatile uint32_t g_cdc_app_rx_byte_count = 0U;
static uint8_t g_cdc_app_rx_buffer[CDC_APP_RX_BUFFER_SIZE]
  __attribute__((section(".ram_d2"), aligned(32)));
static CDC_TxPacket_t g_cdc_tx_queue[CDC_TX_QUEUE_DEPTH]
  __attribute__((section(".ram_d2"), aligned(32)));
static volatile uint8_t g_cdc_tx_head = 0U;
static volatile uint8_t g_cdc_tx_tail = 0U;
static volatile uint8_t g_cdc_tx_busy = 0U;
static volatile uint8_t g_cdc_tx_busy_kind = CDC_TX_BUSY_NONE;
static volatile uint8_t g_cdc_latest_pending = 0U;
static CDC_LatestPacket_t g_cdc_latest_pending_packet
  __attribute__((section(".ram_d2"), aligned(32)));
static CDC_LatestPacket_t g_cdc_latest_active_packet
  __attribute__((section(".ram_d2"), aligned(32)));
static volatile uint32_t g_cdc_tx_drop_count = 0U;
static volatile uint32_t g_cdc_tx_queued_count = 0U;
static volatile uint32_t g_cdc_tx_complete_count = 0U;
static volatile uint32_t g_cdc_tx_start_error_count = 0U;

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */
static void CDC_RxPush(uint8_t byte);
static void CDC_AppRxPush(uint8_t byte);
static void CDC_TxKick(void);

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS,
  CDC_TransmitCplt_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the CDC media low layer over the FS USB IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{
  /* USER CODE BEGIN 3 */
  g_cdc_app_rx_head = 0U;
  g_cdc_app_rx_tail = 0U;
  g_cdc_tx_head = 0U;
  g_cdc_tx_tail = 0U;
  g_cdc_tx_busy = 0U;
  g_cdc_tx_busy_kind = CDC_TX_BUSY_NONE;
  g_cdc_latest_pending = 0U;
  g_cdc_app_rx_overflow_count = 0U;
  g_cdc_app_rx_byte_count = 0U;
  g_cdc_tx_drop_count = 0U;
  g_cdc_tx_queued_count = 0U;
  g_cdc_tx_complete_count = 0U;
  g_cdc_tx_start_error_count = 0U;
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */
  g_cdc_tx_head = 0U;
  g_cdc_tx_tail = 0U;
  g_cdc_tx_busy = 0U;
  g_cdc_tx_busy_kind = CDC_TX_BUSY_NONE;
  g_cdc_latest_pending = 0U;
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 5 */
  switch(cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

    case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

    case CDC_SET_COMM_FEATURE:

    break;

    case CDC_GET_COMM_FEATURE:

    break;

    case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
    case CDC_SET_LINE_CODING:
      if ((pbuf != NULL) && (length >= sizeof(g_line_coding)))
      {
        (void)memcpy(g_line_coding, pbuf, sizeof(g_line_coding));
      }

    break;

    case CDC_GET_LINE_CODING:
      if ((pbuf != NULL) && (length >= sizeof(g_line_coding)))
      {
        (void)memcpy(pbuf, g_line_coding, sizeof(g_line_coding));
      }

    break;

    case CDC_SET_CONTROL_LINE_STATE:

    break;

    case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint until exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  uint32_t rx_len = 0U;
  uint32_t index;

  if ((Buf != NULL) && (Len != NULL))
  {
    rx_len = *Len;
    if (rx_len > APP_TX_DATA_SIZE)
    {
      rx_len = APP_TX_DATA_SIZE;
    }

    if ((rx_len > 0U) && (g_cdc_download_mode != 0U))
    {
      for (index = 0U; index < rx_len; ++index)
      {
        CDC_RxPush(Buf[index]);
      }
    }
    else
    {
      g_cdc_app_rx_byte_count += rx_len;
      for (index = 0U; index < rx_len; ++index)
      {
        CDC_AppRxPush(Buf[index]);
      }
    }
  }

  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
  uint8_t next;
  uint8_t queue_index;
  uint32_t primask;

  if ((Buf == NULL) || (Len == 0U) || (Len > CDC_TX_PACKET_SIZE))
  {
    return USBD_FAIL;
  }

  if (hUsbDeviceFS.pClassData == NULL)
  {
    return USBD_FAIL;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  next = (uint8_t)(g_cdc_tx_head + 1U);
  if (next >= CDC_TX_QUEUE_DEPTH)
  {
    next = 0U;
  }

  if (next == g_cdc_tx_tail)
  {
    g_cdc_tx_drop_count++;
    if (primask == 0U)
    {
      __enable_irq();
    }
    return USBD_BUSY;
  }

  queue_index = g_cdc_tx_head;
  memcpy(g_cdc_tx_queue[queue_index].data, Buf, Len);
  g_cdc_tx_queue[queue_index].length = Len;
  __DMB();
  g_cdc_tx_head = next;
  g_cdc_tx_queued_count++;
  if (primask == 0U)
  {
    __enable_irq();
  }

  CDC_TxKick();
  /* USER CODE END 7 */
  return result;
}

/**
  * @brief  CDC_TransmitCplt_FS
  *         Data transmitted callback
  *
  *         @note
  *         This function is IN transfer complete callback used to inform user that
  *         the submitted Data is successfully sent over USB.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 13 */
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);
  if (g_cdc_tx_busy != 0U)
  {
    if (g_cdc_tx_busy_kind == CDC_TX_BUSY_QUEUE)
    {
      uint8_t next = (uint8_t)(g_cdc_tx_tail + 1U);
      if (next >= CDC_TX_QUEUE_DEPTH)
      {
        next = 0U;
      }
      g_cdc_tx_tail = next;
    }
    g_cdc_tx_busy = 0U;
    g_cdc_tx_busy_kind = CDC_TX_BUSY_NONE;
    g_cdc_tx_complete_count++;
  }
  CDC_TxKick();
  /* USER CODE END 13 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
static void CDC_RxPush(uint8_t byte)
{
  uint32_t next = g_cdc_rx_head + 1U;

  if (next >= (uint32_t)sizeof(g_cdc_rx_buffer))
  {
    next = 0U;
  }

  if (next == g_cdc_rx_tail)
  {
    g_cdc_rx_overflow = 1U;
    return;
  }

  g_cdc_rx_buffer[g_cdc_rx_head] = byte;
  g_cdc_rx_head = next;
}

static void CDC_AppRxPush(uint8_t byte)
{
  uint16_t next = (uint16_t)(g_cdc_app_rx_head + 1U);

  if (next >= CDC_APP_RX_BUFFER_SIZE)
  {
    next = 0U;
  }

  if (next == g_cdc_app_rx_tail)
  {
    g_cdc_app_rx_overflow_count++;
    return;
  }

  g_cdc_app_rx_buffer[g_cdc_app_rx_head] = byte;
  g_cdc_app_rx_head = next;
}

static void CDC_TxKick(void)
{
  uint8_t *buffer;
  uint8_t busy_kind;
  uint16_t length;
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  if ((g_cdc_tx_busy != 0U) || (hUsbDeviceFS.pClassData == NULL) ||
      ((g_cdc_latest_pending == 0U) &&
       (g_cdc_tx_tail == g_cdc_tx_head)))
  {
    if (primask == 0U)
    {
      __enable_irq();
    }
    return;
  }

  if (g_cdc_latest_pending != 0U)
  {
    length = g_cdc_latest_pending_packet.length;
    memcpy(g_cdc_latest_active_packet.data,
           g_cdc_latest_pending_packet.data, length);
    g_cdc_latest_active_packet.length = length;
    g_cdc_latest_pending = 0U;
    buffer = g_cdc_latest_active_packet.data;
    busy_kind = CDC_TX_BUSY_LATEST;
  }
  else
  {
    uint8_t index = g_cdc_tx_tail;
    length = g_cdc_tx_queue[index].length;
    buffer = g_cdc_tx_queue[index].data;
    busy_kind = CDC_TX_BUSY_QUEUE;
  }

  g_cdc_tx_busy = 1U;
  g_cdc_tx_busy_kind = busy_kind;
  if (primask == 0U)
  {
    __enable_irq();
  }

  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, buffer, length);
  if (USBD_CDC_TransmitPacket(&hUsbDeviceFS) != USBD_OK)
  {
    primask = __get_PRIMASK();
    __disable_irq();
    if ((busy_kind == CDC_TX_BUSY_LATEST) &&
        (g_cdc_latest_pending == 0U))
    {
      memcpy(g_cdc_latest_pending_packet.data,
             g_cdc_latest_active_packet.data, length);
      g_cdc_latest_pending_packet.length = length;
      g_cdc_latest_pending = 1U;
    }
    g_cdc_tx_busy = 0U;
    g_cdc_tx_busy_kind = CDC_TX_BUSY_NONE;
    g_cdc_tx_start_error_count++;
    if (primask == 0U)
    {
      __enable_irq();
    }
  }
}

uint32_t CDC_ReadAppBytes(uint8_t *buf, uint32_t len)
{
  uint32_t count = 0U;

  if ((buf == NULL) || (len == 0U))
  {
    return 0U;
  }

  while ((count < len) && (g_cdc_app_rx_tail != g_cdc_app_rx_head))
  {
    buf[count++] = g_cdc_app_rx_buffer[g_cdc_app_rx_tail];
    g_cdc_app_rx_tail++;
    if (g_cdc_app_rx_tail >= CDC_APP_RX_BUFFER_SIZE)
    {
      g_cdc_app_rx_tail = 0U;
    }
  }

  return count;
}

uint8_t CDC_TransmitLatest_FS(const uint8_t *buf, uint16_t len)
{
  uint32_t primask;

  if ((buf == NULL) || (len == 0U) ||
      (len > sizeof(g_cdc_latest_pending_packet.data)))
  {
    return USBD_FAIL;
  }

  if (hUsbDeviceFS.pClassData == NULL)
  {
    return USBD_FAIL;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  if (g_cdc_latest_pending == 0U)
  {
    g_cdc_tx_queued_count++;
  }
  memcpy(g_cdc_latest_pending_packet.data, buf, len);
  g_cdc_latest_pending_packet.length = len;
  __DMB();
  g_cdc_latest_pending = 1U;
  if (primask == 0U)
  {
    __enable_irq();
  }

  CDC_TxKick();
  return USBD_OK;
}

void CDC_TxService(void)
{
  CDC_TxKick();
}

void CDC_GetAppStats(CDC_AppStats *stats)
{
  uint32_t primask;

  if (stats == NULL)
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  stats->app_rx_byte_count = g_cdc_app_rx_byte_count;
  stats->app_rx_overflow_count = g_cdc_app_rx_overflow_count;
  stats->tx_queued_count = g_cdc_tx_queued_count;
  stats->tx_complete_count = g_cdc_tx_complete_count;
  stats->tx_drop_count = g_cdc_tx_drop_count;
  stats->tx_start_error_count = g_cdc_tx_start_error_count;
  if (primask == 0U)
  {
    __enable_irq();
  }
}

void CDC_SetDownloadMode(uint8_t enable)
{
  __disable_irq();
  g_cdc_download_mode = (enable != 0U) ? 1U : 0U;
  if (g_cdc_download_mode != 0U)
  {
    g_cdc_rx_head = 0U;
    g_cdc_rx_tail = 0U;
    g_cdc_rx_overflow = 0U;
  }
  __enable_irq();
}

uint8_t CDC_GetAndClearRxOverflow(void)
{
  uint8_t overflow;

  __disable_irq();
  overflow = g_cdc_rx_overflow;
  g_cdc_rx_overflow = 0U;
  __enable_irq();

  return overflow;
}

uint32_t CDC_ReadBytes(uint8_t *buf, uint32_t len, uint32_t timeout_ms)
{
  uint32_t read_len = 0U;
  uint32_t start_tick;

  if ((buf == NULL) || (len == 0U))
  {
    return 0U;
  }

  start_tick = HAL_GetTick();
  while (read_len < len)
  {
    uint8_t has_data = 0U;

    __disable_irq();
    if (g_cdc_rx_head != g_cdc_rx_tail)
    {
      buf[read_len] = g_cdc_rx_buffer[g_cdc_rx_tail];
      g_cdc_rx_tail++;
      if (g_cdc_rx_tail >= (uint32_t)sizeof(g_cdc_rx_buffer))
      {
        g_cdc_rx_tail = 0U;
      }
      has_data = 1U;
    }
    __enable_irq();

    if (has_data != 0U)
    {
      read_len++;
      RuntimeMonitor_BootProgress();
      continue;
    }

    if ((HAL_GetTick() - start_tick) >= timeout_ms)
    {
      break;
    }
    RuntimeMonitor_BootProgress();
  }

  return read_len;
}

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */
