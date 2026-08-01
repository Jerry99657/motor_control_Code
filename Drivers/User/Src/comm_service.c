#include "comm_service.h"
#include "usbd_cdc_if.h"
#include "lvgl_app.h"
#include <string.h>

#define COMM_UART_RX_BUFFER_SIZE 256U
#define COMM_UART_TX_QUEUE_DEPTH 8U
#define COMM_UART_TX_PACKET_SIZE 32U
#define COMM_PROCESS_CHUNK_SIZE  64U

typedef struct
{
    uint16_t length;
    uint8_t data[COMM_UART_TX_PACKET_SIZE];
} CommUartTxPacket_t;

extern UART_HandleTypeDef huart5;
extern void vofa_usb_rx_cb(uint8_t *buf, uint32_t len);

static uint8_t s_uart_rx_buffer[COMM_UART_RX_BUFFER_SIZE];
static volatile uint16_t s_uart_rx_head = 0U;
static volatile uint16_t s_uart_rx_tail = 0U;
static volatile uint32_t s_uart_rx_overflow_count = 0U;

static CommUartTxPacket_t s_uart_tx_queue[COMM_UART_TX_QUEUE_DEPTH];
static volatile uint8_t s_uart_tx_head = 0U;
static volatile uint8_t s_uart_tx_tail = 0U;
static volatile uint8_t s_uart_tx_busy = 0U;
static volatile uint32_t s_uart_tx_drop_count = 0U;

static uint32_t comm_lock(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return primask;
}

static void comm_unlock(uint32_t primask)
{
    __DMB();
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static void comm_uart_tx_kick(void)
{
    uint8_t index;
    uint16_t length;
    uint32_t primask = comm_lock();

    if ((s_uart_tx_busy != 0U) || (s_uart_tx_tail == s_uart_tx_head))
    {
        comm_unlock(primask);
        return;
    }

    index = s_uart_tx_tail;
    length = s_uart_tx_queue[index].length;
    s_uart_tx_busy = 1U;
    comm_unlock(primask);

    if (HAL_UART_Transmit_IT(&huart5, s_uart_tx_queue[index].data, length) != HAL_OK)
    {
        primask = comm_lock();
        s_uart_tx_busy = 0U;
        comm_unlock(primask);
    }
}

void CommService_Init(void)
{
    uint32_t primask = comm_lock();

    s_uart_rx_head = 0U;
    s_uart_rx_tail = 0U;
    s_uart_rx_overflow_count = 0U;
    s_uart_tx_head = 0U;
    s_uart_tx_tail = 0U;
    s_uart_tx_busy = 0U;
    s_uart_tx_drop_count = 0U;

    comm_unlock(primask);
}

void CommService_UartRxByteFromISR(uint8_t byte)
{
    uint16_t next = (uint16_t)(s_uart_rx_head + 1U);

    if (next >= COMM_UART_RX_BUFFER_SIZE)
    {
        next = 0U;
    }

    if (next == s_uart_rx_tail)
    {
        s_uart_rx_overflow_count++;
        return;
    }

    s_uart_rx_buffer[s_uart_rx_head] = byte;
    s_uart_rx_head = next;
}

uint8_t CommService_UartSend(const uint8_t *data, uint16_t length)
{
    uint8_t next;
    uint8_t index;
    uint32_t primask;

    if ((data == NULL) || (length == 0U) || (length > COMM_UART_TX_PACKET_SIZE))
    {
        return 0U;
    }

    primask = comm_lock();
    next = (uint8_t)(s_uart_tx_head + 1U);
    if (next >= COMM_UART_TX_QUEUE_DEPTH)
    {
        next = 0U;
    }

    if (next == s_uart_tx_tail)
    {
        s_uart_tx_drop_count++;
        comm_unlock(primask);
        return 0U;
    }

    index = s_uart_tx_head;
    memcpy(s_uart_tx_queue[index].data, data, length);
    s_uart_tx_queue[index].length = length;
    s_uart_tx_head = next;
    comm_unlock(primask);

    comm_uart_tx_kick();
    return 1U;
}

void CommService_UartTxCompleteFromISR(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart->Instance != UART5))
    {
        return;
    }

    if (s_uart_tx_busy != 0U)
    {
        uint8_t next = (uint8_t)(s_uart_tx_tail + 1U);
        if (next >= COMM_UART_TX_QUEUE_DEPTH)
        {
            next = 0U;
        }
        s_uart_tx_tail = next;
        s_uart_tx_busy = 0U;
    }

    comm_uart_tx_kick();
}

void CommService_Process(void)
{
    uint8_t buffer[COMM_PROCESS_CHUNK_SIZE];
    uint32_t length = 0U;

    while ((length < sizeof(buffer)) && (s_uart_rx_tail != s_uart_rx_head))
    {
        buffer[length++] = s_uart_rx_buffer[s_uart_rx_tail];
        s_uart_rx_tail++;
        if (s_uart_rx_tail >= COMM_UART_RX_BUFFER_SIZE)
        {
            s_uart_rx_tail = 0U;
        }
    }

    if (length > 0U)
    {
        lvgl_app_com_rx_channel_cb(0U, buffer, length);
    }

    length = CDC_ReadAppBytes(buffer, sizeof(buffer));
    if (length > 0U)
    {
        lvgl_app_com_rx_channel_cb(1U, buffer, length);
        vofa_usb_rx_cb(buffer, length);
    }

    CDC_TxService();
    comm_uart_tx_kick();
}

uint32_t CommService_GetUartRxOverflowCount(void)
{
    return s_uart_rx_overflow_count;
}

uint32_t CommService_GetUartTxDropCount(void)
{
    return s_uart_tx_drop_count;
}
