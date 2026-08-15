#include "comm_service.h"
#include "app_event.h"
#include "usbd_cdc_if.h"
#include <string.h>

#define COMM_UART_RX_BUFFER_SIZE 256U
#define COMM_UART_TX_QUEUE_DEPTH 8U
#define COMM_UART_TX_PACKET_SIZE 32U
#define COMM_PROCESS_CHUNK_SIZE  64U

_Static_assert(COMM_PROCESS_CHUNK_SIZE <= APP_EVENT_PAYLOAD_CAPACITY,
               "Communication chunk must fit in one application event");

typedef struct
{
    uint16_t length;
    uint8_t data[COMM_UART_TX_PACKET_SIZE];
} CommUartTxPacket_t;

extern UART_HandleTypeDef huart5;
static uint8_t s_uart_rx_buffer[COMM_UART_RX_BUFFER_SIZE];
static volatile uint16_t s_uart_rx_head = 0U;
static volatile uint16_t s_uart_rx_tail = 0U;
static volatile uint32_t s_uart_rx_overflow_count = 0U;
static volatile uint32_t s_uart_rx_byte_count = 0U;
static volatile uint32_t s_uart_rx_error_count = 0U;
static volatile uint32_t s_uart_rx_rearm_error_count = 0U;
static volatile uint32_t s_uart_last_error_code = HAL_UART_ERROR_NONE;

static CommUartTxPacket_t s_uart_tx_queue[COMM_UART_TX_QUEUE_DEPTH];
static volatile uint8_t s_uart_tx_head = 0U;
static volatile uint8_t s_uart_tx_tail = 0U;
static volatile uint8_t s_uart_tx_busy = 0U;
static volatile uint32_t s_uart_tx_drop_count = 0U;
static volatile uint32_t s_uart_tx_queued_count = 0U;
static volatile uint32_t s_uart_tx_complete_count = 0U;
static volatile uint32_t s_uart_tx_start_error_count = 0U;

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
        s_uart_tx_start_error_count++;
        comm_unlock(primask);
    }
}

void CommService_Init(void)
{
    uint32_t primask = comm_lock();

    s_uart_rx_head = 0U;
    s_uart_rx_tail = 0U;
    s_uart_rx_overflow_count = 0U;
    s_uart_rx_byte_count = 0U;
    s_uart_rx_error_count = 0U;
    s_uart_rx_rearm_error_count = 0U;
    s_uart_last_error_code = HAL_UART_ERROR_NONE;
    s_uart_tx_head = 0U;
    s_uart_tx_tail = 0U;
    s_uart_tx_busy = 0U;
    s_uart_tx_drop_count = 0U;
    s_uart_tx_queued_count = 0U;
    s_uart_tx_complete_count = 0U;
    s_uart_tx_start_error_count = 0U;

    comm_unlock(primask);
}

void CommService_UartRxByteFromISR(uint8_t byte)
{
    uint16_t next = (uint16_t)(s_uart_rx_head + 1U);

    s_uart_rx_byte_count++;

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
    s_uart_tx_queued_count++;
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
        s_uart_tx_complete_count++;
    }

    comm_uart_tx_kick();
}

void CommService_UartErrorFromISR(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart->Instance != UART5))
    {
        return;
    }

    s_uart_rx_error_count++;
    s_uart_last_error_code = huart->ErrorCode;
}

void CommService_UartRxRearmFailedFromISR(void)
{
    s_uart_rx_rearm_error_count++;
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
        (void)AppEvent_PostCommRx(0U, buffer, (uint16_t)length);
    }

    length = CDC_ReadAppBytes(buffer, sizeof(buffer));
    if (length > 0U)
    {
        (void)AppEvent_PostCommRx(1U, buffer, (uint16_t)length);
    }

    CDC_TxService();
    comm_uart_tx_kick();
}

void CommService_GetStats(CommServiceStats *stats)
{
    CDC_AppStats usb_stats;
    uint32_t primask;

    if (stats == NULL)
    {
        return;
    }

    primask = comm_lock();
    stats->uart_rx_byte_count = s_uart_rx_byte_count;
    stats->uart_rx_overflow_count = s_uart_rx_overflow_count;
    stats->uart_rx_error_count = s_uart_rx_error_count;
    stats->uart_rx_rearm_error_count = s_uart_rx_rearm_error_count;
    stats->uart_last_error_code = s_uart_last_error_code;
    stats->uart_tx_queued_count = s_uart_tx_queued_count;
    stats->uart_tx_complete_count = s_uart_tx_complete_count;
    stats->uart_tx_drop_count = s_uart_tx_drop_count;
    stats->uart_tx_start_error_count = s_uart_tx_start_error_count;
    comm_unlock(primask);

    CDC_GetAppStats(&usb_stats);
    stats->usb_rx_byte_count = usb_stats.app_rx_byte_count;
    stats->usb_rx_overflow_count = usb_stats.app_rx_overflow_count;
    stats->usb_tx_queued_count = usb_stats.tx_queued_count;
    stats->usb_tx_complete_count = usb_stats.tx_complete_count;
    stats->usb_tx_drop_count = usb_stats.tx_drop_count;
    stats->usb_tx_start_error_count = usb_stats.tx_start_error_count;
}
