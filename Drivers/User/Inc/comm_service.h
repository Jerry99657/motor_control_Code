#ifndef COMM_SERVICE_H
#define COMM_SERVICE_H

#include "main.h"
#include <stdint.h>

typedef struct
{
    uint32_t uart_rx_byte_count;
    uint32_t uart_rx_overflow_count;
    uint32_t uart_rx_error_count;
    uint32_t uart_rx_rearm_error_count;
    uint32_t uart_last_error_code;
    uint32_t uart_tx_queued_count;
    uint32_t uart_tx_complete_count;
    uint32_t uart_tx_drop_count;
    uint32_t uart_tx_start_error_count;
    uint32_t uart_stream_tx_start_count;
    uint32_t uart_stream_tx_complete_count;
    uint32_t uart_stream_tx_start_error_count;
    uint32_t usb_rx_byte_count;
    uint32_t usb_rx_overflow_count;
    uint32_t usb_tx_queued_count;
    uint32_t usb_tx_complete_count;
    uint32_t usb_tx_drop_count;
    uint32_t usb_tx_start_error_count;
} CommServiceStats;

void CommService_Init(void);
void CommService_Process(void);

void CommService_UartRxByteFromISR(uint8_t byte);
void CommService_UartTxCompleteFromISR(UART_HandleTypeDef *huart);
void CommService_UartErrorFromISR(UART_HandleTypeDef *huart);
void CommService_UartRxRearmFailedFromISR(void);
uint8_t CommService_UartSend(const uint8_t *data, uint16_t length);

/* Start one low-priority bulk DMA block on UART5. Normal command responses
 * already queued by CommService_UartSend() always win arbitration. The caller
 * must keep data valid until CommService_UartStreamBusy() becomes zero. */
uint8_t CommService_UartStreamSend(const uint8_t *data, uint16_t length);
uint8_t CommService_UartStreamBusy(void);

void CommService_GetStats(CommServiceStats *stats);

#endif /* COMM_SERVICE_H */
