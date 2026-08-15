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

void CommService_GetStats(CommServiceStats *stats);

#endif /* COMM_SERVICE_H */
