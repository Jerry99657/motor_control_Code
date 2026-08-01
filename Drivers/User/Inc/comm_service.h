#ifndef COMM_SERVICE_H
#define COMM_SERVICE_H

#include "main.h"
#include <stdint.h>

void CommService_Init(void);
void CommService_Process(void);

void CommService_UartRxByteFromISR(uint8_t byte);
void CommService_UartTxCompleteFromISR(UART_HandleTypeDef *huart);
uint8_t CommService_UartSend(const uint8_t *data, uint16_t length);

uint32_t CommService_GetUartRxOverflowCount(void);
uint32_t CommService_GetUartTxDropCount(void);

#endif /* COMM_SERVICE_H */
