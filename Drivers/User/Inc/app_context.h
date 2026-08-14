#ifndef APP_CONTEXT_H
#define APP_CONTEXT_H

#include "main.h"
#include <stdint.h>

typedef struct
{
    ADC_HandleTypeDef *adc1;
    IWDG_HandleTypeDef *iwdg1;
    SD_HandleTypeDef *sd1;
    TIM_HandleTypeDef *tim6;
    TIM_HandleTypeDef *tim7;
    TIM_HandleTypeDef *tim13;
    TIM_HandleTypeDef *tim16;
    UART_HandleTypeDef *uart4;
    UART_HandleTypeDef *uart5;
    uint8_t jpeg_init_ok;
    uint8_t dma2d_init_ok;
    uint8_t tim7_init_ok;
} AppContext;

void AppContext_Init(const AppContext *context);
const AppContext *AppContext_Get(void);

#endif /* APP_CONTEXT_H */
