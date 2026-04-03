#ifndef QSPI_ANIM_LOADER_H
#define QSPI_ANIM_LOADER_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define QSPI_ANIM_LOADER_OK           0
#define QSPI_ANIM_LOADER_ERR_PARAM   -1
#define QSPI_ANIM_LOADER_ERR_TIMEOUT -2
#define QSPI_ANIM_LOADER_ERR_HEADER  -3
#define QSPI_ANIM_LOADER_ERR_QSPI    -4
#define QSPI_ANIM_LOADER_ERR_CRC     -5
#define QSPI_ANIM_LOADER_ERR_UART    -6

int8_t QSPI_StartAnim_DownloadViaUart(UART_HandleTypeDef *huart, uint32_t base_addr, uint32_t header_timeout_ms);
int8_t QSPI_StartAnim_DownloadViaCdc(uint32_t base_addr, uint32_t header_timeout_ms);

#ifdef __cplusplus
}
#endif

#endif
