#ifndef MJPEG_SCHEDULER_H
#define MJPEG_SCHEDULER_H

#include <stdint.h>
#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

HAL_StatusTypeDef MJPEG_Scheduler_SetFrameIntervalMs(uint32_t frame_interval_ms);
void MJPEG_Scheduler_OnTim7Tick(void);
uint8_t MJPEG_Scheduler_ConsumeFrameTick(void);
uint32_t MJPEG_Scheduler_GetFrameTickCount(void);

#ifdef __cplusplus
}
#endif

#endif /* MJPEG_SCHEDULER_H */
