#ifndef APP_HAL_BRIDGE_H
#define APP_HAL_BRIDGE_H

#include <stdint.h>

void AppHalBridge_Init(void);
void AppHalBridge_Process(void);
uint32_t AppHalBridge_GetTim7FrameTick(void);

#endif /* APP_HAL_BRIDGE_H */
