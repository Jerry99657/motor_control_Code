#ifndef SAFETY_MANAGER_H
#define SAFETY_MANAGER_H

#include "main.h"
#include <stdint.h>

typedef enum
{
    SAFETY_FAULT_NONE = 0U,
    SAFETY_FAULT_ESTOP = (1UL << 0),
    SAFETY_FAULT_IMU_STALE = (1UL << 2),
    SAFETY_FAULT_CONTROL_OVERRUN = (1UL << 3)
} SafetyFault_t;

void Safety_Init(void);
void Safety_ControlTick10ms(void);
void Safety_SetMotionActive(uint8_t motion_active);
void Safety_LatchFault(uint32_t fault_mask);
uint8_t Safety_IsMotionAllowed(void);
uint32_t Safety_GetFaults(void);

uint32_t Safety_ControlTimingStart(void);
void Safety_ControlTimingFinish(uint32_t start_cycles);
uint32_t Safety_GetControlMaxCycles(void);
uint32_t Safety_GetControlOverrunCount(void);

#endif /* SAFETY_MANAGER_H */
