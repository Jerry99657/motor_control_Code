#ifndef DC_MOTOR_OL_H
#define DC_MOTOR_OL_H

#include "main.h"
#include <stdint.h>

#define DCMOTOR_OL_ENCODER_COUNTS_PER_REV 1000U
#define DCMOTOR_OL_SAMPLE_PERIOD_MS        10U
#define DCMOTOR_OL_MAX_TARGET_RPM          300
#define DCMOTOR_OL_NO_LOAD_RPM             630

HAL_StatusTypeDef DCMotor_OL_Init(void);
void DCMotor_OL_SetSpeed(uint8_t motor_index, int16_t speed_percent);
void DCMotor_OL_StopAll(void);
void DCMotor_OL_Tick10ms(void);
int32_t DCMotor_OL_GetSpeedRpm(uint8_t motor_index);
int16_t DCMotor_OL_GetDutyPercent(uint8_t motor_index);

#endif /* DC_MOTOR_OL_H */