#ifndef DC_MOTOR_OL_H
#define DC_MOTOR_OL_H

#include "main.h"
#include <stdint.h>

#define DCMOTOR_OL_ENCODER_COUNTS_PER_REV 1000U
#define DCMOTOR_OL_SAMPLE_PERIOD_MS        10U
#define DCMOTOR_OL_MAX_TARGET_RPM          300
#define DCMOTOR_OL_NO_LOAD_RPM             630

typedef enum {
    DCMOTOR_CONTROL_MODE_SPEED = 0,
    DCMOTOR_CONTROL_MODE_POSITION
} DCMotorControlMode;

typedef struct
{
    int32_t target_rpm[4];
    int32_t measured_rpm[4];
    int16_t applied_duty_percent[4];
    uint32_t encoder_suspect_events[4];
    uint32_t max_speed_error_rpm[4];
    uint8_t encoder_suspect_mask;
} DCMotorDiagnostics;

HAL_StatusTypeDef DCMotor_OL_Init(void);
void DCMotor_OL_SetSpeed(uint8_t motor_index, int16_t speed_percent);
void DCMotor_OL_SetTargetPosition(uint8_t motor_index, int64_t target_pulses, int16_t speed_limit_percent);
void DCMotor_OL_StopAll(void);
void DCMotor_OL_RequestSpeed(uint8_t motor_index, int16_t speed_percent);
void DCMotor_OL_RequestStopAll(void);
void DCMotor_OL_ApplyPendingCommands(void);
uint8_t DCMotor_OL_IsMotionActive(void);
void DCMotor_OL_Tick10ms(void);
int32_t DCMotor_OL_GetSpeedRpm(uint8_t motor_index);
int16_t DCMotor_OL_GetDutyPercent(uint8_t motor_index);
int64_t DCMotor_OL_GetPositionPulses(uint8_t motor_index);
/* Returns 0 before learning, then +1/-1 for the raw encoder phase mapping. */
int8_t DCMotor_OL_GetEncoderPolarity(uint8_t motor_index);
void DCMotor_OL_GetDiagnostics(DCMotorDiagnostics *diagnostics);

#endif /* DC_MOTOR_OL_H */
