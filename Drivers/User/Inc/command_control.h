#ifndef COMMAND_CONTROL_H
#define COMMAND_CONTROL_H

#include <stdint.h>

#define COMMAND_CONTROL_MOTOR_COUNT 4U
#define COMMAND_CONTROL_SPEED_MIN   (-100)
#define COMMAND_CONTROL_SPEED_MAX   100

typedef struct
{
    int16_t motor_speed_setpoint[COMMAND_CONTROL_MOTOR_COUNT];
    int8_t joystick_lx;
    int8_t joystick_ly;
    int8_t joystick_rx;
    int8_t joystick_ry;
    int8_t gyro_signed_speed;
    uint8_t active;
    uint8_t gyro_enabled;
    uint32_t accepted_command_count;
    uint32_t rejected_command_count;
    uint32_t stop_count;
} CommandControlSnapshot;

void CommandControl_Init(void);
void CommandControl_Enter(void);
void CommandControl_Leave(void);
uint8_t CommandControl_IsActive(void);

uint8_t CommandControl_SetMotorSpeed(uint8_t motor_index,
                                     int16_t speed_percent);
uint8_t CommandControl_SetJoystick(int8_t lx, int8_t ly,
                                   int8_t rx, int8_t ry);
void CommandControl_SetGyroState(uint8_t enabled,
                                 int8_t signed_speed_percent);
void CommandControl_Stop(void);
void CommandControl_GetSnapshot(CommandControlSnapshot *snapshot);

#endif /* COMMAND_CONTROL_H */
