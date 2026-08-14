#include "command_control.h"

#include "dc_motor_ol.h"
#include "foc_link.h"
#include "mecanum.h"
#include <stddef.h>
#include <string.h>

static CommandControlSnapshot s_state;

static int16_t command_control_clamp_speed(int16_t speed_percent)
{
    if (speed_percent < COMMAND_CONTROL_SPEED_MIN)
    {
        return COMMAND_CONTROL_SPEED_MIN;
    }
    if (speed_percent > COMMAND_CONTROL_SPEED_MAX)
    {
        return COMMAND_CONTROL_SPEED_MAX;
    }
    return speed_percent;
}

void CommandControl_Init(void)
{
    memset(&s_state, 0, sizeof(s_state));
}

void CommandControl_Enter(void)
{
    s_state.active = 1U;
    CommandControl_Stop();
}

void CommandControl_Leave(void)
{
    if (s_state.active != 0U)
    {
        CommandControl_Stop();
    }
    s_state.active = 0U;
}

uint8_t CommandControl_IsActive(void)
{
    return s_state.active;
}

uint8_t CommandControl_SetMotorSpeed(uint8_t motor_index,
                                     int16_t speed_percent)
{
    if ((s_state.active == 0U) || (motor_index == 0U) ||
        (motor_index > COMMAND_CONTROL_MOTOR_COUNT))
    {
        s_state.rejected_command_count++;
        return 0U;
    }

    speed_percent = command_control_clamp_speed(speed_percent);
    s_state.motor_speed_setpoint[motor_index - 1U] = speed_percent;
    Mecanum_CancelControl();
    DCMotor_OL_RequestSpeed(motor_index, speed_percent);
    s_state.accepted_command_count++;
    return 1U;
}

uint8_t CommandControl_SetJoystick(int8_t lx, int8_t ly,
                                   int8_t rx, int8_t ry)
{
    float wz;
    float vy;
    float vx;

    if (s_state.active == 0U)
    {
        s_state.rejected_command_count++;
        return 0U;
    }

    s_state.joystick_lx = lx;
    s_state.joystick_ly = ly;
    s_state.joystick_rx = rx;
    s_state.joystick_ry = ry;
    wz = (float)lx * 2.0f;
    vy = (float)rx * 10.0f;
    vx = (float)ry * 10.0f;
    Mecanum_MixedControl(vx, vy, wz, 0.0f, 0.0f, 0.0f);
    s_state.accepted_command_count++;
    return 1U;
}

void CommandControl_SetGyroState(uint8_t enabled,
                                 int8_t signed_speed_percent)
{
    if (signed_speed_percent < COMMAND_CONTROL_SPEED_MIN)
    {
        signed_speed_percent = COMMAND_CONTROL_SPEED_MIN;
    }
    else if (signed_speed_percent > COMMAND_CONTROL_SPEED_MAX)
    {
        signed_speed_percent = COMMAND_CONTROL_SPEED_MAX;
    }

    s_state.gyro_enabled = (enabled != 0U) ? 1U : 0U;
    s_state.gyro_signed_speed = signed_speed_percent;
}

void CommandControl_Stop(void)
{
    uint8_t index;

    if (s_state.active == 0U)
    {
        return;
    }

    for (index = 0U; index < COMMAND_CONTROL_MOTOR_COUNT; ++index)
    {
        s_state.motor_speed_setpoint[index] = 0;
    }
    s_state.joystick_lx = 0;
    s_state.joystick_ly = 0;
    s_state.joystick_rx = 0;
    s_state.joystick_ry = 0;
    s_state.gyro_enabled = 0U;
    s_state.gyro_signed_speed = 0;

    Mecanum_GyroDisable();
    Mecanum_MixedControl(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    DCMotor_OL_RequestStopAll();
    (void)FOC_Link_SendStop();
    s_state.stop_count++;
}

void CommandControl_GetSnapshot(CommandControlSnapshot *snapshot)
{
    if (snapshot != NULL)
    {
        *snapshot = s_state;
    }
}
