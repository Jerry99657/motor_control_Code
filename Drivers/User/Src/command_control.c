#include "command_control.h"

#include "dc_motor_ol.h"
#include "foc_link.h"
#include "mecanum.h"
#include <stddef.h>
#include <string.h>

#define COMMAND_CONTROL_JOYSTICK_TAKEOVER_DEADZONE 4

typedef enum
{
    COMMAND_CONTROL_OWNER_NONE = 0,
    COMMAND_CONTROL_OWNER_DIRECT_MOTOR,
    COMMAND_CONTROL_OWNER_CHASSIS
} CommandControlMotionOwner;

static CommandControlSnapshot s_state;
static CommandControlMotionOwner s_motion_owner;

static void command_control_clear_motor_setpoints(void)
{
    uint8_t index;

    for (index = 0U; index < COMMAND_CONTROL_MOTOR_COUNT; ++index)
    {
        s_state.motor_speed_setpoint[index] = 0;
    }
}

static void command_control_request_all_motor_speeds_zero(void)
{
    uint8_t index;

    for (index = 0U; index < COMMAND_CONTROL_MOTOR_COUNT; ++index)
    {
        DCMotor_OL_RequestSpeed((uint8_t)(index + 1U), 0);
    }
}

static uint8_t command_control_has_motor_setpoint(void)
{
    uint8_t index;

    for (index = 0U; index < COMMAND_CONTROL_MOTOR_COUNT; ++index)
    {
        if (s_state.motor_speed_setpoint[index] != 0)
        {
            return 1U;
        }
    }
    return 0U;
}

static uint8_t command_control_joystick_requests_takeover(int8_t lx,
                                                           int8_t rx,
                                                           int8_t ry)
{
    return (((lx < -COMMAND_CONTROL_JOYSTICK_TAKEOVER_DEADZONE) ||
             (lx > COMMAND_CONTROL_JOYSTICK_TAKEOVER_DEADZONE)) ||
            ((rx < -COMMAND_CONTROL_JOYSTICK_TAKEOVER_DEADZONE) ||
             (rx > COMMAND_CONTROL_JOYSTICK_TAKEOVER_DEADZONE)) ||
            ((ry < -COMMAND_CONTROL_JOYSTICK_TAKEOVER_DEADZONE) ||
             (ry > COMMAND_CONTROL_JOYSTICK_TAKEOVER_DEADZONE))) ? 1U : 0U;
}

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
    s_motion_owner = COMMAND_CONTROL_OWNER_NONE;
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
    Mecanum_CancelControl();
    if (s_motion_owner != COMMAND_CONTROL_OWNER_DIRECT_MOTOR)
    {
        /* A direct-motor session starts from a known wheel state.  Clear old
         * chassis/gyro targets first, then apply the requested motor below. */
        command_control_clear_motor_setpoints();
        command_control_request_all_motor_speeds_zero();
    }
    s_state.motor_speed_setpoint[motor_index - 1U] = speed_percent;
    s_motion_owner = (command_control_has_motor_setpoint() != 0U) ?
                     COMMAND_CONTROL_OWNER_DIRECT_MOTOR :
                     COMMAND_CONTROL_OWNER_NONE;
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

    /* The ESP32 sends centred joystick frames continuously.  A centred frame
     * from UART5 must not cancel a deliberate M1..M4 command received over
     * USB.  Once the user really moves a stick, joystick control takes
     * ownership; subsequent centred frames retain their normal stop action. */
    if ((s_motion_owner == COMMAND_CONTROL_OWNER_DIRECT_MOTOR) &&
        (command_control_joystick_requests_takeover(lx, rx, ry) == 0U))
    {
        s_state.accepted_command_count++;
        return 1U;
    }

    command_control_clear_motor_setpoints();
    s_motion_owner = COMMAND_CONTROL_OWNER_CHASSIS;
    wz = (float)lx * 2.0f;
    vy = (float)rx * 10.0f;
    vx = (float)ry * 10.0f;
    Mecanum_MixedControl(vx, vy, wz, 0.0f, 0.0f, 0.0f);
    s_state.accepted_command_count++;
    return 1U;
}

uint8_t CommandControl_SetMecanum(uint8_t mode, int16_t vx,
                                  int16_t vy, int16_t wz)
{
    if ((s_state.active == 0U) || (mode > 1U))
    {
        s_state.rejected_command_count++;
        return 0U;
    }

    if (mode == 1U)
    {
        Mecanum_MixedControl(100.0f, 100.0f, 100.0f,
                             (float)vx, (float)vy, (float)wz);
    }
    else
    {
        vx = command_control_clamp_speed(vx);
        vy = command_control_clamp_speed(vy);
        wz = command_control_clamp_speed(wz);
        Mecanum_MixedControl((float)vx, (float)vy, (float)wz,
                             0.0f, 0.0f, 0.0f);
    }
    command_control_clear_motor_setpoints();
    s_motion_owner = COMMAND_CONTROL_OWNER_CHASSIS;
    s_state.accepted_command_count++;
    return 1U;
}

uint8_t CommandControl_SetGyro(uint8_t enabled, int8_t direction,
                               uint8_t speed_percent)
{
    int8_t signed_speed;

    if ((s_state.active == 0U) || (enabled > 1U) ||
        (speed_percent > 100U) ||
        ((direction != MECANUM_GYRO_DIRECTION_CW) &&
         (direction != MECANUM_GYRO_DIRECTION_CCW)))
    {
        s_state.rejected_command_count++;
        return 0U;
    }

    signed_speed = (int8_t)((direction == MECANUM_GYRO_DIRECTION_CCW) ?
                            -(int16_t)speed_percent :
                            (int16_t)speed_percent);
    if (enabled != 0U)
    {
        if (Mecanum_GyroEnable(direction, speed_percent) == 0U)
        {
            s_state.rejected_command_count++;
            return 0U;
        }
        command_control_clear_motor_setpoints();
        s_motion_owner = COMMAND_CONTROL_OWNER_CHASSIS;
        CommandControl_SetGyroState(1U, signed_speed);
    }
    else
    {
        uint8_t was_enabled = Mecanum_IsGyroModeEnabled();
        Mecanum_GyroDisable();
        if (was_enabled != 0U)
        {
            Mecanum_MixedControl(0.0f, 0.0f, 0.0f,
                                 0.0f, 0.0f, 0.0f);
            command_control_clear_motor_setpoints();
            s_motion_owner = COMMAND_CONTROL_OWNER_NONE;
        }
        /* Keep the remote slider preset visible while gyro mode is off. */
        CommandControl_SetGyroState(0U, signed_speed);
    }

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
    if (s_state.active == 0U)
    {
        return;
    }

    command_control_clear_motor_setpoints();
    s_motion_owner = COMMAND_CONTROL_OWNER_NONE;
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
