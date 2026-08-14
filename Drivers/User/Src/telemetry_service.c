#include "telemetry_service.h"
#include "app_boot.h"
#include "command_control.h"
#include "dc_motor_ol.h"
#include "foc_link.h"
#include "usbd_cdc_if.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

typedef enum
{
    TELEMETRY_CH_MOTOR1_SPEED_RPM = 0,
    TELEMETRY_CH_MOTOR2_SPEED_RPM,
    TELEMETRY_CH_MOTOR3_SPEED_RPM,
    TELEMETRY_CH_MOTOR4_SPEED_RPM,
    TELEMETRY_CH_MOTOR1_DUTY_PERCENT,
    TELEMETRY_CH_MOTOR2_DUTY_PERCENT,
    TELEMETRY_CH_MOTOR3_DUTY_PERCENT,
    TELEMETRY_CH_MOTOR4_DUTY_PERCENT,
    TELEMETRY_CH_FOC_CONTROL_MODE,
    TELEMETRY_CH_FOC_REQUESTED_SPEED,
    TELEMETRY_CH_FOC_ACTUAL_SPEED,
    TELEMETRY_CH_FOC_REQUESTED_TURN_POSITION,
    TELEMETRY_CH_FOC_ACTUAL_TURN_POSITION,
    TELEMETRY_CH_FOC_ABSOLUTE_POSITION,
    TELEMETRY_CHANNEL_COUNT
} TelemetryChannel;

typedef struct
{
    float channel[TELEMETRY_CHANNEL_COUNT];
    uint8_t tail[4];
} TelemetryFrame;

_Static_assert(sizeof(TelemetryFrame) ==
               ((TELEMETRY_CHANNEL_COUNT * sizeof(float)) + 4U),
               "VOFA JustFloat frame must not contain padding");

static uint32_t s_last_send_tick;
static char s_usb_text_buffer[64];
static uint8_t s_usb_text_index;

void TelemetryService_Init(void)
{
    s_last_send_tick = 0U;
    s_usb_text_index = 0U;
    memset(s_usb_text_buffer, 0, sizeof(s_usb_text_buffer));
}

void TelemetryService_UsbRx(const uint8_t *buffer, uint32_t length)
{
    uint32_t index;

    if ((buffer == NULL) || (CommandControl_IsActive() == 0U))
    {
        s_usb_text_index = 0U;
        return;
    }

    for (index = 0U; index < length; ++index)
    {
        if ((buffer[index] == '\n') || (buffer[index] == '\r'))
        {
            if (s_usb_text_index > 0U)
            {
                int motor_index;
                int speed_percent;

                s_usb_text_buffer[s_usb_text_index] = '\0';
                if (sscanf(s_usb_text_buffer, "M%d:%d",
                           &motor_index, &speed_percent) == 2)
                {
                    if ((motor_index >= 1) && (motor_index <= 4))
                    {
                        if (speed_percent > 100) speed_percent = 100;
                        if (speed_percent < -100) speed_percent = -100;
                        (void)CommandControl_SetMotorSpeed(
                            (uint8_t)motor_index, (int16_t)speed_percent);
                    }
                }
                else if (strcmp(s_usb_text_buffer, "STOP") == 0)
                {
                    CommandControl_Stop();
                }
                s_usb_text_index = 0U;
            }
        }
        else if (s_usb_text_index < (sizeof(s_usb_text_buffer) - 1U))
        {
            s_usb_text_buffer[s_usb_text_index++] = (char)buffer[index];
        }
    }
}

void TelemetryService_Process(void)
{
    uint32_t now = HAL_GetTick();
    TelemetryFrame frame = {0};
    FOC_LinkTelemetry foc_telemetry;
    FOC_LinkCommandState foc_command;
    float foc_absolute_position;
    float foc_turn_position;

    if ((now - s_last_send_tick) < 20U)
    {
        return;
    }
    s_last_send_tick = now;

    if (AppBoot_IsCdcReady() == 0U)
    {
        return;
    }

    FOC_Link_GetTelemetry(&foc_telemetry);
    FOC_Link_GetCommandState(&foc_command);

    frame.channel[TELEMETRY_CH_MOTOR1_SPEED_RPM] =
        (float)DCMotor_OL_GetSpeedRpm(1);
    frame.channel[TELEMETRY_CH_MOTOR2_SPEED_RPM] =
        (float)DCMotor_OL_GetSpeedRpm(2);
    frame.channel[TELEMETRY_CH_MOTOR3_SPEED_RPM] =
        (float)DCMotor_OL_GetSpeedRpm(3);
    frame.channel[TELEMETRY_CH_MOTOR4_SPEED_RPM] =
        (float)DCMotor_OL_GetSpeedRpm(4);

    frame.channel[TELEMETRY_CH_MOTOR1_DUTY_PERCENT] =
        (float)DCMotor_OL_GetDutyPercent(1);
    frame.channel[TELEMETRY_CH_MOTOR2_DUTY_PERCENT] =
        (float)DCMotor_OL_GetDutyPercent(2);
    frame.channel[TELEMETRY_CH_MOTOR3_DUTY_PERCENT] =
        (float)DCMotor_OL_GetDutyPercent(3);
    frame.channel[TELEMETRY_CH_MOTOR4_DUTY_PERCENT] =
        (float)DCMotor_OL_GetDutyPercent(4);

    frame.channel[TELEMETRY_CH_FOC_CONTROL_MODE] = (float)foc_command.mode;
    frame.channel[TELEMETRY_CH_FOC_REQUESTED_SPEED] = foc_command.speed_target;
    frame.channel[TELEMETRY_CH_FOC_REQUESTED_TURN_POSITION] =
        foc_command.position_target;

    if (foc_telemetry.valid != 0U)
    {
        foc_absolute_position = foc_telemetry.channel[13];
        foc_turn_position = fmodf(foc_absolute_position, 6.28318530718f);
        if (foc_turn_position < 0.0f)
        {
            foc_turn_position += 6.28318530718f;
        }

        frame.channel[TELEMETRY_CH_FOC_ACTUAL_SPEED] = foc_telemetry.channel[1];
        frame.channel[TELEMETRY_CH_FOC_ACTUAL_TURN_POSITION] = foc_turn_position;
        frame.channel[TELEMETRY_CH_FOC_ABSOLUTE_POSITION] = foc_absolute_position;
    }

    frame.tail[0] = 0x00U;
    frame.tail[1] = 0x00U;
    frame.tail[2] = 0x80U;
    frame.tail[3] = 0x7FU;
    (void)CDC_Transmit_FS((uint8_t *)&frame, sizeof(frame));
}
