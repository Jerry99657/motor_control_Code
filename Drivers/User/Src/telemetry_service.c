#include "telemetry_service.h"
#include "dc_motor_ol.h"
#include "foc_link.h"
#include "usbd_cdc_if.h"
#include <math.h>

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

void TelemetryService_Init(void)
{
    s_last_send_tick = 0U;
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
    /* Periodic telemetry owns a coalescing "latest sample" slot.  It must not
     * fill the reliable response/log queue while the host briefly pauses, and
     * it starts as soon as USB enumeration is complete rather than depending
     * on the boot welcome message. */
    (void)CDC_TransmitLatest_FS((const uint8_t *)&frame, sizeof(frame));
}
