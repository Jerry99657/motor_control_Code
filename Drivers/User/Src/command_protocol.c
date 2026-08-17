#include "command_protocol.h"

#include "comm_service.h"
#include "camera_stream.h"
#include "command_control.h"
#include "dc_motor_ol.h"
#include "foc_link.h"
#include "mecanum.h"
#include "nes_runtime.h"
#include "usbd_cdc_if.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#define COMMAND_PROTOCOL_FRAME_CAPACITY 16U
#define COMMAND_PROTOCOL_FRAME_MIN_SIZE 6U
#define COMMAND_PROTOCOL_TEXT_CAPACITY  64U
#define COMMAND_PROTOCOL_HEADER_0       0x77U
#define COMMAND_PROTOCOL_HEADER_1       0x68U
#define COMMAND_PROTOCOL_TAIL           0x0AU

typedef struct
{
    uint8_t frame[COMMAND_PROTOCOL_FRAME_CAPACITY];
    uint16_t frame_length;
    char text[COMMAND_PROTOCOL_TEXT_CAPACITY];
    uint16_t text_length;
} CommandProtocolChannelState;

static CommandProtocolChannelState s_channel[COMMAND_PROTOCOL_CHANNEL_COUNT];
static CommandProtocolStats s_stats;

static uint8_t command_protocol_send(uint8_t channel, const uint8_t *data,
                                     uint16_t length)
{
    uint8_t queued = 0U;

    if ((data == NULL) || (length == 0U))
    {
        return 0U;
    }

    if (channel == COMMAND_PROTOCOL_CHANNEL_UART5)
    {
        queued = CommService_UartSend(data, length);
    }
    else if (channel == COMMAND_PROTOCOL_CHANNEL_USB)
    {
        queued = (CDC_Transmit_FS((uint8_t *)data, length) == USBD_OK) ? 1U : 0U;
    }

    if (queued == 0U)
    {
        s_stats.response_drop_count++;
    }
    return queued;
}

static void command_protocol_send_text(uint8_t channel, const char *text)
{
    if (text != NULL)
    {
        (void)command_protocol_send(channel, (const uint8_t *)text,
                                    (uint16_t)strlen(text));
    }
}

static float command_protocol_read_float_le(const uint8_t *bytes)
{
    float value = 0.0f;

    if (bytes != NULL)
    {
        memcpy(&value, bytes, sizeof(value));
    }
    return value;
}

static int8_t command_protocol_execute_foc(const uint8_t *frame,
                                           uint8_t length)
{
    if ((frame == NULL) || (length < 7U))
    {
        return FOC_LINK_ERR_ARGUMENT;
    }

    switch (frame[5])
    {
        case FOC_LINK_OP_SPEED:
            return (length == 11U) ?
                FOC_Link_SendSpeed(command_protocol_read_float_le(&frame[6])) :
                FOC_LINK_ERR_ARGUMENT;
        case FOC_LINK_OP_ANGLE:
            return (length == 11U) ?
                FOC_Link_SendAngle(command_protocol_read_float_le(&frame[6])) :
                FOC_LINK_ERR_ARGUMENT;
        case FOC_LINK_OP_TORQUE:
            return (length == 11U) ?
                FOC_Link_SendTorque(command_protocol_read_float_le(&frame[6])) :
                FOC_LINK_ERR_ARGUMENT;
        case FOC_LINK_OP_MOTOR:
            return (length == 8U) ? FOC_Link_SendMotor(frame[6]) :
                                    FOC_LINK_ERR_ARGUMENT;
        case FOC_LINK_OP_SENSORLESS:
            return (length == 7U) ? FOC_Link_SendSensorless() :
                                    FOC_LINK_ERR_ARGUMENT;
        case FOC_LINK_OP_LOCK:
            return (length == 7U) ? FOC_Link_SendLock() :
                                    FOC_LINK_ERR_ARGUMENT;
        case FOC_LINK_OP_STOP:
            return (length == 7U) ? FOC_Link_SendStop() :
                                    FOC_LINK_ERR_ARGUMENT;
        default:
            return FOC_LINK_ERR_ARGUMENT;
    }
}

static void command_protocol_send_foc_ack(uint8_t channel, uint8_t operation,
                                          int8_t result)
{
    uint8_t response[8] = {
        COMMAND_PROTOCOL_HEADER_0, COMMAND_PROTOCOL_HEADER_1, 0x08U,
        FOC_LINK_HOST_DEVICE_ID, FOC_LINK_HOST_CMD_WRITE, operation,
        (uint8_t)result, COMMAND_PROTOCOL_TAIL
    };

    (void)command_protocol_send(channel, response, sizeof(response));
}

static void command_protocol_send_foc_status(uint8_t channel)
{
    FOC_LinkTelemetry telemetry;
    uint8_t response[32] = {0};
    uint8_t flags = 0U;

    FOC_Link_GetTelemetry(&telemetry);
    if (telemetry.valid != 0U) flags |= 0x01U;
    if (FOC_Link_IsTelemetryAlive(250U) != 0U) flags |= 0x02U;
    if (telemetry.rx_overflow_count != 0U) flags |= 0x04U;
    if (telemetry.rx_error_count != 0U) flags |= 0x08U;
    if (telemetry.tx_drop_count != 0U) flags |= 0x10U;

    response[0] = COMMAND_PROTOCOL_HEADER_0;
    response[1] = COMMAND_PROTOCOL_HEADER_1;
    response[2] = (uint8_t)sizeof(response);
    response[3] = FOC_LINK_HOST_DEVICE_ID;
    response[4] = FOC_LINK_HOST_CMD_READ;
    response[5] = flags;
    memcpy(&response[6], &telemetry.channel[0], sizeof(float));
    memcpy(&response[10], &telemetry.channel[1], sizeof(float));
    memcpy(&response[14], &telemetry.channel[2], sizeof(float));
    memcpy(&response[18], &telemetry.channel[3], sizeof(float));
    memcpy(&response[22], &telemetry.channel[8], sizeof(float));
    memcpy(&response[26], &telemetry.channel[9], sizeof(float));
    response[30] = (uint8_t)telemetry.frame_count;
    response[31] = COMMAND_PROTOCOL_TAIL;
    (void)command_protocol_send(channel, response, sizeof(response));
}

static uint8_t command_protocol_parse_foc_text(uint8_t channel,
                                               const char *line)
{
    float value = 0.0f;
    unsigned int motor_id = 0U;
    char trailing = '\0';
    char response[40];
    int8_t result = FOC_LINK_ERR_ARGUMENT;
    const char *operation = "ERROR";

    if ((line == NULL) || (strncmp(line, "FOC ", 4U) != 0))
    {
        return 0U;
    }

    if (sscanf(line, "FOC SPEED %f %c", &value, &trailing) == 1)
    {
        operation = "SPEED";
        result = FOC_Link_SendSpeed(value);
    }
    else if (sscanf(line, "FOC ANGLE %f %c", &value, &trailing) == 1)
    {
        operation = "ANGLE";
        result = FOC_Link_SendAngle(value);
    }
    else if (sscanf(line, "FOC TORQUE %f %c", &value, &trailing) == 1)
    {
        operation = "TORQUE";
        result = FOC_Link_SendTorque(value);
    }
    else if ((sscanf(line, "FOC MOTOR %u %c", &motor_id, &trailing) == 1) &&
             (motor_id <= 1U))
    {
        operation = "MOTOR";
        result = FOC_Link_SendMotor((uint8_t)motor_id);
    }
    else if (strcmp(line, "FOC SENSORLESS") == 0)
    {
        operation = "SENSORLESS";
        result = FOC_Link_SendSensorless();
    }
    else if (strcmp(line, "FOC LOCK") == 0)
    {
        operation = "LOCK";
        result = FOC_Link_SendLock();
    }
    else if (strcmp(line, "FOC STOP") == 0)
    {
        operation = "STOP";
        result = FOC_Link_SendStop();
    }

    (void)snprintf(response, sizeof(response), "FOC %s %s (%d)\r\n",
                   operation, (result == FOC_LINK_OK) ? "QUEUED" : "FAILED",
                   (int)result);
    command_protocol_send_text(channel, response);
    if (result == FOC_LINK_OK)
    {
        s_stats.accepted_command_count++;
    }
    else
    {
        s_stats.rejected_command_count++;
    }
    return 1U;
}

static uint8_t command_protocol_parse_usb_motor_text(uint8_t channel,
                                                      const char *line)
{
    int motor_index;
    int speed_percent;

    if ((channel != COMMAND_PROTOCOL_CHANNEL_USB) || (line == NULL))
    {
        return 0U;
    }

    {
        char trailing;
        int fields = sscanf(line, "M%d:%d %c", &motor_index,
                            &speed_percent, &trailing);
        if (fields == 2)
        {
            uint8_t accepted = 0U;

            if ((motor_index >= 1) && (motor_index <= 4))
            {
                if (speed_percent > 100) speed_percent = 100;
                if (speed_percent < -100) speed_percent = -100;
                accepted = CommandControl_SetMotorSpeed((uint8_t)motor_index,
                                                         (int16_t)speed_percent);
            }
            if (accepted != 0U) s_stats.accepted_command_count++;
            else s_stats.rejected_command_count++;
            return 1U;
        }
    }

    if (strcmp(line, "STOP") == 0)
    {
        CommandControl_Stop();
        s_stats.accepted_command_count++;
        return 1U;
    }
    return 0U;
}

static void command_protocol_parse_text(uint8_t channel, char *line)
{
    char command[8] = {0};
    char action[8] = {0};
    char direction_text[8] = {0};
    char extra[8] = {0};
    int speed_percent = 0;
    int fields;
    uint16_t index;
    int8_t direction;
    uint8_t accepted;

    if ((line == NULL) || (CommandControl_IsActive() == 0U))
    {
        return;
    }

    for (index = 0U; line[index] != '\0'; ++index)
    {
        line[index] = (char)toupper((unsigned char)line[index]);
    }

    if ((command_protocol_parse_foc_text(channel, line) != 0U) ||
        (command_protocol_parse_usb_motor_text(channel, line) != 0U))
    {
        return;
    }

    fields = sscanf(line, "%7s %7s %7s %d %7s", command, action,
                    direction_text, &speed_percent, extra);
    if (strcmp(command, "GYRO") != 0)
    {
        s_stats.rejected_command_count++;
        return;
    }

    if ((fields == 2) && (strcmp(action, "OFF") == 0))
    {
        accepted = CommandControl_SetGyro(0U, MECANUM_GYRO_DIRECTION_CW, 0U);
        command_protocol_send_text(channel,
                                   (accepted != 0U) ? "GYRO OFF OK\r\n" :
                                                     "ERR GYRO OFF\r\n");
    }
    else if ((fields == 4) && (strcmp(action, "ON") == 0) &&
             (speed_percent >= 0) && (speed_percent <= 100))
    {
        if (strcmp(direction_text, "CW") == 0)
        {
            direction = MECANUM_GYRO_DIRECTION_CW;
        }
        else if (strcmp(direction_text, "CCW") == 0)
        {
            direction = MECANUM_GYRO_DIRECTION_CCW;
        }
        else
        {
            command_protocol_send_text(channel, "ERR GYRO DIR CW|CCW\r\n");
            s_stats.rejected_command_count++;
            return;
        }

        accepted = CommandControl_SetGyro(1U, direction,
                                           (uint8_t)speed_percent);
        command_protocol_send_text(channel,
                                   (accepted != 0U) ? "GYRO ON OK\r\n" :
                                                     "ERR GYRO RANGE 0-100\r\n");
    }
    else
    {
        accepted = 0U;
        command_protocol_send_text(
            channel, "ERR USE: GYRO ON CW|CCW 0-100 / GYRO OFF\r\n");
    }

    if (accepted != 0U) s_stats.accepted_command_count++;
    else s_stats.rejected_command_count++;
}

static void command_protocol_text_byte(uint8_t channel, uint8_t byte)
{
    CommandProtocolChannelState *state = &s_channel[channel];

    if ((byte == '\r') || (byte == '\n'))
    {
        if (state->text_length != 0U)
        {
            state->text[state->text_length] = '\0';
            s_stats.text_line_count[channel]++;
            command_protocol_parse_text(channel, state->text);
            state->text_length = 0U;
        }
        return;
    }

    if ((byte < 0x20U) || (byte > 0x7EU))
    {
        state->text_length = 0U;
        return;
    }

    if (state->text_length < (COMMAND_PROTOCOL_TEXT_CAPACITY - 1U))
    {
        state->text[state->text_length++] = (char)byte;
    }
    else
    {
        state->text_length = 0U;
        s_stats.rejected_command_count++;
    }
}

static uint8_t command_protocol_dispatch_frame(uint8_t channel,
                                               const uint8_t *frame,
                                               uint8_t length)
{
    uint8_t device_id = frame[3];
    uint8_t command = frame[4];
    uint8_t accepted = 0U;

    if ((channel == COMMAND_PROTOCOL_CHANNEL_UART5) &&
        (device_id == 0x0EU) && (command == 0x02U) && (length == 0x08U))
    {
        if (NES_Runtime_IsActive() != 0U)
        {
            NES_Runtime_SetRemoteButtons(frame[5]);
            accepted = 1U;
        }
    }
    else if ((channel == COMMAND_PROTOCOL_CHANNEL_UART5) &&
             (device_id == 0x0EU) && (command == 0x03U) &&
             (length == 0x07U))
    {
        if (NES_Runtime_IsActive() != 0U)
        {
            NES_Runtime_RequestRemoteReset();
            accepted = 1U;
        }
    }
    else if ((channel == COMMAND_PROTOCOL_CHANNEL_UART5) &&
             (device_id == CAMERA_STREAM_DEVICE_ID) &&
             (command == CAMERA_STREAM_COMMAND_ENABLE) &&
             (length == 0x07U))
    {
        CameraStream_SetRemoteEnabled((frame[5] != 0U) ? 1U : 0U);
        accepted = 1U;
    }
    else if ((channel == COMMAND_PROTOCOL_CHANNEL_UART5) &&
             (device_id == CAMERA_STREAM_DEVICE_ID) &&
             (command == CAMERA_STREAM_COMMAND_FRAME_ACK) &&
             (length == 0x08U))
    {
        uint16_t sequence = (uint16_t)frame[5] |
                            ((uint16_t)frame[6] << 8U);
        CameraStream_AcknowledgeFrame(sequence);
        accepted = 1U;
    }
    else if (CommandControl_IsActive() == 0U)
    {
        accepted = 0U;
    }
    else if ((device_id == 0x0CU) && (length == 0x0AU))
    {
        accepted = CommandControl_SetJoystick((int8_t)frame[4],
                                               (int8_t)frame[5],
                                               (int8_t)frame[6],
                                               (int8_t)frame[7]);
    }
    else if ((command == FOC_LINK_HOST_CMD_WRITE) &&
             (device_id == FOC_LINK_HOST_DEVICE_ID))
    {
        uint8_t operation = (length >= 7U) ? frame[5] : 0U;
        int8_t result = command_protocol_execute_foc(frame, length);
        command_protocol_send_foc_ack(channel, operation, result);
        accepted = (result == FOC_LINK_OK) ? 1U : 0U;
    }
    else if ((command == 0x02U) && (device_id == 0x01U) &&
             (length == 0x0AU))
    {
        uint8_t motor;
        accepted = 1U;
        for (motor = 0U; motor < 4U; ++motor)
        {
            if (CommandControl_SetMotorSpeed((uint8_t)(motor + 1U),
                                             (int8_t)frame[5U + motor]) == 0U)
            {
                accepted = 0U;
            }
        }
    }
    else if ((command == 0x02U) && (device_id == 0x02U) &&
             (length == 0x08U))
    {
        accepted = CommandControl_SetMotorSpeed(frame[5], (int8_t)frame[6]);
    }
    else if ((command == 0x02U) && (device_id == 0x03U) &&
             (length == 0x0DU))
    {
        int16_t vx = (int16_t)(((uint16_t)frame[7] << 8U) | frame[6]);
        int16_t vy = (int16_t)(((uint16_t)frame[9] << 8U) | frame[8]);
        int16_t wz = (int16_t)(((uint16_t)frame[11] << 8U) | frame[10]);
        accepted = CommandControl_SetMecanum(frame[5], vx, vy, wz);
    }
    else if ((command == 0x02U) && (device_id == 0x0DU) &&
             (length == 0x09U))
    {
        accepted = CommandControl_SetGyro(frame[5], (int8_t)frame[6], frame[7]);
    }
    else if ((command == FOC_LINK_HOST_CMD_READ) &&
             (device_id == FOC_LINK_HOST_DEVICE_ID) && (length == 0x06U))
    {
        command_protocol_send_foc_status(channel);
        accepted = 1U;
    }
    else if ((command == 0x01U) && (device_id == 0x03U) &&
             (length == 0x06U))
    {
        uint8_t response[10] = {
            COMMAND_PROTOCOL_HEADER_0, COMMAND_PROTOCOL_HEADER_1, 0x0AU,
            0x03U, 0x01U, 0U, 0U, 0U, 0U, COMMAND_PROTOCOL_TAIL
        };
        uint8_t motor;
        for (motor = 0U; motor < 4U; ++motor)
        {
            response[5U + motor] =
                (uint8_t)DCMotor_OL_GetSpeedRpm((uint8_t)(motor + 1U));
        }
        accepted = command_protocol_send(channel, response, sizeof(response));
    }
    else if ((command == 0x01U) && (device_id == 0x04U) &&
             (length == 0x07U))
    {
        uint8_t port = frame[5];
        uint8_t response[8] = {
            COMMAND_PROTOCOL_HEADER_0, COMMAND_PROTOCOL_HEADER_1, 0x08U,
            0x04U, 0x01U, port, 0U, COMMAND_PROTOCOL_TAIL
        };
        if ((port >= 1U) && (port <= 4U))
        {
            response[6] = (uint8_t)DCMotor_OL_GetSpeedRpm(port);
        }
        accepted = command_protocol_send(channel, response, sizeof(response));
    }

    if (accepted != 0U) s_stats.accepted_command_count++;
    else s_stats.rejected_command_count++;
    return accepted;
}

static void command_protocol_frame_byte(uint8_t channel, uint8_t byte)
{
    CommandProtocolChannelState *state = &s_channel[channel];

    if (state->frame_length < COMMAND_PROTOCOL_FRAME_CAPACITY)
    {
        state->frame[state->frame_length++] = byte;
    }
    else
    {
        state->frame_length = 0U;
        s_stats.frame_error_count[channel]++;
        return;
    }

    while (state->frame_length >= 3U)
    {
        uint8_t frame_length;

        if ((state->frame[0] != COMMAND_PROTOCOL_HEADER_0) ||
            (state->frame[1] != COMMAND_PROTOCOL_HEADER_1))
        {
            memmove(state->frame, &state->frame[1], state->frame_length - 1U);
            state->frame_length--;
            s_stats.resync_discard_count[channel]++;
            continue;
        }

        frame_length = state->frame[2];
        if ((frame_length < COMMAND_PROTOCOL_FRAME_MIN_SIZE) ||
            (frame_length > COMMAND_PROTOCOL_FRAME_CAPACITY))
        {
            memmove(state->frame, &state->frame[2], state->frame_length - 2U);
            state->frame_length -= 2U;
            s_stats.frame_error_count[channel]++;
            continue;
        }

        if (state->frame_length < frame_length)
        {
            break;
        }

        if (state->frame[frame_length - 1U] == COMMAND_PROTOCOL_TAIL)
        {
            s_stats.valid_frame_count[channel]++;
            (void)command_protocol_dispatch_frame(channel, state->frame,
                                                   frame_length);
        }
        else
        {
            s_stats.frame_error_count[channel]++;
            memmove(state->frame, &state->frame[1],
                    state->frame_length - 1U);
            state->frame_length--;
            continue;
        }

        memmove(state->frame, &state->frame[frame_length],
                state->frame_length - frame_length);
        state->frame_length -= frame_length;
    }
}

void CommandProtocol_Init(void)
{
    memset(s_channel, 0, sizeof(s_channel));
    memset(&s_stats, 0, sizeof(s_stats));
}

void CommandProtocol_ResetInput(void)
{
    uint8_t channel;

    for (channel = 0U; channel < COMMAND_PROTOCOL_CHANNEL_COUNT; ++channel)
    {
        s_channel[channel].frame_length = 0U;
        s_channel[channel].text_length = 0U;
    }
}

void CommandProtocol_Receive(uint8_t channel, const uint8_t *data,
                             uint16_t length)
{
    uint16_t index;

    if ((channel >= COMMAND_PROTOCOL_CHANNEL_COUNT) || (data == NULL) ||
        (length == 0U))
    {
        return;
    }

    s_stats.rx_byte_count[channel] += length;
    if (CommandControl_IsActive() == 0U)
    {
        s_channel[channel].text_length = 0U;
    }

    for (index = 0U; index < length; ++index)
    {
        if (CommandControl_IsActive() != 0U)
        {
            command_protocol_text_byte(channel, data[index]);
        }
        command_protocol_frame_byte(channel, data[index]);
    }
}

void CommandProtocol_GetStats(CommandProtocolStats *stats)
{
    if (stats != NULL)
    {
        *stats = s_stats;
    }
}
