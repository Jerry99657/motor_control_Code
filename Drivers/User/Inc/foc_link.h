#ifndef FOC_LINK_H
#define FOC_LINK_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FOC_LINK_HOST_DEVICE_ID       0x0FU
#define FOC_LINK_HOST_CMD_READ        0x01U
#define FOC_LINK_HOST_CMD_WRITE       0x02U

typedef enum
{
  FOC_LINK_OP_SPEED = 0x01U,
  FOC_LINK_OP_ANGLE = 0x02U,
  FOC_LINK_OP_TORQUE = 0x03U,
  FOC_LINK_OP_MOTOR = 0x04U,
  FOC_LINK_OP_SENSORLESS = 0x05U,
  FOC_LINK_OP_LOCK = 0x06U,
  /* 0x07 is reserved until the slave implements a native safe Stop. */
  FOC_LINK_OP_STOP = 0x08U
} FOC_LinkOperation;

typedef enum
{
  FOC_LINK_OK = 0,
  FOC_LINK_ERR_ARGUMENT = -1,
  FOC_LINK_ERR_QUEUE_FULL = -2,
  FOC_LINK_ERR_FORMAT = -3,
  FOC_LINK_ERR_SAFETY = -4
} FOC_LinkResult;

typedef enum
{
  FOC_LINK_CONTROL_IDLE = 0U,
  FOC_LINK_CONTROL_SPEED = 1U,
  FOC_LINK_CONTROL_POSITION = 2U,
  FOC_LINK_CONTROL_TORQUE = 3U
} FOC_LinkControlMode;

typedef struct
{
  float speed_target;
  float position_target;
  float torque_target;
  FOC_LinkControlMode mode;
} FOC_LinkCommandState;

typedef struct
{
  float channel[16];
  uint32_t frame_count;
  uint32_t last_rx_tick;
  uint32_t rx_overflow_count;
  uint32_t rx_error_count;
  uint32_t tx_drop_count;
  uint8_t valid;
} FOC_LinkTelemetry;

void FOC_Link_Init(void);
void FOC_Link_Process(void);
void FOC_Link_SetSafetyInhibit(uint8_t inhibit);

int8_t FOC_Link_SendSpeed(float value);
int8_t FOC_Link_SendAngle(float value);
int8_t FOC_Link_SendTorque(float value);
int8_t FOC_Link_SendMotor(uint8_t motor_id);
int8_t FOC_Link_SendSensorless(void);
int8_t FOC_Link_SendLock(void);
int8_t FOC_Link_SendStop(void);

void FOC_Link_GetTelemetry(FOC_LinkTelemetry *telemetry);
void FOC_Link_GetCommandState(FOC_LinkCommandState *state);
uint8_t FOC_Link_IsTelemetryAlive(uint32_t timeout_ms);

void FOC_Link_UartRxCompleteFromISR(UART_HandleTypeDef *huart);
void FOC_Link_UartTxCompleteFromISR(UART_HandleTypeDef *huart);
void FOC_Link_UartErrorFromISR(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* FOC_LINK_H */
