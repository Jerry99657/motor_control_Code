#include "foc_link.h"
#include "safety_manager.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define FOC_LINK_TX_QUEUE_DEPTH       8U
#define FOC_LINK_TX_LINE_SIZE         32U
#define FOC_LINK_RX_RING_SIZE         512U
#define FOC_LINK_TELEMETRY_FLOATS     16U
#define FOC_LINK_TELEMETRY_SIZE       68U

typedef struct
{
  uint8_t length;
  uint8_t data[FOC_LINK_TX_LINE_SIZE];
} FOC_LinkTxPacket;

extern UART_HandleTypeDef huart4;

static FOC_LinkTxPacket s_tx_queue[FOC_LINK_TX_QUEUE_DEPTH];
static volatile uint8_t s_tx_head = 0U;
static volatile uint8_t s_tx_tail = 0U;
static volatile uint8_t s_tx_busy = 0U;
static volatile uint32_t s_tx_drop_count = 0U;

static uint8_t s_rx_byte = 0U;
static uint8_t s_rx_ring[FOC_LINK_RX_RING_SIZE];
static volatile uint16_t s_rx_head = 0U;
static volatile uint16_t s_rx_tail = 0U;
static volatile uint32_t s_rx_overflow_count = 0U;
static volatile uint32_t s_rx_error_count = 0U;
static volatile uint8_t s_rx_rearm_pending = 0U;
static uint32_t s_last_rx_rearm_tick = 0U;

static uint8_t s_telemetry_window[FOC_LINK_TELEMETRY_SIZE];
static uint8_t s_telemetry_count = 0U;
static FOC_LinkTelemetry s_telemetry;
static FOC_LinkCommandState s_command_state;
static volatile uint8_t s_safety_inhibit = 0U;
static volatile uint8_t s_safety_stop_pending = 0U;

_Static_assert(sizeof(float) == 4U, "FOC telemetry requires IEEE-754 float32");

static uint32_t foc_link_lock(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  __DMB();
  return primask;
}

static void foc_link_unlock(uint32_t primask)
{
  __DMB();
  if (primask == 0U)
  {
    __enable_irq();
  }
}

static void foc_link_arm_rx(void)
{
  if (HAL_UART_Receive_IT(&huart4, &s_rx_byte, 1U) == HAL_OK)
  {
    s_rx_rearm_pending = 0U;
  }
  else
  {
    s_rx_rearm_pending = 1U;
    s_rx_error_count++;
  }
}

static void foc_link_tx_kick(void)
{
  uint8_t index;
  uint8_t length;
  uint32_t primask = foc_link_lock();

  if ((s_tx_busy != 0U) || (s_tx_tail == s_tx_head))
  {
    foc_link_unlock(primask);
    return;
  }
  index = s_tx_tail;
  length = s_tx_queue[index].length;
  s_tx_busy = 1U;
  foc_link_unlock(primask);

  if (HAL_UART_Transmit_IT(&huart4, s_tx_queue[index].data, length) != HAL_OK)
  {
    primask = foc_link_lock();
    s_tx_busy = 0U;
    foc_link_unlock(primask);
  }
}

static int8_t foc_link_queue_line(const char *line, int length)
{
  uint8_t next;
  uint8_t index;
  uint32_t primask;

  if ((line == NULL) || (length <= 0) ||
      (length > (int)FOC_LINK_TX_LINE_SIZE) ||
      (line[length - 1] != '\n'))
  {
    return FOC_LINK_ERR_FORMAT;
  }

  primask = foc_link_lock();
  next = (uint8_t)(s_tx_head + 1U);
  if (next >= FOC_LINK_TX_QUEUE_DEPTH)
  {
    next = 0U;
  }
  if (next == s_tx_tail)
  {
    s_tx_drop_count++;
    foc_link_unlock(primask);
    return FOC_LINK_ERR_QUEUE_FULL;
  }

  index = s_tx_head;
  memcpy(s_tx_queue[index].data, line, (size_t)length);
  s_tx_queue[index].length = (uint8_t)length;
  s_tx_head = next;
  foc_link_unlock(primask);
  foc_link_tx_kick();
  return FOC_LINK_OK;
}

static int8_t foc_link_queue_priority_line(const char *line, int length)
{
  uint8_t index;
  uint8_t next;
  uint32_t primask;

  if ((line == NULL) || (length <= 0) ||
      (length > (int)FOC_LINK_TX_LINE_SIZE) ||
      (line[length - 1] != '\n'))
  {
    return FOC_LINK_ERR_FORMAT;
  }

  primask = foc_link_lock();
  if (s_tx_busy != 0U)
  {
    index = (uint8_t)(s_tx_tail + 1U);
    if (index >= FOC_LINK_TX_QUEUE_DEPTH)
    {
      index = 0U;
    }
  }
  else
  {
    /* No packet is on the wire: discard every queued stale command. */
    s_tx_head = s_tx_tail;
    index = s_tx_tail;
  }

  memcpy(s_tx_queue[index].data, line, (size_t)length);
  s_tx_queue[index].length = (uint8_t)length;
  next = (uint8_t)(index + 1U);
  if (next >= FOC_LINK_TX_QUEUE_DEPTH)
  {
    next = 0U;
  }
  /* If TX is active this leaves exactly current-packet -> Stop; otherwise it
   * leaves only Stop. Commands queued before the safety request are dropped. */
  s_tx_head = next;
  foc_link_unlock(primask);
  foc_link_tx_kick();
  return FOC_LINK_OK;
}

static int8_t foc_link_format_one(const char *prefix, float value)
{
  char line[FOC_LINK_TX_LINE_SIZE];
  int length;

  if ((prefix == NULL) || (isfinite(value) == 0))
  {
    return FOC_LINK_ERR_ARGUMENT;
  }
  length = snprintf(line, sizeof(line), "%s%.7g\n", prefix, (double)value);
  if ((length <= 0) || (length >= (int)sizeof(line)))
  {
    return FOC_LINK_ERR_FORMAT;
  }
  return foc_link_queue_line(line, length);
}

static void foc_link_accept_telemetry(const uint8_t *frame)
{
  uint8_t i;
  uint8_t valid = 1U;

  for (i = 0U; i < FOC_LINK_TELEMETRY_FLOATS; ++i)
  {
    float value;
    memcpy(&value, &frame[(uint32_t)i * sizeof(float)], sizeof(value));
    s_telemetry.channel[i] = value;
    if (isfinite(value) == 0)
    {
      valid = 0U;
    }
  }
  s_telemetry.frame_count++;
  s_telemetry.last_rx_tick = HAL_GetTick();
  s_telemetry.valid = valid;
}

static void foc_link_parse_byte(uint8_t byte)
{
  if (s_telemetry_count < FOC_LINK_TELEMETRY_SIZE)
  {
    s_telemetry_window[s_telemetry_count++] = byte;
  }

  if (s_telemetry_count < FOC_LINK_TELEMETRY_SIZE)
  {
    return;
  }

  if ((s_telemetry_window[64] == 0x00U) &&
      (s_telemetry_window[65] == 0x00U) &&
      (s_telemetry_window[66] == 0x80U) &&
      (s_telemetry_window[67] == 0x7FU))
  {
    foc_link_accept_telemetry(s_telemetry_window);
    s_telemetry_count = 0U;
  }
  else
  {
    memmove(s_telemetry_window, &s_telemetry_window[1],
            FOC_LINK_TELEMETRY_SIZE - 1U);
    s_telemetry_count = FOC_LINK_TELEMETRY_SIZE - 1U;
  }
}

void FOC_Link_Init(void)
{
  uint32_t primask = foc_link_lock();

  s_tx_head = 0U;
  s_tx_tail = 0U;
  s_tx_busy = 0U;
  s_tx_drop_count = 0U;
  s_rx_head = 0U;
  s_rx_tail = 0U;
  s_rx_overflow_count = 0U;
  s_rx_error_count = 0U;
  s_rx_rearm_pending = 0U;
  s_last_rx_rearm_tick = 0U;
  s_telemetry_count = 0U;
  memset(&s_telemetry, 0, sizeof(s_telemetry));
  memset(&s_command_state, 0, sizeof(s_command_state));
  s_safety_inhibit = 0U;
  s_safety_stop_pending = 0U;
  foc_link_unlock(primask);

  foc_link_arm_rx();
}

void FOC_Link_Process(void)
{
  uint16_t tail;
  uint8_t send_safety_stop = 0U;
  uint32_t now = HAL_GetTick();

  if ((s_rx_rearm_pending != 0U) &&
      ((now - s_last_rx_rearm_tick) >= 10U))
  {
    s_last_rx_rearm_tick = now;
    foc_link_arm_rx();
  }

  while (s_rx_tail != s_rx_head)
  {
    tail = s_rx_tail;
    foc_link_parse_byte(s_rx_ring[tail]);
    tail++;
    if (tail >= FOC_LINK_RX_RING_SIZE)
    {
      tail = 0U;
    }
    s_rx_tail = tail;
  }
  {
    uint32_t primask = foc_link_lock();
    if (s_safety_stop_pending != 0U)
    {
      s_safety_stop_pending = 0U;
      send_safety_stop = 1U;
    }
    foc_link_unlock(primask);
  }
  if (send_safety_stop != 0U)
  {
    (void)FOC_Link_SendStop();
  }
  foc_link_tx_kick();
}

void FOC_Link_SetSafetyInhibit(uint8_t inhibit)
{
  uint32_t primask = foc_link_lock();
  inhibit = (inhibit != 0U) ? 1U : 0U;
  if ((inhibit != 0U) && (s_safety_inhibit == 0U))
  {
    s_safety_stop_pending = 1U;
  }
  s_safety_inhibit = inhibit;
  foc_link_unlock(primask);
}

int8_t FOC_Link_SendSpeed(float value)
{
  if ((value != 0.0f) &&
      ((s_safety_inhibit != 0U) || (Safety_IsMotionAllowed() == 0U)))
  {
    return FOC_LINK_ERR_SAFETY;
  }
  int8_t result = foc_link_format_one("Speed:", value);

  if (result == FOC_LINK_OK)
  {
    s_command_state.speed_target = value;
    s_command_state.mode = FOC_LINK_CONTROL_SPEED;
    Safety_SetExternalMotionActive((value != 0.0f) ? 1U : 0U);
  }
  return result;
}

int8_t FOC_Link_SendAngle(float value)
{
  if ((s_safety_inhibit != 0U) || (Safety_IsMotionAllowed() == 0U))
  {
    return FOC_LINK_ERR_SAFETY;
  }
  int8_t result = foc_link_format_one("Angle:", value);

  if (result == FOC_LINK_OK)
  {
    s_command_state.position_target = value;
    s_command_state.mode = FOC_LINK_CONTROL_POSITION;
    Safety_SetExternalMotionActive(1U);
  }
  return result;
}

int8_t FOC_Link_SendTorque(float value)
{
  if ((value != 0.0f) &&
      ((s_safety_inhibit != 0U) || (Safety_IsMotionAllowed() == 0U)))
  {
    return FOC_LINK_ERR_SAFETY;
  }
  int8_t result = foc_link_format_one("Torque:", value);

  if (result == FOC_LINK_OK)
  {
    s_command_state.torque_target = value;
    s_command_state.mode = FOC_LINK_CONTROL_TORQUE;
    Safety_SetExternalMotionActive((value != 0.0f) ? 1U : 0U);
  }
  return result;
}

int8_t FOC_Link_SendMotor(uint8_t motor_id)
{
  char line[FOC_LINK_TX_LINE_SIZE];
  int length;
  int8_t result;

  if ((s_safety_inhibit != 0U) || (Safety_IsMotionAllowed() == 0U))
  {
    return FOC_LINK_ERR_SAFETY;
  }
  if (motor_id > 1U)
  {
    return FOC_LINK_ERR_ARGUMENT;
  }
  length = snprintf(line, sizeof(line), "Motor:%u\n", (unsigned int)motor_id);
  if ((length <= 0) || (length >= (int)sizeof(line)))
  {
    return FOC_LINK_ERR_FORMAT;
  }
  result = foc_link_queue_line(line, length);
  if (result == FOC_LINK_OK)
  {
    memset(&s_command_state, 0, sizeof(s_command_state));
    Safety_SetExternalMotionActive(0U);
  }
  return result;
}

int8_t FOC_Link_SendSensorless(void)
{
  static const char line[] = "Sensorless\n";
  int8_t result;

  if ((s_safety_inhibit != 0U) || (Safety_IsMotionAllowed() == 0U))
  {
    return FOC_LINK_ERR_SAFETY;
  }
  result = foc_link_queue_line(line, (int)(sizeof(line) - 1U));

  if (result == FOC_LINK_OK)
  {
    memset(&s_command_state, 0, sizeof(s_command_state));
    Safety_SetExternalMotionActive(0U);
  }
  return result;
}

int8_t FOC_Link_SendLock(void)
{
  static const char line[] = "Lock\n";

  if ((s_safety_inhibit != 0U) || (Safety_IsMotionAllowed() == 0U))
  {
    return FOC_LINK_ERR_SAFETY;
  }
  return foc_link_queue_line(line, (int)(sizeof(line) - 1U));
}

int8_t FOC_Link_SendStop(void)
{
  static const char line[] = "Speed:0\n";
  int8_t result = foc_link_queue_priority_line(line,
                                                (int)(sizeof(line) - 1U));

  if (result == FOC_LINK_OK)
  {
    s_command_state.speed_target = 0.0f;
    s_command_state.torque_target = 0.0f;
    s_command_state.mode = FOC_LINK_CONTROL_IDLE;
    Safety_SetExternalMotionActive(0U);
  }
  return result;
}

void FOC_Link_GetTelemetry(FOC_LinkTelemetry *telemetry)
{
  if (telemetry == NULL)
  {
    return;
  }
  memcpy(telemetry, &s_telemetry, sizeof(*telemetry));
  telemetry->rx_overflow_count = s_rx_overflow_count;
  telemetry->rx_error_count = s_rx_error_count;
  telemetry->tx_drop_count = s_tx_drop_count;
}

void FOC_Link_GetCommandState(FOC_LinkCommandState *state)
{
  if (state == NULL)
  {
    return;
  }
  memcpy(state, &s_command_state, sizeof(*state));
}

uint8_t FOC_Link_IsTelemetryAlive(uint32_t timeout_ms)
{
  if ((s_telemetry.valid == 0U) || (timeout_ms == 0U))
  {
    return 0U;
  }
  return ((HAL_GetTick() - s_telemetry.last_rx_tick) < timeout_ms) ? 1U : 0U;
}

void FOC_Link_UartRxCompleteFromISR(UART_HandleTypeDef *huart)
{
  uint16_t next;

  if ((huart == NULL) || (huart->Instance != UART4))
  {
    return;
  }
  next = (uint16_t)(s_rx_head + 1U);
  if (next >= FOC_LINK_RX_RING_SIZE)
  {
    next = 0U;
  }
  if (next == s_rx_tail)
  {
    s_rx_overflow_count++;
  }
  else
  {
    s_rx_ring[s_rx_head] = s_rx_byte;
    s_rx_head = next;
  }
  foc_link_arm_rx();
}

void FOC_Link_UartTxCompleteFromISR(UART_HandleTypeDef *huart)
{
  if ((huart == NULL) || (huart->Instance != UART4))
  {
    return;
  }
  if (s_tx_busy != 0U)
  {
    uint8_t next = (uint8_t)(s_tx_tail + 1U);
    if (next >= FOC_LINK_TX_QUEUE_DEPTH)
    {
      next = 0U;
    }
    s_tx_tail = next;
    s_tx_busy = 0U;
  }
  foc_link_tx_kick();
}

void FOC_Link_UartErrorFromISR(UART_HandleTypeDef *huart)
{
  if ((huart == NULL) || (huart->Instance != UART4))
  {
    return;
  }
  s_rx_error_count++;
  foc_link_arm_rx();
}
