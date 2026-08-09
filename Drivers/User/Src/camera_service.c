#include "camera_service.h"

#include "main.h"
#include "ov5640.h"

#include <stddef.h>
#include <string.h>

#define CAMERA_OV5640_I2C_ADDRESS       0x78U
#define CAMERA_RESET_ASSERT_MS          2U
#define CAMERA_PWDN_RELEASE_MS          10U
#define CAMERA_RESET_RELEASE_MS         20U
#define CAMERA_AE_SETTLE_MS             100U
#define CAMERA_DMA_WORD_COUNT           (CAMERA_JPEG_BUFFER_CAPACITY / sizeof(uint32_t))
#define CAMERA_SCCB_SCL_PIN              GPIO_PIN_14
#define CAMERA_SCCB_SDA_PIN              GPIO_PIN_15
#define CAMERA_SCCB_GPIO_PORT             GPIOF
#define CAMERA_SCCB_DELAY_US               2U
#define CAMERA_ID_READ_RETRIES            3U
#define CAMERA_OV5640_ID_HIGH_REG          0x300AU
#define CAMERA_OV5640_ID_LOW_REG           0x300BU

extern I2C_HandleTypeDef hi2c4;
extern DCMI_HandleTypeDef hdcmi;

static OV5640_Object_t s_sensor;

/* The compressed input must coexist with the RGB565 media pool during JPEG
   decode. DMA1 can access D2 SRAM and the project MPU keeps it non-cacheable. */
static uint8_t s_jpeg_buffer[CAMERA_JPEG_BUFFER_CAPACITY]
  __attribute__((section(".ram_d2"), aligned(32)));

static Camera_State s_state = CAMERA_STATE_OFF;
static Camera_Result s_last_result = CAMERA_RESULT_OK;
static uint32_t s_sensor_id = 0U;
static uint32_t s_frame_size = 0U;
static uint32_t s_dma_bytes_received = 0U;
static uint32_t s_dcmi_error = 0U;
static uint32_t s_capture_started_at = 0U;
static uint32_t s_capture_timeout_ms = CAMERA_CAPTURE_TIMEOUT_DEFAULT;
static uint8_t *s_frame_data = NULL;
static uint8_t s_sensor_active = 0U;
static Camera_InitStage s_init_stage = CAMERA_INIT_STAGE_NONE;
static uint32_t s_i2c_error = HAL_I2C_ERROR_NONE;
static uint32_t s_i2c_state = HAL_I2C_STATE_RESET;
static uint8_t s_scl_level = 0U;
static uint8_t s_sda_level = 0U;
static uint8_t s_pwdn_active_level = 1U;
static uint8_t s_reset_release_level = 0U;
static uint8_t s_sccb_nack_phase = 0U;

static volatile uint8_t s_capture_active = 0U;
static volatile uint8_t s_frame_event = 0U;
static volatile uint8_t s_error_event = 0U;

static int32_t Camera_Bus_Init(void);
static int32_t Camera_Bus_DeInit(void);
static int32_t Camera_Bus_Write(uint16_t address, uint16_t reg,
                                uint8_t *data, uint16_t length);
static int32_t Camera_Bus_Read(uint16_t address, uint16_t reg,
                               uint8_t *data, uint16_t length);
static int32_t Camera_Bus_GetTick(void);
static int32_t Camera_ReadSensorId(uint32_t *sensor_id);
static void Camera_SCCB_ConfigurePins(void);
static void Camera_SCCB_DelayUs(uint32_t delay_us);
static void Camera_SCCB_Start(void);
static void Camera_SCCB_Stop(void);
static uint8_t Camera_SCCB_WriteByte(uint8_t value);
static uint8_t Camera_SCCB_ReadByte(void);
static void Camera_SCCB_SendAck(uint8_t nack);
static void Camera_RecordSCCBStatus(uint8_t success);
static void Camera_ResetAssert(void);
static void Camera_ResetRelease(void);
static void Camera_HoldPins(void);
static void Camera_StopHardware(void);
static Camera_Result Camera_FinishCapture(void);
static Camera_Result Camera_FindJpeg(uint32_t received_bytes);
static HAL_StatusTypeDef Camera_I2C_Recover(void);
static void Camera_I2C_ConfigurePins(void);
static void Camera_RecordI2CStatus(void);

static void Camera_RecordI2CStatus(void)
{
  s_i2c_error = HAL_I2C_GetError(&hi2c4);
  s_i2c_state = (uint32_t)HAL_I2C_GetState(&hi2c4);
  s_scl_level = (HAL_GPIO_ReadPin(CAMERA_SCCB_GPIO_PORT,
                                 CAMERA_SCCB_SCL_PIN) == GPIO_PIN_SET) ? 1U : 0U;
  s_sda_level = (HAL_GPIO_ReadPin(CAMERA_SCCB_GPIO_PORT,
                                 CAMERA_SCCB_SDA_PIN) == GPIO_PIN_SET) ? 1U : 0U;
}

static void Camera_RecordSCCBStatus(uint8_t success)
{
  s_i2c_error = (success != 0U) ? HAL_I2C_ERROR_NONE : HAL_I2C_ERROR_AF;
  s_i2c_state = HAL_I2C_STATE_READY;
  s_scl_level = (HAL_GPIO_ReadPin(CAMERA_SCCB_GPIO_PORT,
                                 CAMERA_SCCB_SCL_PIN) == GPIO_PIN_SET) ? 1U : 0U;
  s_sda_level = (HAL_GPIO_ReadPin(CAMERA_SCCB_GPIO_PORT,
                                 CAMERA_SCCB_SDA_PIN) == GPIO_PIN_SET) ? 1U : 0U;
}

static void Camera_ResetAssert(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  /* PC4 is wired directly to the sensor-side OV_RESET net.  Assert it only by
     sinking current; the camera board supplies the high level from DOVDD_2V8. */
  HAL_GPIO_WritePin(OV_RESET_GPIO_Port, OV_RESET_Pin, GPIO_PIN_RESET);
  gpio.Pin = OV_RESET_Pin;
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OV_RESET_GPIO_Port, &gpio);
}

static void Camera_ResetRelease(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  /* Writing SET to an open-drain output releases PC4.  R3 on the camera board
     must then pull OV_RESET to DOVDD_2V8; do not inject a 3.3 V high level. */
  HAL_GPIO_WritePin(OV_RESET_GPIO_Port, OV_RESET_Pin, GPIO_PIN_SET);
  gpio.Pin = OV_RESET_Pin;
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OV_RESET_GPIO_Port, &gpio);
}

static void Camera_SCCB_DelayUs(uint32_t delay_us)
{
  uint32_t start;
  uint32_t cycles;

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U)
  {
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  }

  cycles = (SystemCoreClock / 1000000U) * delay_us;
  start = DWT->CYCCNT;
  while ((uint32_t)(DWT->CYCCNT - start) < cycles)
  {
  }
}

static void Camera_SCCB_ConfigurePins(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOF_CLK_ENABLE();
  gpio.Pin = CAMERA_SCCB_SCL_PIN | CAMERA_SCCB_SDA_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CAMERA_SCCB_GPIO_PORT, &gpio);
  HAL_GPIO_WritePin(CAMERA_SCCB_GPIO_PORT,
                    CAMERA_SCCB_SCL_PIN | CAMERA_SCCB_SDA_PIN,
                    GPIO_PIN_SET);
  Camera_SCCB_DelayUs(CAMERA_SCCB_DELAY_US);
}

static inline void Camera_SCCB_SetSCL(uint8_t high)
{
  CAMERA_SCCB_GPIO_PORT->BSRR = (high != 0U)
                                 ? CAMERA_SCCB_SCL_PIN
                                 : ((uint32_t)CAMERA_SCCB_SCL_PIN << 16U);
}

static inline void Camera_SCCB_SetSDA(uint8_t high)
{
  CAMERA_SCCB_GPIO_PORT->BSRR = (high != 0U)
                                 ? CAMERA_SCCB_SDA_PIN
                                 : ((uint32_t)CAMERA_SCCB_SDA_PIN << 16U);
}

static inline void Camera_SCCB_SDAOutput(void)
{
  MODIFY_REG(CAMERA_SCCB_GPIO_PORT->MODER,
             GPIO_MODER_MODE15_Msk, GPIO_MODER_MODE15_0);
}

static inline void Camera_SCCB_SDAInput(void)
{
  CLEAR_BIT(CAMERA_SCCB_GPIO_PORT->MODER, GPIO_MODER_MODE15_Msk);
}

static void Camera_SCCB_Start(void)
{
  Camera_SCCB_SDAOutput();
  Camera_SCCB_SetSDA(1U);
  Camera_SCCB_SetSCL(1U);
  Camera_SCCB_DelayUs(CAMERA_SCCB_DELAY_US);
  Camera_SCCB_SetSDA(0U);
  Camera_SCCB_DelayUs(CAMERA_SCCB_DELAY_US);
  Camera_SCCB_SetSCL(0U);
}

static void Camera_SCCB_Stop(void)
{
  Camera_SCCB_SDAOutput();
  Camera_SCCB_SetSDA(0U);
  Camera_SCCB_DelayUs(CAMERA_SCCB_DELAY_US);
  Camera_SCCB_SetSCL(1U);
  Camera_SCCB_DelayUs(CAMERA_SCCB_DELAY_US);
  Camera_SCCB_SetSDA(1U);
  Camera_SCCB_DelayUs(CAMERA_SCCB_DELAY_US);
}

static uint8_t Camera_SCCB_WriteByte(uint8_t value)
{
  uint32_t bit;
  uint8_t nack;

  Camera_SCCB_SDAOutput();
  for (bit = 0U; bit < 8U; ++bit)
  {
    Camera_SCCB_SetSDA((value & 0x80U) != 0U ? 1U : 0U);
    value <<= 1U;
    Camera_SCCB_DelayUs(CAMERA_SCCB_DELAY_US);
    Camera_SCCB_SetSCL(1U);
    Camera_SCCB_DelayUs(CAMERA_SCCB_DELAY_US);
    Camera_SCCB_SetSCL(0U);
  }

  /* Release SDA only for the ninth clock so the sensor can drive ACK low. */
  Camera_SCCB_SDAInput();
  Camera_SCCB_DelayUs(CAMERA_SCCB_DELAY_US);
  Camera_SCCB_SetSCL(1U);
  Camera_SCCB_DelayUs(CAMERA_SCCB_DELAY_US);
  nack = (HAL_GPIO_ReadPin(CAMERA_SCCB_GPIO_PORT,
                          CAMERA_SCCB_SDA_PIN) == GPIO_PIN_SET) ? 1U : 0U;
  Camera_SCCB_SetSCL(0U);
  Camera_SCCB_SDAOutput();
  return nack;
}

static uint8_t Camera_SCCB_ReadByte(void)
{
  uint32_t bit;
  uint8_t value = 0U;

  Camera_SCCB_SDAInput();
  for (bit = 0U; bit < 8U; ++bit)
  {
    Camera_SCCB_DelayUs(CAMERA_SCCB_DELAY_US);
    Camera_SCCB_SetSCL(1U);
    value <<= 1U;
    if (HAL_GPIO_ReadPin(CAMERA_SCCB_GPIO_PORT,
                        CAMERA_SCCB_SDA_PIN) == GPIO_PIN_SET)
    {
      value |= 1U;
    }
    Camera_SCCB_DelayUs(CAMERA_SCCB_DELAY_US);
    Camera_SCCB_SetSCL(0U);
  }
  Camera_SCCB_SDAOutput();
  return value;
}

static void Camera_SCCB_SendAck(uint8_t nack)
{
  Camera_SCCB_SDAOutput();
  Camera_SCCB_SetSDA((nack != 0U) ? 1U : 0U);
  Camera_SCCB_DelayUs(CAMERA_SCCB_DELAY_US);
  Camera_SCCB_SetSCL(1U);
  Camera_SCCB_DelayUs(CAMERA_SCCB_DELAY_US);
  Camera_SCCB_SetSCL(0U);
  Camera_SCCB_DelayUs(CAMERA_SCCB_DELAY_US);
}

static void Camera_I2C_ConfigurePins(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOF_CLK_ENABLE();
  gpio.Pin = CAMERA_SCCB_SCL_PIN | CAMERA_SCCB_SDA_PIN;
  gpio.Mode = GPIO_MODE_AF_OD;
  /* R10/R11 on the camera board pull SCCB to DOVDD_2V8.  Do not enable the
     STM32's 3.3 V internal pull-ups on these sensor IO pins. */
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate = GPIO_AF4_I2C4;
  HAL_GPIO_Init(CAMERA_SCCB_GPIO_PORT, &gpio);
}

static HAL_StatusTypeDef Camera_I2C_Recover(void)
{
  GPIO_InitTypeDef gpio = {0};
  HAL_StatusTypeDef status;
  uint32_t pulse;

  s_init_stage = CAMERA_INIT_STAGE_I2C_RECOVERY;

  /* Reset the I2C state machine before temporarily using PF14/PF15 as GPIO.
     The sensor is held in reset/power-down while these recovery clocks run. */
  (void)HAL_I2C_DeInit(&hi2c4);
  __HAL_RCC_GPIOF_CLK_ENABLE();

  gpio.Pin = CAMERA_SCCB_SCL_PIN | CAMERA_SCCB_SDA_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CAMERA_SCCB_GPIO_PORT, &gpio);

  HAL_GPIO_WritePin(CAMERA_SCCB_GPIO_PORT,
                    CAMERA_SCCB_SCL_PIN | CAMERA_SCCB_SDA_PIN,
                    GPIO_PIN_SET);
  HAL_Delay(1U);
  for (pulse = 0U; pulse < 9U; ++pulse)
  {
    HAL_GPIO_WritePin(CAMERA_SCCB_GPIO_PORT, CAMERA_SCCB_SCL_PIN,
                      GPIO_PIN_RESET);
    HAL_Delay(1U);
    HAL_GPIO_WritePin(CAMERA_SCCB_GPIO_PORT, CAMERA_SCCB_SCL_PIN,
                      GPIO_PIN_SET);
    HAL_Delay(1U);
  }

  /* Generate a STOP condition and return the pins to I2C4 alternate mode. */
  HAL_GPIO_WritePin(CAMERA_SCCB_GPIO_PORT, CAMERA_SCCB_SDA_PIN,
                    GPIO_PIN_RESET);
  HAL_Delay(1U);
  HAL_GPIO_WritePin(CAMERA_SCCB_GPIO_PORT, CAMERA_SCCB_SCL_PIN,
                    GPIO_PIN_SET);
  HAL_Delay(1U);
  HAL_GPIO_WritePin(CAMERA_SCCB_GPIO_PORT, CAMERA_SCCB_SDA_PIN,
                    GPIO_PIN_SET);
  HAL_Delay(1U);

  status = HAL_I2C_Init(&hi2c4);
  if (status == HAL_OK)
  {
    status = HAL_I2CEx_ConfigAnalogFilter(&hi2c4,
                                          I2C_ANALOGFILTER_ENABLE);
  }
  if (status == HAL_OK)
  {
    status = HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0U);
  }
  Camera_I2C_ConfigurePins();
  Camera_RecordI2CStatus();
  return status;
}

static int32_t Camera_Bus_Init(void)
{
  if ((hi2c4.Instance != I2C4) ||
      (HAL_I2C_GetState(&hi2c4) == HAL_I2C_STATE_RESET))
  {
    Camera_RecordI2CStatus();
    return OV5640_ERROR;
  }

  /* Bit-bang the SCCB framing while keeping both lines open-drain.  The camera
     board's R10/R11 provide the valid 2.8 V high level; SDA is also released
     explicitly during ACK and read cycles. */
  Camera_SCCB_ConfigurePins();
  Camera_RecordSCCBStatus(1U);
  return OV5640_OK;
}

static int32_t Camera_Bus_DeInit(void)
{
  /* I2C4 belongs to CubeMX and may be reused; do not de-initialize it here. */
  return OV5640_OK;
}

static int32_t Camera_Bus_Write(uint16_t address, uint16_t reg,
                                uint8_t *data, uint16_t length)
{
  uint16_t index;
  uint8_t failed = 0U;

  if ((data == NULL) || (length == 0U))
  {
    return OV5640_ERROR;
  }

  s_sccb_nack_phase = 0U;
  Camera_SCCB_Start();
  if (Camera_SCCB_WriteByte((uint8_t)address) != 0U)
  {
    failed = 1U;
    s_sccb_nack_phase = 1U;
  }
  else if (Camera_SCCB_WriteByte((uint8_t)(reg >> 8U)) != 0U)
  {
    failed = 1U;
    s_sccb_nack_phase = 2U;
  }
  else if (Camera_SCCB_WriteByte((uint8_t)reg) != 0U)
  {
    failed = 1U;
    s_sccb_nack_phase = 3U;
  }
  for (index = 0U; (index < length) && (failed == 0U); ++index)
  {
    if (Camera_SCCB_WriteByte(data[index]) != 0U)
    {
      failed = 1U;
      s_sccb_nack_phase = 4U;
    }
  }
  Camera_SCCB_Stop();
  Camera_RecordSCCBStatus((failed == 0U) ? 1U : 0U);
  return (failed == 0U) ? OV5640_OK : OV5640_ERROR;
}

static int32_t Camera_Bus_Read(uint16_t address, uint16_t reg,
                               uint8_t *data, uint16_t length)
{
  uint16_t index;
  uint8_t failed = 0U;

  if ((data == NULL) || (length == 0U))
  {
    return OV5640_ERROR;
  }

  s_sccb_nack_phase = 0U;
  Camera_SCCB_Start();
  if (Camera_SCCB_WriteByte((uint8_t)address) != 0U)
  {
    failed = 1U;
    s_sccb_nack_phase = 1U;
  }
  else if (Camera_SCCB_WriteByte((uint8_t)(reg >> 8U)) != 0U)
  {
    failed = 1U;
    s_sccb_nack_phase = 2U;
  }
  else if (Camera_SCCB_WriteByte((uint8_t)reg) != 0U)
  {
    failed = 1U;
    s_sccb_nack_phase = 3U;
  }
  Camera_SCCB_Stop();
  if (failed != 0U)
  {
    Camera_RecordSCCBStatus(0U);
    return OV5640_ERROR;
  }

  Camera_SCCB_Start();
  failed = Camera_SCCB_WriteByte((uint8_t)(address | 0x01U));
  if (failed != 0U)
  {
    s_sccb_nack_phase = 5U;
  }
  if (failed == 0U)
  {
    for (index = 0U; index < length; ++index)
    {
      data[index] = Camera_SCCB_ReadByte();
      Camera_SCCB_SendAck((index == (uint16_t)(length - 1U)) ? 1U : 0U);
    }
  }
  Camera_SCCB_Stop();
  Camera_RecordSCCBStatus((failed == 0U) ? 1U : 0U);
  return (failed == 0U) ? OV5640_OK : OV5640_ERROR;
}

static int32_t Camera_ReadSensorId(uint32_t *sensor_id)
{
  uint32_t attempt;
  uint8_t id_high;
  uint8_t id_low;

  if (sensor_id == NULL)
  {
    return OV5640_ERROR;
  }

  *sensor_id = 0U;
  for (attempt = 0U; attempt < CAMERA_ID_READ_RETRIES; ++attempt)
  {
    if ((Camera_Bus_Read(CAMERA_OV5640_I2C_ADDRESS,
                         CAMERA_OV5640_ID_HIGH_REG, &id_high, 1U) == OV5640_OK) &&
        (Camera_Bus_Read(CAMERA_OV5640_I2C_ADDRESS,
                         CAMERA_OV5640_ID_LOW_REG, &id_low, 1U) == OV5640_OK))
    {
      *sensor_id = ((uint32_t)id_high << 8U) | id_low;
      return OV5640_OK;
    }

    HAL_Delay(10U);
  }

  return OV5640_ERROR;
}

static int32_t Camera_Bus_GetTick(void)
{
  return (int32_t)HAL_GetTick();
}

static void Camera_HoldPins(void)
{
  /* Power-down first, then assert the sensor's active-low OV_RESET net. */
  HAL_GPIO_WritePin(OV_PWDN_GPIO_Port, OV_PWDN_Pin, GPIO_PIN_SET);
  Camera_ResetAssert();
}

void Camera_Service_BootHold(void)
{
  Camera_HoldPins();
  s_sensor_active = 0U;
  s_capture_active = 0U;
  s_frame_event = 0U;
  s_error_event = 0U;
  s_state = CAMERA_STATE_OFF;
  s_last_result = CAMERA_RESULT_OK;
  s_init_stage = CAMERA_INIT_STAGE_NONE;
  Camera_RecordI2CStatus();
}

Camera_Result Camera_Service_Init(void)
{
  OV5640_IO_t io;

  if ((s_state == CAMERA_STATE_INITIALIZING) ||
      (s_state == CAMERA_STATE_CAPTURING))
  {
    return CAMERA_RESULT_BUSY;
  }
  if ((s_state == CAMERA_STATE_READY) && (s_sensor_active != 0U))
  {
    return CAMERA_RESULT_OK;
  }

  s_state = CAMERA_STATE_INITIALIZING;
  s_last_result = CAMERA_RESULT_OK;
  s_sensor_id = 0U;
  s_dcmi_error = 0U;
  s_i2c_error = HAL_I2C_ERROR_NONE;
  s_init_stage = CAMERA_INIT_STAGE_I2C_RECOVERY;
  memset(&s_sensor, 0, sizeof(s_sensor));
  memset(&io, 0, sizeof(io));

  if ((hdcmi.Instance != DCMI) || (hdcmi.DMA_Handle == NULL))
  {
    s_last_result = CAMERA_RESULT_DCMI;
    s_state = CAMERA_STATE_ERROR;
    return s_last_result;
  }

  /* A one-word FIFO threshold prevents a short JPEG tail from remaining in
     the DMA FIFO when DCMI raises its variable-length end-of-frame event. */
  hdcmi.DMA_Handle->Init.FIFOMode = DMA_FIFOMODE_ENABLE;
  hdcmi.DMA_Handle->Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
  if (HAL_DMA_Init(hdcmi.DMA_Handle) != HAL_OK)
  {
    s_last_result = CAMERA_RESULT_DCMI;
    s_state = CAMERA_STATE_ERROR;
    return s_last_result;
  }

  Camera_HoldPins();
  HAL_Delay(CAMERA_RESET_ASSERT_MS);
  if (Camera_I2C_Recover() != HAL_OK)
  {
    s_last_result = CAMERA_RESULT_I2C;
    s_state = CAMERA_STATE_ERROR;
    return s_last_result;
  }

  /* Leave power-down, then release RESET to the board's 2.8 V pull-up. */
  HAL_GPIO_WritePin(OV_PWDN_GPIO_Port, OV_PWDN_Pin, GPIO_PIN_RESET);
  HAL_Delay(CAMERA_PWDN_RELEASE_MS);
  Camera_ResetRelease();
  HAL_Delay(CAMERA_RESET_RELEASE_MS);
  s_pwdn_active_level =
    (HAL_GPIO_ReadPin(OV_PWDN_GPIO_Port, OV_PWDN_Pin) == GPIO_PIN_SET) ? 1U : 0U;
  s_reset_release_level =
    (HAL_GPIO_ReadPin(OV_RESET_GPIO_Port, OV_RESET_Pin) == GPIO_PIN_SET) ? 1U : 0U;
  s_sensor_active = 1U;
  s_init_stage = CAMERA_INIT_STAGE_I2C_PROBE;

  /* Do not attempt SCCB while RESET is physically low.  Preserve the sampled
     P/R levels so the UI reports the electrical cause after hardware is put
     back into its safe sleep state. */
  if (s_reset_release_level == 0U)
  {
    s_i2c_error = HAL_I2C_ERROR_AF;
    s_i2c_state = HAL_I2C_STATE_READY;
    s_sccb_nack_phase = 0U;
    s_last_result = CAMERA_RESULT_I2C;
    Camera_StopHardware();
    s_state = CAMERA_STATE_ERROR;
    return s_last_result;
  }

  io.Init = Camera_Bus_Init;
  io.DeInit = Camera_Bus_DeInit;
  io.Address = CAMERA_OV5640_I2C_ADDRESS;
  io.WriteReg = Camera_Bus_Write;
  io.ReadReg = Camera_Bus_Read;
  io.ModifyReg = NULL;
  io.GetTick = Camera_Bus_GetTick;

  s_sensor.Mode = PARALLEL_MODE;
  if (OV5640_RegisterBusIO(&s_sensor, &io) != OV5640_OK)
  {
    s_last_result = CAMERA_RESULT_I2C;
    Camera_StopHardware();
    s_state = CAMERA_STATE_ERROR;
    return s_last_result;
  }

  s_init_stage = CAMERA_INIT_STAGE_READ_ID;
  /* Detect first, without the software reset performed by OV5640_ReadID().
     This mirrors ov5640_detect() in the supplied ESP32 project.  OV5640_Init
     performs its own reset as the first sensor-configuration operation. */
  if (Camera_ReadSensorId(&s_sensor_id) != OV5640_OK)
  {
    s_last_result = CAMERA_RESULT_I2C;
    Camera_StopHardware();
    s_state = CAMERA_STATE_ERROR;
    return s_last_result;
  }
  if (s_sensor_id != CAMERA_OV5640_SENSOR_ID)
  {
    s_last_result = CAMERA_RESULT_BAD_SENSOR_ID;
    Camera_StopHardware();
    s_state = CAMERA_STATE_ERROR;
    return s_last_result;
  }

  s_init_stage = CAMERA_INIT_STAGE_SENSOR_CONFIG;
  if ((OV5640_Init(&s_sensor, OV5640_R320x240, OV5640_JPEG) != OV5640_OK) ||
      (OV5640_SetPCLK(&s_sensor, OV5640_PCLK_24M) != OV5640_OK))
  {
    s_last_result = CAMERA_RESULT_SENSOR_INIT;
    Camera_StopHardware();
    s_state = CAMERA_STATE_ERROR;
    return s_last_result;
  }

  /* Let automatic exposure and white balance settle before the first snapshot. */
  HAL_Delay(CAMERA_AE_SETTLE_MS);
  s_init_stage = CAMERA_INIT_STAGE_NONE;
  s_state = CAMERA_STATE_READY;
  return CAMERA_RESULT_OK;
}

Camera_Result Camera_Service_StartSnapshot(uint32_t timeout_ms)
{
  if (s_state == CAMERA_STATE_CAPTURING)
  {
    return CAMERA_RESULT_BUSY;
  }
  if ((s_state != CAMERA_STATE_READY) || (s_sensor_active == 0U))
  {
    return CAMERA_RESULT_NOT_READY;
  }
  s_frame_data = NULL;
  s_frame_size = 0U;
  s_dma_bytes_received = 0U;
  s_dcmi_error = 0U;
  s_frame_event = 0U;
  s_error_event = 0U;
  s_init_stage = CAMERA_INIT_STAGE_CAPTURE;
  s_capture_timeout_ms = (timeout_ms == 0U)
                           ? CAMERA_CAPTURE_TIMEOUT_DEFAULT : timeout_ms;

  if (OV5640_Start(&s_sensor) != OV5640_OK)
  {
    s_last_result = CAMERA_RESULT_SENSOR_INIT;
    Camera_StopHardware();
    s_state = CAMERA_STATE_ERROR;
    return s_last_result;
  }

  hdcmi.ErrorCode = HAL_DCMI_ERROR_NONE;
  __HAL_DCMI_CLEAR_FLAG(&hdcmi, DCMI_FLAG_FRAMERI | DCMI_FLAG_OVRRI |
                                DCMI_FLAG_ERRRI | DCMI_FLAG_VSYNCRI |
                                DCMI_FLAG_LINERI);
  __HAL_DCMI_DISABLE_IT(&hdcmi, DCMI_IT_LINE | DCMI_IT_VSYNC);
  __HAL_DCMI_ENABLE_IT(&hdcmi, DCMI_IT_FRAME | DCMI_IT_ERR | DCMI_IT_OVR);

  s_capture_active = 1U;
  s_capture_started_at = HAL_GetTick();
  s_state = CAMERA_STATE_CAPTURING;

  if (HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT,
                         (uint32_t)s_jpeg_buffer,
                         CAMERA_DMA_WORD_COUNT) != HAL_OK)
  {
    s_capture_active = 0U;
    s_last_result = CAMERA_RESULT_DCMI;
    Camera_StopHardware();
    s_state = CAMERA_STATE_ERROR;
    return s_last_result;
  }

  return CAMERA_RESULT_OK;
}

void Camera_Service_Process(void)
{
  if (s_state != CAMERA_STATE_CAPTURING)
  {
    return;
  }

  if (s_error_event != 0U)
  {
    s_dcmi_error = HAL_DCMI_GetError(&hdcmi);
    s_last_result = CAMERA_RESULT_DCMI;
    Camera_StopHardware();
    s_state = CAMERA_STATE_ERROR;
    return;
  }

  if (s_frame_event != 0U)
  {
    s_last_result = Camera_FinishCapture();
    Camera_StopHardware();
    s_state = (s_last_result == CAMERA_RESULT_OK)
                ? CAMERA_STATE_FRAME_READY : CAMERA_STATE_ERROR;
    return;
  }

  if ((HAL_GetTick() - s_capture_started_at) >= s_capture_timeout_ms)
  {
    s_last_result = CAMERA_RESULT_TIMEOUT;
    Camera_StopHardware();
    s_state = CAMERA_STATE_ERROR;
  }
}

static Camera_Result Camera_FinishCapture(void)
{
  uint32_t remaining_words;

  /* Frame IRQ precedes this main-loop service by several ms, allowing the DMA
     FIFO to commit its final word before the counter is sampled. */
  __DSB();
  remaining_words = __HAL_DMA_GET_COUNTER(hdcmi.DMA_Handle);
  if (remaining_words > CAMERA_DMA_WORD_COUNT)
  {
    return CAMERA_RESULT_DCMI;
  }

  s_dma_bytes_received = (CAMERA_DMA_WORD_COUNT - remaining_words) * sizeof(uint32_t);
  return Camera_FindJpeg(s_dma_bytes_received);
}

static Camera_Result Camera_FindJpeg(uint32_t received_bytes)
{
  uint32_t start;
  uint32_t end;

  if (received_bytes > CAMERA_JPEG_BUFFER_CAPACITY)
  {
    received_bytes = CAMERA_JPEG_BUFFER_CAPACITY;
  }

  for (start = 0U; (start + 1U) < received_bytes; ++start)
  {
    if ((s_jpeg_buffer[start] == 0xFFU) &&
        (s_jpeg_buffer[start + 1U] == 0xD8U))
    {
      break;
    }
  }
  if ((start + 1U) >= received_bytes)
  {
    return (received_bytes >= CAMERA_JPEG_BUFFER_CAPACITY)
             ? CAMERA_RESULT_BUFFER_FULL : CAMERA_RESULT_INVALID_JPEG;
  }

  for (end = start + 2U; (end + 1U) < received_bytes; ++end)
  {
    if ((s_jpeg_buffer[end] == 0xFFU) &&
        (s_jpeg_buffer[end + 1U] == 0xD9U))
    {
      s_frame_size = (end + 2U) - start;
      if (start != 0U)
      {
        memmove(s_jpeg_buffer, &s_jpeg_buffer[start], s_frame_size);
      }
      s_frame_data = s_jpeg_buffer;
      return CAMERA_RESULT_OK;
    }
  }

  return (received_bytes >= CAMERA_JPEG_BUFFER_CAPACITY)
           ? CAMERA_RESULT_BUFFER_FULL : CAMERA_RESULT_INVALID_JPEG;
}

Camera_Result Camera_Service_CaptureJpeg(const uint8_t **jpeg_data,
                                        uint32_t *jpeg_size,
                                        uint32_t timeout_ms)
{
  Camera_Result result;

  if ((jpeg_data == NULL) || (jpeg_size == NULL))
  {
    return CAMERA_RESULT_INVALID_ARGUMENT;
  }

  result = Camera_Service_Init();
  if (result != CAMERA_RESULT_OK)
  {
    return result;
  }
  result = Camera_Service_StartSnapshot(timeout_ms);
  if (result != CAMERA_RESULT_OK)
  {
    return result;
  }

  while (s_state == CAMERA_STATE_CAPTURING)
  {
    Camera_Service_Process();
    HAL_Delay(1U);
  }

  return Camera_Service_GetSnapshot(jpeg_data, jpeg_size);
}

Camera_Result Camera_Service_GetSnapshot(const uint8_t **jpeg_data,
                                         uint32_t *jpeg_size)
{
  if ((jpeg_data == NULL) || (jpeg_size == NULL))
  {
    return CAMERA_RESULT_INVALID_ARGUMENT;
  }
  if ((s_state != CAMERA_STATE_FRAME_READY) ||
      (s_frame_data == NULL) || (s_frame_size == 0U))
  {
    return (s_last_result == CAMERA_RESULT_OK)
             ? CAMERA_RESULT_NOT_READY : s_last_result;
  }

  *jpeg_data = s_frame_data;
  *jpeg_size = s_frame_size;
  return CAMERA_RESULT_OK;
}

static void Camera_StopHardware(void)
{
  if (s_capture_active != 0U)
  {
    (void)HAL_DCMI_Stop(&hdcmi);
  }
  s_capture_active = 0U;

  if (s_sensor_active != 0U)
  {
    if (s_sensor.IsInitialized != 0U)
    {
      (void)OV5640_Stop(&s_sensor);
      (void)OV5640_DeInit(&s_sensor);
    }
    Camera_HoldPins();
    s_sensor_active = 0U;
  }
}

void Camera_Service_Sleep(void)
{
  Camera_StopHardware();
  s_frame_event = 0U;
  s_error_event = 0U;
  s_frame_data = NULL;
  s_frame_size = 0U;
  s_state = CAMERA_STATE_OFF;
  s_last_result = CAMERA_RESULT_OK;
}

Camera_State Camera_Service_GetState(void)
{
  return s_state;
}

Camera_Result Camera_Service_GetLastResult(void)
{
  return s_last_result;
}

void Camera_Service_GetDiagnostics(Camera_Diagnostics *diagnostics)
{
  if (diagnostics == NULL)
  {
    return;
  }

  diagnostics->state = s_state;
  diagnostics->last_result = s_last_result;
  diagnostics->init_stage = s_init_stage;
  diagnostics->sensor_id = s_sensor_id;
  diagnostics->frame_size = s_frame_size;
  diagnostics->dma_bytes_received = s_dma_bytes_received;
  diagnostics->dcmi_error = s_dcmi_error;
  diagnostics->i2c_error = s_i2c_error;
  diagnostics->i2c_state = s_i2c_state;
  diagnostics->scl_level = s_scl_level;
  diagnostics->sda_level = s_sda_level;
  diagnostics->pwdn_level = s_pwdn_active_level;
  diagnostics->reset_level = s_reset_release_level;
  diagnostics->sccb_nack_phase = s_sccb_nack_phase;
}

void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *camera_dcmi)
{
  if ((camera_dcmi != NULL) && (camera_dcmi->Instance == DCMI) &&
      (s_capture_active != 0U))
  {
    s_frame_event = 1U;
  }
}

void HAL_DCMI_ErrorCallback(DCMI_HandleTypeDef *camera_dcmi)
{
  if ((camera_dcmi != NULL) && (camera_dcmi->Instance == DCMI) &&
      (s_capture_active != 0U))
  {
    s_error_event = 1U;
  }
}
