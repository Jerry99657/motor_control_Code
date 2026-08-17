#ifndef CAMERA_SERVICE_H
#define CAMERA_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define CAMERA_OV5640_SENSOR_ID          0x5640U
#define CAMERA_JPEG_WIDTH                320U
#define CAMERA_JPEG_HEIGHT               240U
#define CAMERA_JPEG_BUFFER_COUNT         2U
#define CAMERA_JPEG_BUFFER_CAPACITY      (64U * 1024U)
#define CAMERA_CAPTURE_TIMEOUT_DEFAULT   1000U

typedef enum
{
  CAMERA_STATE_OFF = 0,
  CAMERA_STATE_INITIALIZING,
  CAMERA_STATE_READY,
  CAMERA_STATE_CAPTURING,
  CAMERA_STATE_FRAME_READY,
  CAMERA_STATE_ERROR
} Camera_State;

typedef enum
{
  CAMERA_RESULT_OK = 0,
  CAMERA_RESULT_INVALID_ARGUMENT = -1,
  CAMERA_RESULT_BUSY = -2,
  CAMERA_RESULT_NOT_READY = -3,
  CAMERA_RESULT_I2C = -4,
  CAMERA_RESULT_BAD_SENSOR_ID = -5,
  CAMERA_RESULT_SENSOR_INIT = -6,
  CAMERA_RESULT_DCMI = -7,
  CAMERA_RESULT_TIMEOUT = -8,
  CAMERA_RESULT_BUFFER_FULL = -9,
  CAMERA_RESULT_INVALID_JPEG = -10
} Camera_Result;

typedef enum
{
  CAMERA_INIT_STAGE_NONE = 0,
  CAMERA_INIT_STAGE_I2C_RECOVERY,
  CAMERA_INIT_STAGE_I2C_PROBE,
  CAMERA_INIT_STAGE_READ_ID,
  CAMERA_INIT_STAGE_SENSOR_CONFIG,
  CAMERA_INIT_STAGE_CAPTURE
} Camera_InitStage;

typedef struct
{
  Camera_State state;
  Camera_Result last_result;
  Camera_InitStage init_stage;
  uint32_t sensor_id;
  uint32_t frame_size;
  uint32_t dma_bytes_received;
  uint32_t dcmi_error;
  uint32_t i2c_error;
  uint32_t i2c_state;
  uint8_t scl_level;
  uint8_t sda_level;
  uint8_t pwdn_level;
  uint8_t sccb_nack_phase;
  uint8_t autofocus_enabled;
  uint8_t autofocus_ready;
  uint8_t autofocus_status;
} Camera_Diagnostics;

/* Keep OV_PWDN enabled. OV_RESET is owned by the camera carrier because PC4
 * is assigned to the buzzer. Safe to call once GPIO is ready. */
void Camera_Service_BootHold(void);

/* Power up and configure OV5640 for 320x240 JPEG parallel output. */
Camera_Result Camera_Service_Init(void);

/* Start one non-blocking DCMI snapshot. Camera_Service_Process() finishes it. */
Camera_Result Camera_Service_StartSnapshot(uint32_t timeout_ms);
void Camera_Service_Process(void);

/* Convenience bring-up API: initialize and capture one JPEG. The returned
 * slot must be released with Camera_Service_ReleaseSnapshot(). */
Camera_Result Camera_Service_CaptureJpeg(const uint8_t **jpeg_data,
                                        uint32_t *jpeg_size,
                                        uint32_t timeout_ms);

/* The returned pointer remains valid until Camera_Service_ReleaseSnapshot()
 * or Camera_Service_Sleep(); capture may continue into the other slot. */
Camera_Result Camera_Service_GetSnapshot(const uint8_t **jpeg_data,
                                         uint32_t *jpeg_size);

/* Release a snapshot previously returned by Camera_Service_GetSnapshot().
 * DCMI will not overwrite that JPEG slot until it is released. */
void Camera_Service_ReleaseSnapshot(const uint8_t *jpeg_data);

/* Borrow one compressed-frame slot while the camera is OFF. This avoids a
 * second 64 KiB allocation for loading album JPEG files. */
Camera_Result Camera_Service_GetIdleJpegWorkspace(uint8_t **buffer,
                                                  uint32_t *capacity);

/* Stop DCMI and put OV5640 in PWDN through PF13. PC4 is buzzer-only. */
void Camera_Service_Sleep(void);

Camera_State Camera_Service_GetState(void);
Camera_Result Camera_Service_GetLastResult(void);
void Camera_Service_GetDiagnostics(Camera_Diagnostics *diagnostics);

#ifdef __cplusplus
}
#endif

#endif /* CAMERA_SERVICE_H */
