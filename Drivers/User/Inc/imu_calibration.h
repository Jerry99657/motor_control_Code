#ifndef IMU_CALIBRATION_H
#define IMU_CALIBRATION_H

#include "imu.h"

#include <stdint.h>

typedef enum
{
    IMU_CAL_STATE_IDLE = 0,
    IMU_CAL_STATE_SETTLING,
    IMU_CAL_STATE_COLLECTING,
    IMU_CAL_STATE_SUCCESS,
    IMU_CAL_STATE_FAILED_MOTION,
    IMU_CAL_STATE_FAILED_QUALITY,
    IMU_CAL_STATE_FAILED_STORAGE,
    IMU_CAL_STATE_CANCELLED
} ImuCalibrationState;

typedef struct
{
    ImuCalibrationState state;
    imu_calibration_t calibration;
    float gyro_std_dps[3];
    float accel_std_g[3];
    float accel_norm_g;
    float temperature_c;
    uint32_t sequence;
    uint16_t sample_count;
    uint16_t target_samples;
    uint16_t restart_count;
    uint8_t progress_percent;
    uint8_t storage_available;
    uint8_t persistent_valid;
    uint8_t runtime_calibrated;
} ImuCalibrationStatus;

void IMU_Calibration_Init(uint8_t qspi_ready);
int8_t IMU_Calibration_Start(void);
void IMU_Calibration_Cancel(void);
void IMU_Calibration_Process(uint32_t now);
void IMU_Calibration_ProcessRaw(int16_t ax, int16_t ay, int16_t az,
                                int16_t gx, int16_t gy, int16_t gz,
                                int16_t temperature_raw,
                                uint32_t sample_tick);
void IMU_Calibration_GetStatus(ImuCalibrationStatus *status);
uint8_t IMU_Calibration_IsActive(void);
uint8_t IMU_Calibration_HasStableRuntimeBias(void);

#endif /* IMU_CALIBRATION_H */
