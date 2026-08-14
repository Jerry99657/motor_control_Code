#ifndef IMU_RECORDER_H
#define IMU_RECORDER_H

#include "imu_service.h"

#include <stdint.h>

typedef enum
{
    IMU_RECORDER_IDLE = 0,
    IMU_RECORDER_RECORDING,
    IMU_RECORDER_ERROR
} ImuRecorderState;

typedef struct
{
    ImuRecorderState state;
    uint32_t record_count;
    uint32_t bytes_written;
    uint32_t dropped_count;
    uint32_t start_tick;
    uint8_t fs_error;
    char filename[32];
} ImuRecorderStatus;

void IMU_Recorder_Init(void);
int8_t IMU_Recorder_Start(void);
int8_t IMU_Recorder_Stop(void);
void IMU_Recorder_Push(const ImuServiceSnapshot_t *snapshot);
void IMU_Recorder_GetStatus(ImuRecorderStatus *status);
uint8_t IMU_Recorder_IsRecording(void);

#endif /* IMU_RECORDER_H */
