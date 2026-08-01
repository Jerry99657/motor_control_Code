#ifndef IMU_SERVICE_H
#define IMU_SERVICE_H

#include "imu.h"
#include <stdint.h>

typedef struct
{
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
    eulerian_angles_t angles;
    uint32_t sample_tick;
    uint32_t sequence;
} ImuServiceSnapshot_t;

void IMU_Service_Init(void);
void IMU_Service_Process(void);
uint8_t IMU_Service_GetSnapshot(ImuServiceSnapshot_t *snapshot);
uint32_t IMU_Service_GetFailureCount(void);

#endif /* IMU_SERVICE_H */
