#ifndef __MPU6500_H
#define __MPU6500_H

#include "main.h"

/* Ground-vehicle profile used by MPU6500_Init(): +/-500 dps and +/-4 g.
 * These ranges retain comfortable headroom above the 200 dps gyro mode while
 * providing four times the resolution of the previous +/-2000 dps, +/-16 g
 * configuration. */
#define MPU6500_GYRO_LSB_PER_DPS  65.5f
#define MPU6500_ACCEL_LSB_PER_G    8192.0f
#define MPU6500_ACCEL_1G_LSB       8192

void MPU6500_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t MPU6500_ReadReg(uint8_t RegAddress);
uint8_t MPU6500_Init(void);
HAL_StatusTypeDef MPU6500_GetData(int16_t *Accx, int16_t *Accy, int16_t *Accz,
                                  int16_t *Gyrox, int16_t *Gyroy, int16_t *Gyroz);
HAL_StatusTypeDef MPU6500_GetDataEx(int16_t *Accx, int16_t *Accy, int16_t *Accz,
                                    int16_t *Temperature,
                                    int16_t *Gyrox, int16_t *Gyroy, int16_t *Gyroz);

#endif
