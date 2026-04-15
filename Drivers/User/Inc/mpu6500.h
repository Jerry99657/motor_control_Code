#ifndef __MPU6500_H
#define __MPU6500_H

#include "main.h"

void MPU6500_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t MPU6500_ReadReg(uint8_t RegAddress);
uint8_t MPU6500_Init(void);
void MPU6500_GetData(int16_t *Accx, int16_t *Accy, int16_t *Accz, int16_t *Gyrox, int16_t *Gyroy, int16_t *Gyroz);

#endif
