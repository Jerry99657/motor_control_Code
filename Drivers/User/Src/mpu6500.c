#include "mpu6500.h"
#include "mpu6500_reg.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c1;
static uint16_t MPU_Addr = 0xD0;

void MPU6500_WriteReg(uint8_t RegAddress, uint8_t Data) {
    HAL_I2C_Mem_Write(&hi2c1, MPU_Addr, RegAddress, I2C_MEMADD_SIZE_8BIT, &Data, 1, 100);
}

uint8_t MPU6500_ReadReg(uint8_t RegAddress) {
    uint8_t Data;
    HAL_I2C_Mem_Read(&hi2c1, MPU_Addr, RegAddress, I2C_MEMADD_SIZE_8BIT, &Data, 1, 100);
    return Data;
}

uint8_t MPU6500_Init(void) {
    if (HAL_I2C_IsDeviceReady(&hi2c1, 0xD0, 2, 100) == HAL_OK) {
        MPU_Addr = 0xD0;
    } else if (HAL_I2C_IsDeviceReady(&hi2c1, 0xD2, 2, 100) == HAL_OK) {
        MPU_Addr = 0xD2;
    } else {
        return 0; // Device not found
    }
    
    uint8_t who_am_i = MPU6500_ReadReg(MPU6500_WHO_AM_I);
    if (who_am_i != MPU6500_CHIP_ID && who_am_i != 0x70 && who_am_i != 0x71 && who_am_i != 0x68 && who_am_i != 0x12) {
        // Not the expected MPU6500 ID or common compatible IDs, but continue.
        // We will just let it continue but might be wrong ID
    }
    
    MPU6500_WriteReg(MPU6500_PWR_MGMT_1, 0x80); // Reset
    HAL_Delay(100);
    MPU6500_WriteReg(MPU6500_PWR_MGMT_1, 0x01); // Wakeup, clock=x gyro
    HAL_Delay(10);
    MPU6500_WriteReg(MPU6500_PWR_MGMT_2, 0x00);
    MPU6500_WriteReg(MPU6500_SMPLRT_DIV, 0x09); // 100Hz
    MPU6500_WriteReg(MPU6500_CONFIG, 0x06);     // LPF
    MPU6500_WriteReg(MPU6500_GYRO_CONFIG, 0x18);// 2000dps
    MPU6500_WriteReg(MPU6500_ACCEL_CONFIG, 0x18);// 16g
    MPU6500_WriteReg(0x37, 0x02); // INT_PIN_CFG: BYPASS_EN=1 (often needed to stabilize I2C access on ICM20608)
    return who_am_i; // Return the ID
}

void MPU6500_GetData(int16_t *Accx, int16_t *Accy, int16_t *Accz, int16_t *Gyrox, int16_t *Gyroy, int16_t *Gyroz) {
    uint8_t buf[14] = {0};
    
    // Split the burst read to avoid STM32H7 I2C Arbitration Lost (ARLO) on long reads due to slow rise time
    HAL_StatusTypeDef status1 = HAL_I2C_Mem_Read(&hi2c1, MPU_Addr, MPU6500_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, buf, 6, 100);
    if (status1 == HAL_OK) {
        HAL_I2C_Mem_Read(&hi2c1, MPU_Addr, MPU6500_TEMP_OUT_H,   I2C_MEMADD_SIZE_8BIT, buf+6, 2, 100);
        HAL_I2C_Mem_Read(&hi2c1, MPU_Addr, MPU6500_GYRO_XOUT_H,  I2C_MEMADD_SIZE_8BIT, buf+8, 6, 100);
    }
    
    // Fallback: If burst reading fails with ARLO error, read byte by byte
    if (status1 != HAL_OK || hi2c1.ErrorCode != HAL_I2C_ERROR_NONE) {
        for(int i = 0; i < 14; i++) {
            buf[i] = MPU6500_ReadReg(MPU6500_ACCEL_XOUT_H + i);
        }
    }

    *Accx  = (int16_t)((buf[0]  << 8) | buf[1]);
    *Accy  = (int16_t)((buf[2]  << 8) | buf[3]);
    *Accz  = (int16_t)((buf[4]  << 8) | buf[5]);
    *Gyrox = (int16_t)((buf[8]  << 8) | buf[9]);
    *Gyroy = (int16_t)((buf[10] << 8) | buf[11]);
    *Gyroz = (int16_t)((buf[12] << 8) | buf[13]);
}
