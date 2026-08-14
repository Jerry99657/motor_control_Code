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
    MPU6500_WriteReg(MPU6500_SMPLRT_DIV, 0x09);  // 1 kHz / 10 = 100 Hz
    MPU6500_WriteReg(MPU6500_CONFIG, 0x03);      // Gyro DLPF about 41 Hz
    MPU6500_WriteReg(MPU6500_GYRO_CONFIG, 0x08); // +/-500 dps, 65.5 LSB/dps
    MPU6500_WriteReg(MPU6500_ACCEL_CONFIG, 0x08);// +/-4 g, 8192 LSB/g
    MPU6500_WriteReg(MPU6500_ACCEL_CONFIG_2, 0x03); // Accel DLPF about 44.8 Hz
    MPU6500_WriteReg(0x37, 0x02); // INT_PIN_CFG: BYPASS_EN=1 (often needed to stabilize I2C access on ICM20608)
    return who_am_i; // Return the ID
}

HAL_StatusTypeDef MPU6500_GetData(int16_t *Accx, int16_t *Accy, int16_t *Accz,
                                  int16_t *Gyrox, int16_t *Gyroy, int16_t *Gyroz) {
    int16_t temperature;

    return MPU6500_GetDataEx(Accx, Accy, Accz, &temperature,
                             Gyrox, Gyroy, Gyroz);
}

HAL_StatusTypeDef MPU6500_GetDataEx(int16_t *Accx, int16_t *Accy, int16_t *Accz,
                                    int16_t *Temperature,
                                    int16_t *Gyrox, int16_t *Gyroy, int16_t *Gyroz) {
    uint8_t buf[14];
    HAL_StatusTypeDef status;

    if ((Accx == NULL) || (Accy == NULL) || (Accz == NULL) ||
        (Temperature == NULL) ||
        (Gyrox == NULL) || (Gyroy == NULL) || (Gyroz == NULL)) {
        return HAL_ERROR;
    }

    /* Accelerometer, temperature and gyroscope registers are contiguous. */
    status = HAL_I2C_Mem_Read(&hi2c1, MPU_Addr, MPU6500_ACCEL_XOUT_H,
                              I2C_MEMADD_SIZE_8BIT, buf, sizeof(buf), 5U);
    if (status != HAL_OK) {
        return status;
    }

    *Accx  = (int16_t)((buf[0]  << 8) | buf[1]);
    *Accy  = (int16_t)((buf[2]  << 8) | buf[3]);
    *Accz  = (int16_t)((buf[4]  << 8) | buf[5]);
    *Temperature = (int16_t)((buf[6] << 8) | buf[7]);
    *Gyrox = (int16_t)((buf[8]  << 8) | buf[9]);
    *Gyroy = (int16_t)((buf[10] << 8) | buf[11]);
    *Gyroz = (int16_t)((buf[12] << 8) | buf[13]);
    return HAL_OK;
}
