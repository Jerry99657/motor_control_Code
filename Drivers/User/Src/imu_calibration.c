#include "imu_calibration.h"

#include "mpu6500.h"
#include "qspi_partition.h"
#include "qspi_w25q64.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define IMU_CAL_MAGIC                     0x43554D49U /* "IMUC" little endian */
#define IMU_CAL_FORMAT_VERSION            1U
#define IMU_CAL_RECORD_SIZE               128U
#define IMU_CAL_CRC_OFFSET                124U
#define IMU_CAL_SETTLE_MS                 800U
#define IMU_CAL_TIMEOUT_MS                30000U
#define IMU_CAL_TARGET_SAMPLES            600U
#define IMU_CAL_GYRO_MOTION_DPS           1.5f
#define IMU_CAL_ACCEL_MIN_G               0.85f
#define IMU_CAL_ACCEL_MAX_G               1.15f
#define IMU_CAL_MAX_GYRO_STD_DPS          0.50f
#define IMU_CAL_MAX_ACCEL_STD_G           0.025f

typedef struct
{
    double mean[7];
    double m2[7];
    uint32_t count;
} ImuCalibrationAccumulator;

static ImuCalibrationStatus s_status;
static ImuCalibrationAccumulator s_accumulator;
static uint32_t s_start_tick;
static uint32_t s_settle_tick;
static uint8_t s_active_slot = 0xFFU;
static int8_t s_mount_z_sign = 1;

static uint16_t imu_cal_read_u16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static uint32_t imu_cal_read_u32(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

static float imu_cal_read_float(const uint8_t *data)
{
    float value;
    memcpy(&value, data, sizeof(value));
    return value;
}

static void imu_cal_write_u16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
}

static void imu_cal_write_u32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
    data[2] = (uint8_t)(value >> 16);
    data[3] = (uint8_t)(value >> 24);
}

static void imu_cal_write_float(uint8_t *data, float value)
{
    memcpy(data, &value, sizeof(value));
}

static uint32_t imu_cal_crc32(const uint8_t *data, uint32_t size)
{
    uint32_t crc = 0xFFFFFFFFU;
    uint32_t i;
    uint8_t bit;

    for (i = 0U; i < size; ++i)
    {
        crc ^= data[i];
        for (bit = 0U; bit < 8U; ++bit)
        {
            crc = (crc >> 1) ^ ((0U - (crc & 1U)) & 0xEDB88320U);
        }
    }
    return crc ^ 0xFFFFFFFFU;
}

static uint8_t imu_cal_record_valid(const uint8_t *record)
{
    uint8_t axis;

    if ((imu_cal_read_u32(&record[0]) != IMU_CAL_MAGIC) ||
        (imu_cal_read_u16(&record[4]) != IMU_CAL_FORMAT_VERSION) ||
        (imu_cal_read_u16(&record[6]) != IMU_CAL_RECORD_SIZE) ||
        (imu_cal_crc32(record, IMU_CAL_CRC_OFFSET) !=
         imu_cal_read_u32(&record[IMU_CAL_CRC_OFFSET])))
    {
        return 0U;
    }

    for (axis = 0U; axis < 3U; ++axis)
    {
        if ((!isfinite(imu_cal_read_float(&record[24U + axis * 4U]))) ||
            (!isfinite(imu_cal_read_float(&record[36U + axis * 4U]))) ||
            (!isfinite(imu_cal_read_float(&record[48U + axis * 4U]))) ||
            (!isfinite(imu_cal_read_float(&record[60U + axis * 4U]))))
        {
            return 0U;
        }
    }
    return 1U;
}

static void imu_cal_decode_record(const uint8_t *record)
{
    uint8_t axis;

    s_status.sequence = imu_cal_read_u32(&record[8]);
    s_status.sample_count = imu_cal_read_u16(&record[12]);
    s_status.restart_count = imu_cal_read_u16(&record[14]);
    for (axis = 0U; axis < 3U; ++axis)
    {
        s_status.calibration.gyro_bias[axis] =
            imu_cal_read_float(&record[24U + axis * 4U]);
        s_status.calibration.accel_offset[axis] =
            imu_cal_read_float(&record[36U + axis * 4U]);
        s_status.gyro_std_dps[axis] =
            imu_cal_read_float(&record[48U + axis * 4U]);
        s_status.accel_std_g[axis] =
            imu_cal_read_float(&record[60U + axis * 4U]);
    }
    s_status.temperature_c = imu_cal_read_float(&record[72]);
    s_status.accel_norm_g = imu_cal_read_float(&record[76]);
}

static void imu_cal_build_record(uint8_t *record, uint32_t sequence)
{
    uint8_t axis;

    memset(record, 0xFF, IMU_CAL_RECORD_SIZE);
    imu_cal_write_u32(&record[0], IMU_CAL_MAGIC);
    imu_cal_write_u16(&record[4], IMU_CAL_FORMAT_VERSION);
    imu_cal_write_u16(&record[6], IMU_CAL_RECORD_SIZE);
    imu_cal_write_u32(&record[8], sequence);
    imu_cal_write_u16(&record[12], s_status.sample_count);
    imu_cal_write_u16(&record[14], s_status.restart_count);
    imu_cal_write_u32(&record[16], HAL_GetTick());
    for (axis = 0U; axis < 3U; ++axis)
    {
        imu_cal_write_float(&record[24U + axis * 4U],
                            s_status.calibration.gyro_bias[axis]);
        imu_cal_write_float(&record[36U + axis * 4U],
                            s_status.calibration.accel_offset[axis]);
        imu_cal_write_float(&record[48U + axis * 4U],
                            s_status.gyro_std_dps[axis]);
        imu_cal_write_float(&record[60U + axis * 4U],
                            s_status.accel_std_g[axis]);
    }
    imu_cal_write_float(&record[72], s_status.temperature_c);
    imu_cal_write_float(&record[76], s_status.accel_norm_g);
    imu_cal_write_u32(&record[IMU_CAL_CRC_OFFSET],
                      imu_cal_crc32(record, IMU_CAL_CRC_OFFSET));
}

static uint8_t imu_cal_save(void)
{
    uint8_t record[IMU_CAL_RECORD_SIZE];
    uint8_t verify[IMU_CAL_RECORD_SIZE];
    uint8_t was_mapped;
    uint8_t target_slot;
    uint32_t target_address;
    uint32_t next_sequence;
    uint8_t saved = 0U;

    if (s_status.storage_available == 0U)
    {
        return 0U;
    }

    target_slot = (s_active_slot == 0U) ? 1U : 0U;
    target_address = (target_slot == 0U) ?
        QSPI_PARTITION_IMU_CAL_SLOT_A : QSPI_PARTITION_IMU_CAL_SLOT_B;
    next_sequence = s_status.sequence + 1U;
    imu_cal_build_record(record, next_sequence);
    was_mapped = QSPI_W25Qxx_IsMemoryMapped();

    if ((was_mapped == 0U) ||
        (QSPI_W25Qxx_ExitMemoryMappedMode() == QSPI_W25QXX_OK))
    {
        if ((QSPI_W25Qxx_SectorErase(target_address) == QSPI_W25QXX_OK) &&
            (QSPI_W25Qxx_WriteBuffer(record, target_address,
                                     sizeof(record)) == QSPI_W25QXX_OK) &&
            (QSPI_W25Qxx_ReadBuffer(verify, target_address,
                                    sizeof(verify)) == QSPI_W25QXX_OK) &&
            (memcmp(record, verify, sizeof(record)) == 0) &&
            (imu_cal_record_valid(verify) != 0U))
        {
            s_active_slot = target_slot;
            s_status.sequence = next_sequence;
            s_status.persistent_valid = 1U;
            saved = 1U;
        }
    }

    if ((was_mapped != 0U) &&
        (QSPI_W25Qxx_MemoryMappedMode() != QSPI_W25QXX_OK))
    {
        saved = 0U;
    }
    else if (was_mapped != 0U)
    {
        SCB_InvalidateDCache_by_Addr(
            (uint32_t *)(QSPI_BASE + QSPI_PARTITION_IMU_CAL_OFFSET),
            (int32_t)QSPI_PARTITION_IMU_CAL_SIZE);
    }
    return saved;
}

static void imu_cal_accumulator_reset(void)
{
    memset(&s_accumulator, 0, sizeof(s_accumulator));
    s_status.sample_count = 0U;
    s_status.progress_percent = 0U;
}

static void imu_cal_collection_restart(uint32_t sample_tick)
{
    uint16_t restart_count = s_status.restart_count;

    if (restart_count < UINT16_MAX)
    {
        restart_count++;
    }
    imu_cal_accumulator_reset();
    s_status.restart_count = restart_count;
    s_settle_tick = sample_tick;
    s_status.state = IMU_CAL_STATE_SETTLING;
}

static void imu_cal_accumulator_push(const double values[7])
{
    uint8_t i;

    s_accumulator.count++;
    for (i = 0U; i < 7U; ++i)
    {
        double delta = values[i] - s_accumulator.mean[i];
        s_accumulator.mean[i] += delta / (double)s_accumulator.count;
        s_accumulator.m2[i] += delta *
            (values[i] - s_accumulator.mean[i]);
    }
    s_status.sample_count = (uint16_t)s_accumulator.count;
    s_status.progress_percent = (uint8_t)
        ((s_accumulator.count * 100U) / IMU_CAL_TARGET_SAMPLES);
}

static uint8_t imu_cal_sample_is_still(const double values[7])
{
    double accel_norm = sqrt((values[0] * values[0]) +
                             (values[1] * values[1]) +
                             (values[2] * values[2]));
    double gyro_limit = MPU6500_GYRO_LSB_PER_DPS *
                        IMU_CAL_GYRO_MOTION_DPS;

    return ((accel_norm >= MPU6500_ACCEL_LSB_PER_G * IMU_CAL_ACCEL_MIN_G) &&
            (accel_norm <= MPU6500_ACCEL_LSB_PER_G * IMU_CAL_ACCEL_MAX_G) &&
            (fabs(values[3] - s_status.calibration.gyro_bias[0]) <=
             gyro_limit) &&
            (fabs(values[4] - s_status.calibration.gyro_bias[1]) <=
             gyro_limit) &&
            (fabs(values[5] - s_status.calibration.gyro_bias[2]) <=
             gyro_limit)) ? 1U : 0U;
}

static void imu_cal_finish(void)
{
    uint8_t axis;
    uint8_t quality_ok = 1U;

    if (s_accumulator.count < IMU_CAL_TARGET_SAMPLES)
    {
        return;
    }

    for (axis = 0U; axis < 3U; ++axis)
    {
        double gyro_variance = s_accumulator.m2[axis + 3U] /
                               (double)(s_accumulator.count - 1U);
        double accel_variance = s_accumulator.m2[axis] /
                                (double)(s_accumulator.count - 1U);
        s_status.calibration.gyro_bias[axis] =
            (float)s_accumulator.mean[axis + 3U];
        s_status.gyro_std_dps[axis] =
            (float)(sqrt(gyro_variance) / MPU6500_GYRO_LSB_PER_DPS);
        s_status.accel_std_g[axis] =
            (float)(sqrt(accel_variance) / MPU6500_ACCEL_LSB_PER_G);
        if ((s_status.gyro_std_dps[axis] > IMU_CAL_MAX_GYRO_STD_DPS) ||
            (s_status.accel_std_g[axis] > IMU_CAL_MAX_ACCEL_STD_G))
        {
            quality_ok = 0U;
        }
    }

    s_status.calibration.accel_offset[0] = (float)s_accumulator.mean[0];
    s_status.calibration.accel_offset[1] = (float)s_accumulator.mean[1];
    s_status.calibration.accel_offset[2] = (float)s_accumulator.mean[2] -
        ((float)s_mount_z_sign * MPU6500_ACCEL_LSB_PER_G);
    s_status.temperature_c = (float)(s_accumulator.mean[6] / 333.87 + 21.0);
    s_status.accel_norm_g = (float)(sqrt(
        (s_accumulator.mean[0] * s_accumulator.mean[0]) +
        (s_accumulator.mean[1] * s_accumulator.mean[1]) +
        (s_accumulator.mean[2] * s_accumulator.mean[2])) /
        MPU6500_ACCEL_LSB_PER_G);

    if (quality_ok == 0U)
    {
        s_status.state = IMU_CAL_STATE_FAILED_QUALITY;
        return;
    }

    imu_set_calibration(&s_status.calibration);
    imu_reset_attitude();
    s_status.runtime_calibrated = 1U;
    s_status.progress_percent = 100U;
    s_status.state = (imu_cal_save() != 0U) ?
        IMU_CAL_STATE_SUCCESS : IMU_CAL_STATE_FAILED_STORAGE;
}

void IMU_Calibration_Init(uint8_t qspi_ready)
{
    uint8_t record_a[IMU_CAL_RECORD_SIZE];
    uint8_t record_b[IMU_CAL_RECORD_SIZE];
    uint8_t valid_a = 0U;
    uint8_t valid_b = 0U;
    uint8_t was_mapped;
    const uint8_t *selected = NULL;

    memset(&s_status, 0, sizeof(s_status));
    s_status.state = IMU_CAL_STATE_IDLE;
    s_status.target_samples = IMU_CAL_TARGET_SAMPLES;
    s_status.storage_available = (qspi_ready != 0U) ? 1U : 0U;
    imu_get_calibration(&s_status.calibration);
    imu_cal_accumulator_reset();
    s_active_slot = 0xFFU;
    s_mount_z_sign = 1;

    if (s_status.storage_available == 0U)
    {
        return;
    }

    was_mapped = QSPI_W25Qxx_IsMemoryMapped();
    if ((was_mapped != 0U) &&
        (QSPI_W25Qxx_ExitMemoryMappedMode() != QSPI_W25QXX_OK))
    {
        s_status.storage_available = 0U;
        return;
    }

    valid_a = ((QSPI_W25Qxx_ReadBuffer(record_a,
                QSPI_PARTITION_IMU_CAL_SLOT_A, sizeof(record_a)) ==
                QSPI_W25QXX_OK) && (imu_cal_record_valid(record_a) != 0U));
    valid_b = ((QSPI_W25Qxx_ReadBuffer(record_b,
                QSPI_PARTITION_IMU_CAL_SLOT_B, sizeof(record_b)) ==
                QSPI_W25QXX_OK) && (imu_cal_record_valid(record_b) != 0U));

    if ((valid_a != 0U) && (valid_b != 0U))
    {
        if ((int32_t)(imu_cal_read_u32(&record_b[8]) -
                      imu_cal_read_u32(&record_a[8])) > 0)
        {
            selected = record_b;
            s_active_slot = 1U;
        }
        else
        {
            selected = record_a;
            s_active_slot = 0U;
        }
    }
    else if (valid_a != 0U)
    {
        selected = record_a;
        s_active_slot = 0U;
    }
    else if (valid_b != 0U)
    {
        selected = record_b;
        s_active_slot = 1U;
    }

    if (selected != NULL)
    {
        imu_cal_decode_record(selected);
        imu_set_calibration(&s_status.calibration);
        imu_reset_attitude();
        s_status.persistent_valid = 1U;
        s_status.runtime_calibrated = 1U;
    }

    if ((was_mapped != 0U) &&
        (QSPI_W25Qxx_MemoryMappedMode() != QSPI_W25QXX_OK))
    {
        s_status.storage_available = 0U;
    }
}

int8_t IMU_Calibration_Start(void)
{
    if (IMU_Calibration_IsActive() != 0U)
    {
        return -1;
    }

    imu_get_calibration(&s_status.calibration);
    imu_cal_accumulator_reset();
    s_status.restart_count = 0U;
    s_status.state = IMU_CAL_STATE_SETTLING;
    s_start_tick = HAL_GetTick();
    s_settle_tick = s_start_tick;
    return 0;
}

void IMU_Calibration_Cancel(void)
{
    if (IMU_Calibration_IsActive() != 0U)
    {
        s_status.state = IMU_CAL_STATE_CANCELLED;
    }
}

void IMU_Calibration_Process(uint32_t now)
{
    if ((IMU_Calibration_IsActive() != 0U) &&
        ((uint32_t)(now - s_start_tick) >= IMU_CAL_TIMEOUT_MS))
    {
        s_status.state = IMU_CAL_STATE_FAILED_MOTION;
    }
}

void IMU_Calibration_ProcessRaw(int16_t ax, int16_t ay, int16_t az,
                                int16_t gx, int16_t gy, int16_t gz,
                                int16_t temperature_raw,
                                uint32_t sample_tick)
{
    double values[7] = {ax, ay, az, gx, gy, gz, temperature_raw};

    if (abs((int)az) >= (MPU6500_ACCEL_1G_LSB / 2))
    {
        s_mount_z_sign = (az >= 0) ? 1 : -1;
    }

    if (IMU_Calibration_IsActive() == 0U)
    {
        return;
    }

    IMU_Calibration_Process(sample_tick);
    if (IMU_Calibration_IsActive() == 0U) return;

    if (imu_cal_sample_is_still(values) == 0U)
    {
        imu_cal_collection_restart(sample_tick);
        return;
    }

    if ((uint32_t)(sample_tick - s_settle_tick) < IMU_CAL_SETTLE_MS)
    {
        s_status.state = IMU_CAL_STATE_SETTLING;
        return;
    }

    s_status.state = IMU_CAL_STATE_COLLECTING;
    imu_cal_accumulator_push(values);
    if (s_accumulator.count >= IMU_CAL_TARGET_SAMPLES)
    {
        imu_cal_finish();
    }
}

void IMU_Calibration_GetStatus(ImuCalibrationStatus *status)
{
    if (status != NULL)
    {
        *status = s_status;
    }
}

uint8_t IMU_Calibration_IsActive(void)
{
    return ((s_status.state == IMU_CAL_STATE_SETTLING) ||
            (s_status.state == IMU_CAL_STATE_COLLECTING)) ? 1U : 0U;
}

uint8_t IMU_Calibration_HasStableRuntimeBias(void)
{
    return s_status.runtime_calibrated;
}
