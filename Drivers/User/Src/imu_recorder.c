#include "imu_recorder.h"

#include "dc_motor_ol.h"
#include "fatfs.h"
#include "ff.h"
#include "imu_calibration.h"
#include "mecanum_odometry.h"

#include <limits.h>
#include <stdio.h>
#include <string.h>

#define IMU_LOG_MAGIC                0x4C554D49U /* "IMUL" */
#define IMU_LOG_VERSION              1U
#define IMU_LOG_HEADER_SIZE          128U
#define IMU_LOG_BUFFER_SIZE          2016U
#define IMU_LOG_DIRECTORY            "0:/IMU_LOG"
#define IMU_LOG_MAX_FILE_NUMBER      9999U

typedef struct
{
    uint32_t tick_ms;
    uint32_t sequence;
    int16_t raw_accel[3];
    int16_t raw_gyro[3];
    int16_t accel[3];
    int16_t gyro[3];
    int16_t temperature_raw;
    uint16_t flags;
    float pitch_deg;
    float roll_deg;
    float yaw_deg;
    float yaw_rate_dps;
    float odom_x_mm;
    float odom_y_mm;
    float odom_heading_deg;
    int32_t wheel_position[4];
    uint32_t imu_failure_count;
} ImuLogRecord;

_Static_assert(sizeof(ImuLogRecord) == 84U, "Unexpected IMU log record size");

static ImuRecorderStatus s_status;
static FIL s_log_file;
static uint8_t s_buffer[IMU_LOG_BUFFER_SIZE];
static uint16_t s_buffer_used;
static uint8_t s_file_open;
static uint32_t s_last_sync_tick;

static void imu_recorder_write_u16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
}

static void imu_recorder_write_u32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
    data[2] = (uint8_t)(value >> 16);
    data[3] = (uint8_t)(value >> 24);
}

static void imu_recorder_write_float(uint8_t *data, float value)
{
    memcpy(data, &value, sizeof(value));
}

static int32_t imu_recorder_clamp_i64(int64_t value)
{
    if (value > INT32_MAX) return INT32_MAX;
    if (value < INT32_MIN) return INT32_MIN;
    return (int32_t)value;
}

static int8_t imu_recorder_flush(void)
{
    UINT written = 0U;
    FRESULT fr;

    if ((s_file_open == 0U) || (s_buffer_used == 0U))
    {
        return 0;
    }

    fr = f_write(&s_log_file, s_buffer, s_buffer_used, &written);
    if ((fr != FR_OK) || (written != s_buffer_used))
    {
        s_status.fs_error = (uint8_t)fr;
        s_status.state = IMU_RECORDER_ERROR;
        s_status.dropped_count++;
        return -1;
    }

    s_status.bytes_written += written;
    s_buffer_used = 0U;
    if ((uint32_t)(HAL_GetTick() - s_last_sync_tick) >= 5000U)
    {
        fr = f_sync(&s_log_file);
        if (fr != FR_OK)
        {
            s_status.fs_error = (uint8_t)fr;
            s_status.state = IMU_RECORDER_ERROR;
            return -1;
        }
        s_last_sync_tick = HAL_GetTick();
    }
    return 0;
}

static void imu_recorder_close(void)
{
    if (s_file_open != 0U)
    {
        (void)f_sync(&s_log_file);
        (void)f_close(&s_log_file);
        s_file_open = 0U;
    }
}

static void imu_recorder_build_header(uint8_t header[IMU_LOG_HEADER_SIZE])
{
    ImuCalibrationStatus calibration;
    imu_calibration_t runtime_calibration;
    uint8_t axis;

    memset(header, 0, IMU_LOG_HEADER_SIZE);
    IMU_Calibration_GetStatus(&calibration);
    imu_get_calibration(&runtime_calibration);
    imu_recorder_write_u32(&header[0], IMU_LOG_MAGIC);
    imu_recorder_write_u16(&header[4], IMU_LOG_VERSION);
    imu_recorder_write_u16(&header[6], IMU_LOG_HEADER_SIZE);
    imu_recorder_write_u16(&header[8], sizeof(ImuLogRecord));
    imu_recorder_write_u16(&header[10], 10U);
    imu_recorder_write_u32(&header[12], HAL_GetTick());
    imu_recorder_write_u32(&header[16], calibration.sequence);
    for (axis = 0U; axis < 3U; ++axis)
    {
        imu_recorder_write_float(&header[20U + axis * 4U],
            runtime_calibration.gyro_bias[axis]);
        imu_recorder_write_float(&header[32U + axis * 4U],
            runtime_calibration.accel_offset[axis]);
        imu_recorder_write_float(&header[44U + axis * 4U],
            calibration.gyro_std_dps[axis]);
        imu_recorder_write_float(&header[56U + axis * 4U],
            calibration.accel_std_g[axis]);
    }
    imu_recorder_write_float(&header[68], calibration.temperature_c);
    header[72] = calibration.persistent_valid;
}

void IMU_Recorder_Init(void)
{
    memset(&s_status, 0, sizeof(s_status));
    s_status.state = IMU_RECORDER_IDLE;
    s_buffer_used = 0U;
    s_file_open = 0U;
    s_last_sync_tick = 0U;
}

int8_t IMU_Recorder_Start(void)
{
    uint8_t header[IMU_LOG_HEADER_SIZE];
    UINT written = 0U;
    FRESULT fr;
    FILINFO info;
    uint16_t file_number;

    if (s_status.state == IMU_RECORDER_RECORDING)
    {
        return -1;
    }

    s_status.state = IMU_RECORDER_IDLE;
    s_status.record_count = 0U;
    s_status.bytes_written = 0U;
    s_status.dropped_count = 0U;
    s_status.start_tick = 0U;
    s_status.fs_error = 0U;
    s_status.filename[0] = '\0';
    s_buffer_used = 0U;
    s_file_open = 0U;
    s_last_sync_tick = 0U;
    fr = f_mount(&SDFatFS, (TCHAR const *)SDPath, 1U);
    if (fr != FR_OK)
    {
        s_status.state = IMU_RECORDER_ERROR;
        s_status.fs_error = (uint8_t)fr;
        return -2;
    }

    fr = f_mkdir(IMU_LOG_DIRECTORY);
    if ((fr != FR_OK) && (fr != FR_EXIST))
    {
        s_status.state = IMU_RECORDER_ERROR;
        s_status.fs_error = (uint8_t)fr;
        imu_recorder_close();
        return -3;
    }

    for (file_number = 1U; file_number <= IMU_LOG_MAX_FILE_NUMBER;
         ++file_number)
    {
        (void)snprintf(s_status.filename, sizeof(s_status.filename),
                       IMU_LOG_DIRECTORY "/IMU%04u.IMU",
                       (unsigned int)file_number);
        fr = f_stat(s_status.filename, &info);
        if (fr == FR_NO_FILE)
        {
            break;
        }
        if (fr != FR_OK)
        {
            s_status.state = IMU_RECORDER_ERROR;
            s_status.fs_error = (uint8_t)fr;
            imu_recorder_close();
            return -4;
        }
    }
    if (file_number > IMU_LOG_MAX_FILE_NUMBER)
    {
        s_status.state = IMU_RECORDER_ERROR;
        s_status.fs_error = (uint8_t)FR_EXIST;
        imu_recorder_close();
        return -5;
    }

    fr = f_open(&s_log_file, s_status.filename,
                FA_CREATE_NEW | FA_WRITE);
    if (fr != FR_OK)
    {
        s_status.state = IMU_RECORDER_ERROR;
        s_status.fs_error = (uint8_t)fr;
        imu_recorder_close();
        return -6;
    }
    s_file_open = 1U;

    imu_recorder_build_header(header);
    fr = f_write(&s_log_file, header, sizeof(header), &written);
    if ((fr != FR_OK) || (written != sizeof(header)))
    {
        s_status.state = IMU_RECORDER_ERROR;
        s_status.fs_error = (uint8_t)fr;
        imu_recorder_close();
        return -7;
    }

    s_status.bytes_written = written;
    s_status.start_tick = HAL_GetTick();
    s_last_sync_tick = s_status.start_tick;
    s_status.state = IMU_RECORDER_RECORDING;
    return 0;
}

int8_t IMU_Recorder_Stop(void)
{
    int8_t result = 0;

    if (s_status.state == IMU_RECORDER_RECORDING)
    {
        result = imu_recorder_flush();
    }
    imu_recorder_close();
    if (s_status.state != IMU_RECORDER_ERROR)
    {
        s_status.state = IMU_RECORDER_IDLE;
    }
    return result;
}

void IMU_Recorder_Push(const ImuServiceSnapshot_t *snapshot)
{
    ImuLogRecord record;
    MecanumOdometrySnapshot odometry;
    uint8_t motor;

    if ((snapshot == NULL) || (s_status.state != IMU_RECORDER_RECORDING))
    {
        return;
    }

    if ((uint32_t)s_buffer_used + sizeof(record) > sizeof(s_buffer))
    {
        if (imu_recorder_flush() != 0)
        {
            imu_recorder_close();
            return;
        }
    }

    memset(&record, 0, sizeof(record));
    record.tick_ms = snapshot->sample_tick;
    record.sequence = snapshot->sequence;
    record.raw_accel[0] = snapshot->raw_ax;
    record.raw_accel[1] = snapshot->raw_ay;
    record.raw_accel[2] = snapshot->raw_az;
    record.raw_gyro[0] = snapshot->raw_gx;
    record.raw_gyro[1] = snapshot->raw_gy;
    record.raw_gyro[2] = snapshot->raw_gz;
    record.accel[0] = snapshot->ax;
    record.accel[1] = snapshot->ay;
    record.accel[2] = snapshot->az;
    record.gyro[0] = snapshot->gx;
    record.gyro[1] = snapshot->gy;
    record.gyro[2] = snapshot->gz;
    record.temperature_raw = snapshot->temperature_raw;
    record.flags = (IMU_Calibration_HasStableRuntimeBias() != 0U) ? 1U : 0U;
    record.pitch_deg = snapshot->angles.pitch;
    record.roll_deg = snapshot->angles.roll;
    record.yaw_deg = snapshot->angles.yaw;
    record.yaw_rate_dps = snapshot->planar_yaw_rate_dps;
    MecanumOdometry_GetSnapshot(&odometry);
    record.odom_x_mm = odometry.x_mm;
    record.odom_y_mm = odometry.y_mm;
    record.odom_heading_deg = odometry.heading_deg;
    for (motor = 0U; motor < 4U; ++motor)
    {
        record.wheel_position[motor] = imu_recorder_clamp_i64(
            DCMotor_OL_GetPositionPulses((uint8_t)(motor + 1U)));
    }
    record.imu_failure_count = IMU_Service_GetFailureCount();

    memcpy(&s_buffer[s_buffer_used], &record, sizeof(record));
    s_buffer_used = (uint16_t)(s_buffer_used + sizeof(record));
    s_status.record_count++;
}

void IMU_Recorder_GetStatus(ImuRecorderStatus *status)
{
    if (status != NULL)
    {
        *status = s_status;
    }
}

uint8_t IMU_Recorder_IsRecording(void)
{
    return (s_status.state == IMU_RECORDER_RECORDING) ? 1U : 0U;
}
