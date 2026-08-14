#include "imu_service.h"
#include "imu_calibration.h"
#include "imu_recorder.h"
#include "mpu6500.h"

#include <math.h>

#define IMU_SERVICE_PERIOD_MS 10U
#define IMU_SERVICE_MOUNT_Z_DETECT_MIN (MPU6500_ACCEL_1G_LSB / 2)
#define IMU_SERVICE_MIN_DT_SECONDS 0.005f
#define IMU_SERVICE_MAX_DT_SECONDS 0.030f
#define IMU_SERVICE_STATIONARY_ACCEL_ERROR_G 0.06f
#define IMU_SERVICE_STATIONARY_GYRO_DPS 0.60f
#define IMU_SERVICE_STATIONARY_SAMPLES 100U
#define IMU_SERVICE_BIAS_ADAPT_GAIN 0.0015f
#define IMU_SERVICE_RATE_DEADBAND_DPS 0.05f

static ImuServiceSnapshot_t s_snapshots[2];
static volatile uint8_t s_published_index = 0U;
static volatile uint8_t s_has_snapshot = 0U;
static uint32_t s_last_attempt_tick = 0U;
static uint32_t s_last_sample_tick = 0U;
static uint32_t s_sequence = 0U;
static uint32_t s_failure_count = 0U;
static float s_planar_yaw = 0.0f;
static float s_planar_yaw_sign = -1.0f;
static float s_previous_planar_rate_dps = 0.0f;
static uint16_t s_stationary_samples = 0U;

static float imu_service_wrap_degrees(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }
    while (angle < -180.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

static uint8_t imu_service_sample_is_stationary(
    const ImuServiceSnapshot_t *sample)
{
    float ax_g;
    float ay_g;
    float az_g;
    float accel_norm_g;
    float gyro_limit;

    if (sample == NULL)
    {
        return 0U;
    }

    ax_g = (float)sample->ax / MPU6500_ACCEL_LSB_PER_G;
    ay_g = (float)sample->ay / MPU6500_ACCEL_LSB_PER_G;
    az_g = (float)sample->az / MPU6500_ACCEL_LSB_PER_G;
    accel_norm_g = sqrtf((ax_g * ax_g) + (ay_g * ay_g) + (az_g * az_g));
    gyro_limit = MPU6500_GYRO_LSB_PER_DPS *
                 IMU_SERVICE_STATIONARY_GYRO_DPS;

    return ((fabsf(accel_norm_g - 1.0f) <=
             IMU_SERVICE_STATIONARY_ACCEL_ERROR_G) &&
            (fabsf((float)sample->gx) <= gyro_limit) &&
            (fabsf((float)sample->gy) <= gyro_limit) &&
            (fabsf((float)sample->gz) <= gyro_limit)) ? 1U : 0U;
}

void IMU_Service_Init(uint8_t qspi_ready)
{
    s_snapshots[0] = (ImuServiceSnapshot_t){0};
    s_snapshots[1] = (ImuServiceSnapshot_t){0};
    s_published_index = 0U;
    s_has_snapshot = 0U;
    s_last_attempt_tick = HAL_GetTick() - IMU_SERVICE_PERIOD_MS;
    s_last_sample_tick = 0U;
    s_sequence = 0U;
    s_failure_count = 0U;
    s_planar_yaw = 0.0f;
    s_planar_yaw_sign = -1.0f;
    s_previous_planar_rate_dps = 0.0f;
    s_stationary_samples = 0U;
    IMU_Calibration_Init(qspi_ready);
    if (IMU_Calibration_HasStableRuntimeBias() == 0U)
    {
        /* First boot keeps the previous trimmed-mean fallback.  Once a
         * verified W25Q64 record exists, boot applies it immediately and
         * skips the roughly 1.6 s blocking calibration. */
        imu_init();
    }
    IMU_Recorder_Init();
}

void IMU_Service_Process(void)
{
    ImuServiceSnapshot_t sample;
    uint32_t now = HAL_GetTick();
    float dt_seconds;
    float planar_rate_dps;
    uint8_t next_index;
    uint8_t timing_contiguous;
    uint32_t sample_delta_ms;
    ImuCalibrationStatus cal_status;
    static ImuCalibrationState previous_cal_state = IMU_CAL_STATE_IDLE;

    if ((uint32_t)(now - s_last_attempt_tick) < IMU_SERVICE_PERIOD_MS)
    {
        return;
    }
    s_last_attempt_tick = now;
    IMU_Calibration_Process(now);

    if (MPU6500_GetDataEx(&sample.raw_ax, &sample.raw_ay, &sample.raw_az,
                          &sample.temperature_raw,
                          &sample.raw_gx, &sample.raw_gy,
                          &sample.raw_gz) != HAL_OK)
    {
        s_failure_count++;
        return;
    }

    sample.ax = sample.raw_ax;
    sample.ay = sample.raw_ay;
    sample.az = sample.raw_az;
    sample.gx = sample.raw_gx;
    sample.gy = sample.raw_gy;
    sample.gz = sample.raw_gz;
    IMU_Calibration_ProcessRaw(sample.raw_ax, sample.raw_ay, sample.raw_az,
                               sample.raw_gx, sample.raw_gy, sample.raw_gz,
                               sample.temperature_raw, now);
    IMU_Calibration_GetStatus(&cal_status);
    if (((cal_status.state == IMU_CAL_STATE_SUCCESS) ||
         (cal_status.state == IMU_CAL_STATE_FAILED_STORAGE)) &&
        (previous_cal_state != cal_status.state))
    {
        s_planar_yaw = 0.0f;
        s_previous_planar_rate_dps = 0.0f;
        s_stationary_samples = 0U;
    }
    previous_cal_state = cal_status.state;
    if ((s_sequence == 0U) &&
        ((sample.raw_az >= IMU_SERVICE_MOUNT_Z_DETECT_MIN) ||
         (sample.raw_az <= -IMU_SERVICE_MOUNT_Z_DETECT_MIN)))
    {
        /* Detect whether the sensor Z axis points up or down. This keeps
         * clockwise-positive chassis yaw valid for either flat mounting. */
        s_planar_yaw_sign = (sample.raw_az >= 0) ? -1.0f : 1.0f;
    }

    imu_data_calibration(&sample.gx, &sample.gy, &sample.gz,
                         &sample.ax, &sample.ay, &sample.az);

    if (imu_service_sample_is_stationary(&sample) != 0U)
    {
        if (s_stationary_samples < UINT16_MAX)
        {
            s_stationary_samples++;
        }
        if ((s_stationary_samples >= IMU_SERVICE_STATIONARY_SAMPLES) &&
            (IMU_Calibration_HasStableRuntimeBias() == 0U) &&
            (IMU_Calibration_IsActive() == 0U))
        {
            /* Temperature changes gyro zero-rate output. Adapt only after a
             * full second of stationary evidence so genuine slow rotation is
             * never learned as bias. */
            imu_adapt_gyro_bias(sample.raw_gx, sample.raw_gy, sample.raw_gz,
                                IMU_SERVICE_BIAS_ADAPT_GAIN);
        }
    }
    else
    {
        s_stationary_samples = 0U;
    }

    sample.sample_tick = HAL_GetTick();
    sample_delta_ms = (s_last_sample_tick == 0U) ? IMU_SERVICE_PERIOD_MS :
        (uint32_t)(sample.sample_tick - s_last_sample_tick);
    dt_seconds = (float)sample_delta_ms * 0.001f;
    timing_contiguous = ((dt_seconds >= IMU_SERVICE_MIN_DT_SECONDS) &&
                         (dt_seconds <= IMU_SERVICE_MAX_DT_SECONDS)) ? 1U : 0U;
    if (timing_contiguous == 0U)
    {
        /* Do not integrate a long I2C failure interval using only the newest
         * angular-rate sample. The safety layer separately detects staleness. */
        dt_seconds = (float)IMU_SERVICE_PERIOD_MS * 0.001f;
    }
    s_last_sample_tick = sample.sample_tick;
    sample.angles = imu_update_eulerian_angles((float)sample.gx, (float)sample.gy, (float)sample.gz,
                                                (float)sample.ax, (float)sample.ay, (float)sample.az,
                                                dt_seconds);
    /* Field-oriented chassis control only needs planar relative heading.
     * Integrating the calibrated Z rate avoids Euler-yaw distortion caused by
     * roll/pitch fusion while the spinning chassis is accelerating. */
    planar_rate_dps = s_planar_yaw_sign *
                      ((float)sample.gz / MPU6500_GYRO_LSB_PER_DPS);
    if (fabsf(planar_rate_dps) < IMU_SERVICE_RATE_DEADBAND_DPS)
    {
        planar_rate_dps = 0.0f;
    }
    sample.planar_yaw_rate_dps = planar_rate_dps;
    s_planar_yaw = imu_service_wrap_degrees(
        s_planar_yaw +
        ((timing_contiguous != 0U) ?
            (0.5f * (s_previous_planar_rate_dps + planar_rate_dps) *
             dt_seconds) :
            (planar_rate_dps * dt_seconds)));
    s_previous_planar_rate_dps = planar_rate_dps;
    sample.planar_yaw = s_planar_yaw;
    /* A six-axis IMU has no absolute yaw reference. For a flat vehicle the
     * directly integrated, calibrated Z rate is both less coupled to pitch /
     * roll and already protected by the stationary deadband. Keep displayed
     * yaw and heading-hold consumers on this same stable definition. */
    sample.angles.yaw = sample.planar_yaw;
    sample.sequence = ++s_sequence;

    next_index = (uint8_t)(s_published_index ^ 1U);
    s_snapshots[next_index] = sample;
    __DMB();
    s_published_index = next_index;
    s_has_snapshot = 1U;
    IMU_Recorder_Push(&sample);
}

uint8_t IMU_Service_GetSnapshot(ImuServiceSnapshot_t *snapshot)
{
    uint8_t index;

    if ((snapshot == NULL) || (s_has_snapshot == 0U))
    {
        return 0U;
    }

    index = s_published_index;
    __DMB();
    *snapshot = s_snapshots[index];
    return 1U;
}

uint32_t IMU_Service_GetFailureCount(void)
{
    return s_failure_count;
}
