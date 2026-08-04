#include "imu_service.h"
#include "mpu6500.h"

#define IMU_SERVICE_PERIOD_MS 10U
#define IMU_SERVICE_GYRO_LSB_PER_DPS 16.4f
#define IMU_SERVICE_MOUNT_Z_DETECT_MIN 1024

static ImuServiceSnapshot_t s_snapshots[2];
static volatile uint8_t s_published_index = 0U;
static volatile uint8_t s_has_snapshot = 0U;
static uint32_t s_last_attempt_tick = 0U;
static uint32_t s_last_sample_tick = 0U;
static uint32_t s_sequence = 0U;
static uint32_t s_failure_count = 0U;
static float s_planar_yaw = 0.0f;
static float s_planar_yaw_sign = -1.0f;

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

void IMU_Service_Init(void)
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
}

void IMU_Service_Process(void)
{
    ImuServiceSnapshot_t sample;
    uint32_t now = HAL_GetTick();
    float dt_seconds;
    uint8_t next_index;
    int16_t raw_az;

    if ((uint32_t)(now - s_last_attempt_tick) < IMU_SERVICE_PERIOD_MS)
    {
        return;
    }
    s_last_attempt_tick = now;

    if (MPU6500_GetData(&sample.ax, &sample.ay, &sample.az,
                        &sample.gx, &sample.gy, &sample.gz) != HAL_OK)
    {
        s_failure_count++;
        return;
    }

    raw_az = sample.az;
    if ((s_sequence == 0U) &&
        ((raw_az >= IMU_SERVICE_MOUNT_Z_DETECT_MIN) ||
         (raw_az <= -IMU_SERVICE_MOUNT_Z_DETECT_MIN)))
    {
        /* Detect whether the sensor Z axis points up or down. This keeps
         * clockwise-positive chassis yaw valid for either flat mounting. */
        s_planar_yaw_sign = (raw_az >= 0) ? -1.0f : 1.0f;
    }

    imu_data_calibration(&sample.gx, &sample.gy, &sample.gz,
                         &sample.ax, &sample.ay, &sample.az);
    sample.sample_tick = HAL_GetTick();
    dt_seconds = (s_last_sample_tick == 0U)
                   ? ((float)IMU_SERVICE_PERIOD_MS * 0.001f)
                   : ((float)((uint32_t)(sample.sample_tick - s_last_sample_tick)) * 0.001f);
    s_last_sample_tick = sample.sample_tick;
    sample.angles = imu_update_eulerian_angles((float)sample.gx, (float)sample.gy, (float)sample.gz,
                                                (float)sample.ax, (float)sample.ay, (float)sample.az,
                                                dt_seconds);
    /* Field-oriented chassis control only needs planar relative heading.
     * Integrating the calibrated Z rate avoids Euler-yaw distortion caused by
     * roll/pitch fusion while the spinning chassis is accelerating. */
    sample.planar_yaw_rate_dps = s_planar_yaw_sign *
                                 ((float)sample.gz / IMU_SERVICE_GYRO_LSB_PER_DPS);
    s_planar_yaw = imu_service_wrap_degrees(
        s_planar_yaw + sample.planar_yaw_rate_dps * dt_seconds);
    sample.planar_yaw = s_planar_yaw;
    sample.sequence = ++s_sequence;

    next_index = (uint8_t)(s_published_index ^ 1U);
    s_snapshots[next_index] = sample;
    __DMB();
    s_published_index = next_index;
    s_has_snapshot = 1U;
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
