#include "imu.h"
#include "mpu6500.h"

#include <math.h>

#define IMU_PI                         3.14159265358979323846f
#define IMU_RAD_TO_DEG                 (180.0f / IMU_PI)
#define IMU_GYRO_LSB_PER_DPS           16.4f
#define IMU_ACCEL_1G_LSB               2048.0f
#define IMU_ACCEL_LPF_ALPHA            0.35f
#define IMU_ACCEL_TRUST_MIN_G          0.75f
#define IMU_ACCEL_TRUST_MAX_G          1.25f
#define IMU_DEFAULT_DT_SECONDS         0.010f
#define IMU_MIN_DT_SECONDS             0.001f
#define IMU_MAX_DT_SECONDS             0.050f
#define IMU_INTEGRAL_LIMIT             0.5f

quater_info_t g_q_info = {1.0f, 0.0f, 0.0f, 0.0f};

float g_param_kp = 10.0f;
float g_param_ki = 0.02f;

short g_acc_avg[3] = {0, 0, 0};
short g_gyro_avg[3] = {0, 0, 0};

static float s_accel_lpf[3] = {0.0f, 0.0f, 0.0f};
static float s_integral_error[3] = {0.0f, 0.0f, 0.0f};
static uint8_t s_accel_lpf_initialized = 0U;
static uint32_t s_last_wrapper_tick = 0U;

static float imu_inv_sqrt(float value)
{
    if (value <= 0.0f)
    {
        return 0.0f;
    }
    return 1.0f / sqrtf(value);
}

static float imu_clampf(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static void imu_reset_fusion_state(void)
{
    g_q_info.q0 = 1.0f;
    g_q_info.q1 = 0.0f;
    g_q_info.q2 = 0.0f;
    g_q_info.q3 = 0.0f;

    s_accel_lpf[0] = 0.0f;
    s_accel_lpf[1] = 0.0f;
    s_accel_lpf[2] = 0.0f;
    s_integral_error[0] = 0.0f;
    s_integral_error[1] = 0.0f;
    s_integral_error[2] = 0.0f;
    s_accel_lpf_initialized = 0U;
    s_last_wrapper_tick = 0U;
}

static void imu_data_transform(float *gx, float *gy, float *gz,
                               float *ax, float *ay, float *az)
{
    if (s_accel_lpf_initialized == 0U)
    {
        s_accel_lpf[0] = *ax;
        s_accel_lpf[1] = *ay;
        s_accel_lpf[2] = *az;
        s_accel_lpf_initialized = 1U;
    }
    else
    {
        s_accel_lpf[0] += IMU_ACCEL_LPF_ALPHA * (*ax - s_accel_lpf[0]);
        s_accel_lpf[1] += IMU_ACCEL_LPF_ALPHA * (*ay - s_accel_lpf[1]);
        s_accel_lpf[2] += IMU_ACCEL_LPF_ALPHA * (*az - s_accel_lpf[2]);
    }

    *ax = s_accel_lpf[0];
    *ay = s_accel_lpf[1];
    *az = s_accel_lpf[2];

    *gx = (*gx / IMU_GYRO_LSB_PER_DPS) * (IMU_PI / 180.0f);
    *gy = (*gy / IMU_GYRO_LSB_PER_DPS) * (IMU_PI / 180.0f);
    *gz = (*gz / IMU_GYRO_LSB_PER_DPS) * (IMU_PI / 180.0f);
}

static void imu_ahrsupdate_nomagnetic(float gx, float gy, float gz,
                                      float ax, float ay, float az,
                                      float dt_seconds)
{
    float q0 = g_q_info.q0;
    float q1 = g_q_info.q1;
    float q2 = g_q_info.q2;
    float q3 = g_q_info.q3;
    float accel_norm_sq = ax * ax + ay * ay + az * az;
    const float accel_min = IMU_ACCEL_1G_LSB * IMU_ACCEL_TRUST_MIN_G;
    const float accel_max = IMU_ACCEL_1G_LSB * IMU_ACCEL_TRUST_MAX_G;
    float norm;

    /* Acceleration is a gravity reference only while its magnitude is near 1 g. */
    if ((accel_norm_sq >= accel_min * accel_min) &&
        (accel_norm_sq <= accel_max * accel_max))
    {
        float vx;
        float vy;
        float vz;
        float ex;
        float ey;
        float ez;

        norm = imu_inv_sqrt(accel_norm_sq);
        ax *= norm;
        ay *= norm;
        az *= norm;

        vx = 2.0f * (q1 * q3 - q0 * q2);
        vy = 2.0f * (q0 * q1 + q2 * q3);
        vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        /* Full body-frame gravity cross product. ez is not a magnetic yaw reference. */
        ex = ay * vz - az * vy;
        ey = az * vx - ax * vz;
        ez = ax * vy - ay * vx;

        s_integral_error[0] = imu_clampf(s_integral_error[0] + dt_seconds * ex,
                                        -IMU_INTEGRAL_LIMIT, IMU_INTEGRAL_LIMIT);
        s_integral_error[1] = imu_clampf(s_integral_error[1] + dt_seconds * ey,
                                        -IMU_INTEGRAL_LIMIT, IMU_INTEGRAL_LIMIT);
        s_integral_error[2] = imu_clampf(s_integral_error[2] + dt_seconds * ez,
                                        -IMU_INTEGRAL_LIMIT, IMU_INTEGRAL_LIMIT);

        gx += g_param_kp * ex + g_param_ki * s_integral_error[0];
        gy += g_param_kp * ey + g_param_ki * s_integral_error[1];
        gz += g_param_kp * ez + g_param_ki * s_integral_error[2];
    }
    else
    {
        /* Do not tilt the attitude toward transient linear acceleration. */
        s_integral_error[0] *= 0.98f;
        s_integral_error[1] *= 0.98f;
        s_integral_error[2] *= 0.98f;
    }

    {
        float half_t = 0.5f * dt_seconds;
        float delta_2 = (half_t * gx) * (half_t * gx) +
                        (half_t * gy) * (half_t * gy) +
                        (half_t * gz) * (half_t * gz);
        float scale = 1.0f - 0.5f * delta_2;
        float old_q0 = q0;
        float old_q1 = q1;
        float old_q2 = q2;
        float old_q3 = q3;

        q0 = scale * old_q0 + (-old_q1 * gx - old_q2 * gy - old_q3 * gz) * half_t;
        q1 = scale * old_q1 + ( old_q0 * gx + old_q2 * gz - old_q3 * gy) * half_t;
        q2 = scale * old_q2 + ( old_q0 * gy - old_q1 * gz + old_q3 * gx) * half_t;
        q3 = scale * old_q3 + ( old_q0 * gz + old_q1 * gy - old_q2 * gx) * half_t;
    }

    norm = imu_inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (norm > 0.0f)
    {
        g_q_info.q0 = q0 * norm;
        g_q_info.q1 = q1 * norm;
        g_q_info.q2 = q2 * norm;
        g_q_info.q3 = q3 * norm;
    }
}

eulerian_angles_t imu_update_eulerian_angles(float gx, float gy, float gz,
                                             float ax, float ay, float az,
                                             float dt_seconds)
{
    eulerian_angles_t angles;
    float q0;
    float q1;
    float q2;
    float q3;
    float pitch_sine;

    if ((dt_seconds < IMU_MIN_DT_SECONDS) || (dt_seconds > IMU_MAX_DT_SECONDS))
    {
        dt_seconds = IMU_DEFAULT_DT_SECONDS;
    }

    imu_data_transform(&gx, &gy, &gz, &ax, &ay, &az);
    imu_ahrsupdate_nomagnetic(gx, gy, gz, ax, ay, az, dt_seconds);

    q0 = g_q_info.q0;
    q1 = g_q_info.q1;
    q2 = g_q_info.q2;
    q3 = g_q_info.q3;

    pitch_sine = imu_clampf(2.0f * (q0 * q2 - q1 * q3), -1.0f, 1.0f);
    angles.pitch = -asinf(pitch_sine) * IMU_RAD_TO_DEG;
    angles.roll = atan2f(2.0f * (q0 * q1 + q2 * q3),
                         1.0f - 2.0f * (q1 * q1 + q2 * q2)) * IMU_RAD_TO_DEG;
    angles.yaw = -atan2f(2.0f * (q0 * q3 + q1 * q2),
                         1.0f - 2.0f * (q2 * q2 + q3 * q3)) * IMU_RAD_TO_DEG;

    return angles;
}

eulerian_angles_t imu_get_eulerian_angles(float gx, float gy, float gz,
                                          float ax, float ay, float az)
{
    uint32_t now = HAL_GetTick();
    float dt_seconds = IMU_DEFAULT_DT_SECONDS;

    if (s_last_wrapper_tick != 0U)
    {
        dt_seconds = (float)((uint32_t)(now - s_last_wrapper_tick)) * 0.001f;
    }
    s_last_wrapper_tick = now;

    return imu_update_eulerian_angles(gx, gy, gz, ax, ay, az, dt_seconds);
}

void imu_data_calibration(short *gx, short *gy, short *gz,
                          short *ax, short *ay, short *az)
{
    *gx -= g_gyro_avg[0];
    *gy -= g_gyro_avg[1];
    *gz -= g_gyro_avg[2];
    *ax -= g_acc_avg[0];
    *ay -= g_acc_avg[1];
    *az -= (g_acc_avg[2] - (short)IMU_ACCEL_1G_LSB);

    if ((*gx >= -15) && (*gx <= 15)) *gx = 0;
    if ((*gy >= -15) && (*gy <= 15)) *gy = 0;
    if ((*gz >= -15) && (*gz <= 15)) *gz = 0;
}

void imu_init(void)
{
    uint16_t i;
    uint16_t sample_count = 0U;
    int32_t acc_sum[3] = {0, 0, 0};
    int32_t gyro_sum[3] = {0, 0, 0};
    short acc_data[3];
    short gyro_data[3];

    g_acc_avg[0] = 0;
    g_acc_avg[1] = 0;
    g_acc_avg[2] = 0;
    g_gyro_avg[0] = 0;
    g_gyro_avg[1] = 0;
    g_gyro_avg[2] = 0;
    imu_reset_fusion_state();

    HAL_Delay(100U);
    for (i = 0U; i < 250U; ++i)
    {
        if (MPU6500_GetData(&acc_data[0], &acc_data[1], &acc_data[2],
                            &gyro_data[0], &gyro_data[1], &gyro_data[2]) != HAL_OK)
        {
            HAL_Delay(5U);
            continue;
        }

        acc_sum[0] += acc_data[0];
        acc_sum[1] += acc_data[1];
        acc_sum[2] += acc_data[2];
        gyro_sum[0] += gyro_data[0];
        gyro_sum[1] += gyro_data[1];
        gyro_sum[2] += gyro_data[2];
        sample_count++;
        HAL_Delay(5U);
    }

    if (sample_count != 0U)
    {
        g_acc_avg[0] = (short)(acc_sum[0] / sample_count);
        g_acc_avg[1] = (short)(acc_sum[1] / sample_count);
        g_acc_avg[2] = (short)(acc_sum[2] / sample_count);
        g_gyro_avg[0] = (short)(gyro_sum[0] / sample_count);
        g_gyro_avg[1] = (short)(gyro_sum[1] / sample_count);
        g_gyro_avg[2] = (short)(gyro_sum[2] / sample_count);
    }
}
