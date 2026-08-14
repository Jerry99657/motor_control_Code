#include "imu.h"
#include "mpu6500.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#define IMU_PI                         3.14159265358979323846f
#define IMU_RAD_TO_DEG                 (180.0f / IMU_PI)
#define IMU_DEG_TO_RAD                 (IMU_PI / 180.0f)
#define IMU_ACCEL_LPF_CUTOFF_HZ        8.0f
#define IMU_ACCEL_TRUST_FULL_ERROR_G   0.04f
#define IMU_ACCEL_TRUST_ZERO_ERROR_G   0.20f
#define IMU_INNOVATION_FULL_TRUST      0.035f
#define IMU_INNOVATION_ZERO_TRUST      0.20f
#define IMU_DEFAULT_DT_SECONDS         0.010f
#define IMU_MIN_DT_SECONDS             0.001f
#define IMU_MAX_DT_SECONDS             0.050f
#define IMU_INTEGRAL_LIMIT             0.5f
#define IMU_INTEGRAL_TRUST_MIN         0.75f
#define IMU_CALIBRATION_SAMPLES        300U
#define IMU_CALIBRATION_DELAY_MS       5U
#define IMU_CALIBRATION_MIN_SAMPLES    80U
#define IMU_CALIBRATION_MAX_GYRO_DPS   10.0f
#define IMU_CALIBRATION_ACCEL_MIN_G    0.75f
#define IMU_CALIBRATION_ACCEL_MAX_G    1.25f
#define IMU_GYRO_ZERO_DEADBAND_DPS     0.10f
#define IMU_ACCEL_ZERO_DEADBAND_G      0.020f

quater_info_t g_q_info = {1.0f, 0.0f, 0.0f, 0.0f};

float g_param_kp = 4.5f;
float g_param_ki = 0.03f;

static float s_accel_offset[3] = {0.0f, 0.0f, 0.0f};
static float s_gyro_bias[3] = {0.0f, 0.0f, 0.0f};

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

static int16_t imu_float_to_i16(float value)
{
    if (value > 32767.0f) return 32767;
    if (value < -32768.0f) return -32768;
    return (int16_t)lroundf(value);
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

static float imu_data_transform(float *gx, float *gy, float *gz,
                                float *ax, float *ay, float *az,
                                float dt_seconds)
{
    float accel_norm;
    float accel_error_g;
    float accel_trust;
    float filter_tau;
    float alpha;

    accel_norm = sqrtf((*ax * *ax) + (*ay * *ay) + (*az * *az));
    accel_error_g = fabsf((accel_norm / MPU6500_ACCEL_LSB_PER_G) - 1.0f);
    if (accel_error_g <= IMU_ACCEL_TRUST_FULL_ERROR_G)
    {
        accel_trust = 1.0f;
    }
    else if (accel_error_g >= IMU_ACCEL_TRUST_ZERO_ERROR_G)
    {
        accel_trust = 0.0f;
    }
    else
    {
        accel_trust = (IMU_ACCEL_TRUST_ZERO_ERROR_G - accel_error_g) /
                      (IMU_ACCEL_TRUST_ZERO_ERROR_G -
                       IMU_ACCEL_TRUST_FULL_ERROR_G);
    }

    filter_tau = 1.0f / (2.0f * IMU_PI * IMU_ACCEL_LPF_CUTOFF_HZ);
    alpha = imu_clampf(dt_seconds / (filter_tau + dt_seconds), 0.05f, 1.0f);
    if (s_accel_lpf_initialized == 0U)
    {
        s_accel_lpf[0] = *ax;
        s_accel_lpf[1] = *ay;
        s_accel_lpf[2] = *az;
        s_accel_lpf_initialized = 1U;
    }
    else
    {
        s_accel_lpf[0] += alpha * (*ax - s_accel_lpf[0]);
        s_accel_lpf[1] += alpha * (*ay - s_accel_lpf[1]);
        s_accel_lpf[2] += alpha * (*az - s_accel_lpf[2]);
    }

    *ax = s_accel_lpf[0];
    *ay = s_accel_lpf[1];
    *az = s_accel_lpf[2];

    *gx = (*gx / MPU6500_GYRO_LSB_PER_DPS) * IMU_DEG_TO_RAD;
    *gy = (*gy / MPU6500_GYRO_LSB_PER_DPS) * IMU_DEG_TO_RAD;
    *gz = (*gz / MPU6500_GYRO_LSB_PER_DPS) * IMU_DEG_TO_RAD;
    return accel_trust;
}

static void imu_ahrsupdate_nomagnetic(float gx, float gy, float gz,
                                      float ax, float ay, float az,
                                      float dt_seconds,
                                      float accel_trust)
{
    float q0 = g_q_info.q0;
    float q1 = g_q_info.q1;
    float q2 = g_q_info.q2;
    float q3 = g_q_info.q3;
    float accel_norm_sq = ax * ax + ay * ay + az * az;
    float norm;

    /* Fade gravity feedback continuously as linear acceleration grows. A hard
     * trust window makes a vehicle attitude jump when acceleration crosses a
     * threshold. */
    if ((accel_trust > 0.0f) && (accel_norm_sq > 1.0f))
    {
        float vx;
        float vy;
        float vz;
        float ex;
        float ey;
        float ez;
        float innovation;
        float innovation_trust;
        float feedback_trust;

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

        /* Magnitude gating alone misses horizontal vehicle acceleration: the
         * norm of gravity plus 0.1 g sideways changes by only about 0.5%.
         * Also fade correction when the measured and predicted gravity
         * directions disagree, while retaining gyro propagation. */
        innovation = sqrtf((ex * ex) + (ey * ey) + (ez * ez));
        if (innovation <= IMU_INNOVATION_FULL_TRUST)
        {
            innovation_trust = 1.0f;
        }
        else if (innovation >= IMU_INNOVATION_ZERO_TRUST)
        {
            innovation_trust = 0.0f;
        }
        else
        {
            innovation_trust = (IMU_INNOVATION_ZERO_TRUST - innovation) /
                (IMU_INNOVATION_ZERO_TRUST - IMU_INNOVATION_FULL_TRUST);
        }
        feedback_trust = accel_trust * innovation_trust * innovation_trust;

        if (feedback_trust >= IMU_INTEGRAL_TRUST_MIN)
        {
            s_integral_error[0] = imu_clampf(
                s_integral_error[0] + dt_seconds * feedback_trust * ex,
                -IMU_INTEGRAL_LIMIT, IMU_INTEGRAL_LIMIT);
            s_integral_error[1] = imu_clampf(
                s_integral_error[1] + dt_seconds * feedback_trust * ey,
                -IMU_INTEGRAL_LIMIT, IMU_INTEGRAL_LIMIT);
        }
        else
        {
            s_integral_error[0] *= (1.0f - dt_seconds);
            s_integral_error[1] *= (1.0f - dt_seconds);
        }
        /* Gravity alone cannot observe heading. Never integrate a synthetic
         * Z correction into yaw when no magnetometer is present. */
        s_integral_error[2] = 0.0f;

        gx += g_param_kp * feedback_trust * ex +
              g_param_ki * s_integral_error[0];
        gy += g_param_kp * feedback_trust * ey +
              g_param_ki * s_integral_error[1];
        gz += g_param_kp * feedback_trust * ez;
    }
    else
    {
        /* Do not tilt the attitude toward transient linear acceleration. */
        float decay = imu_clampf(1.0f - (2.0f * dt_seconds), 0.0f, 1.0f);
        s_integral_error[0] *= decay;
        s_integral_error[1] *= decay;
        s_integral_error[2] = 0.0f;
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
    float accel_trust;

    if ((dt_seconds < IMU_MIN_DT_SECONDS) || (dt_seconds > IMU_MAX_DT_SECONDS))
    {
        dt_seconds = IMU_DEFAULT_DT_SECONDS;
    }

    accel_trust = imu_data_transform(&gx, &gy, &gz, &ax, &ay, &az,
                                     dt_seconds);
    imu_ahrsupdate_nomagnetic(gx, gy, gz, ax, ay, az, dt_seconds,
                              accel_trust);

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
    const int16_t gyro_deadband = (int16_t)
        (MPU6500_GYRO_LSB_PER_DPS * IMU_GYRO_ZERO_DEADBAND_DPS + 0.5f);
    const int16_t accel_deadband = (int16_t)
        (MPU6500_ACCEL_LSB_PER_G * IMU_ACCEL_ZERO_DEADBAND_G + 0.5f);

    *gx = imu_float_to_i16((float)*gx - s_gyro_bias[0]);
    *gy = imu_float_to_i16((float)*gy - s_gyro_bias[1]);
    *gz = imu_float_to_i16((float)*gz - s_gyro_bias[2]);
    *ax = imu_float_to_i16((float)*ax - s_accel_offset[0]);
    *ay = imu_float_to_i16((float)*ay - s_accel_offset[1]);
    *az = imu_float_to_i16((float)*az - s_accel_offset[2]);

    /* Express the deadbands in physical units so changing sensor range never
     * changes their real meaning. The previous implementation removed this
     * step, allowing 2-3 raw gyro counts to accumulate into visible yaw. */
    if (abs((int)*gx) <= gyro_deadband) *gx = 0;
    if (abs((int)*gy) <= gyro_deadband) *gy = 0;
    if (abs((int)*gz) <= gyro_deadband) *gz = 0;

    /* The robot is calibrated around its boot-time chassis plane. Suppress
     * residual acceleration below 0.02 g (about 1.1 degrees) in the published
     * raw channels; larger tilt/acceleration remains fully measurable. */
    if (abs((int)*ax) <= accel_deadband) *ax = 0;
    if (abs((int)*ay) <= accel_deadband) *ay = 0;
    if (abs((int)*az - MPU6500_ACCEL_1G_LSB) <= accel_deadband)
    {
        *az = MPU6500_ACCEL_1G_LSB;
    }
}

void imu_adapt_gyro_bias(short raw_gx, short raw_gy, short raw_gz,
                         float gain)
{
    gain = imu_clampf(gain, 0.0f, 0.01f);
    s_gyro_bias[0] += gain * ((float)raw_gx - s_gyro_bias[0]);
    s_gyro_bias[1] += gain * ((float)raw_gy - s_gyro_bias[1]);
    s_gyro_bias[2] += gain * ((float)raw_gz - s_gyro_bias[2]);
}

void imu_set_calibration(const imu_calibration_t *calibration)
{
    uint8_t axis;

    if (calibration == NULL)
    {
        return;
    }

    for (axis = 0U; axis < 3U; ++axis)
    {
        s_gyro_bias[axis] = calibration->gyro_bias[axis];
        s_accel_offset[axis] = calibration->accel_offset[axis];
    }
}

void imu_get_calibration(imu_calibration_t *calibration)
{
    uint8_t axis;

    if (calibration == NULL)
    {
        return;
    }

    for (axis = 0U; axis < 3U; ++axis)
    {
        calibration->gyro_bias[axis] = s_gyro_bias[axis];
        calibration->accel_offset[axis] = s_accel_offset[axis];
    }
}

void imu_reset_attitude(void)
{
    imu_reset_fusion_state();
}

void imu_init(void)
{
    uint16_t i;
    uint16_t sample_count = 0U;
    int64_t acc_sum[3] = {0, 0, 0};
    int64_t gyro_sum[3] = {0, 0, 0};
    int16_t acc_min[3] = {INT16_MAX, INT16_MAX, INT16_MAX};
    int16_t acc_max[3] = {INT16_MIN, INT16_MIN, INT16_MIN};
    int16_t gyro_min[3] = {INT16_MAX, INT16_MAX, INT16_MAX};
    int16_t gyro_max[3] = {INT16_MIN, INT16_MIN, INT16_MIN};
    short acc_data[3];
    short gyro_data[3];
    int64_t accel_norm_sq;
    const int32_t accel_min =
        (int32_t)(MPU6500_ACCEL_LSB_PER_G * IMU_CALIBRATION_ACCEL_MIN_G);
    const int32_t accel_max =
        (int32_t)(MPU6500_ACCEL_LSB_PER_G * IMU_CALIBRATION_ACCEL_MAX_G);
    const int32_t gyro_limit =
        (int32_t)(MPU6500_GYRO_LSB_PER_DPS * IMU_CALIBRATION_MAX_GYRO_DPS);
    uint8_t axis;

    s_accel_offset[0] = 0.0f;
    s_accel_offset[1] = 0.0f;
    s_accel_offset[2] = 0.0f;
    s_gyro_bias[0] = 0.0f;
    s_gyro_bias[1] = 0.0f;
    s_gyro_bias[2] = 0.0f;
    imu_reset_fusion_state();

    HAL_Delay(100U);
    for (i = 0U; i < IMU_CALIBRATION_SAMPLES; ++i)
    {
        if (MPU6500_GetData(&acc_data[0], &acc_data[1], &acc_data[2],
                            &gyro_data[0], &gyro_data[1], &gyro_data[2]) != HAL_OK)
        {
            HAL_Delay(IMU_CALIBRATION_DELAY_MS);
            continue;
        }

        accel_norm_sq = (int64_t)acc_data[0] * acc_data[0] +
                        (int64_t)acc_data[1] * acc_data[1] +
                        (int64_t)acc_data[2] * acc_data[2];
        if ((accel_norm_sq < ((int64_t)accel_min * accel_min)) ||
            (accel_norm_sq > ((int64_t)accel_max * accel_max)) ||
            ((int32_t)abs(gyro_data[0]) > gyro_limit) ||
            ((int32_t)abs(gyro_data[1]) > gyro_limit) ||
            ((int32_t)abs(gyro_data[2]) > gyro_limit))
        {
            HAL_Delay(IMU_CALIBRATION_DELAY_MS);
            continue;
        }

        for (axis = 0U; axis < 3U; ++axis)
        {
            acc_sum[axis] += acc_data[axis];
            gyro_sum[axis] += gyro_data[axis];
            if (acc_data[axis] < acc_min[axis]) acc_min[axis] = acc_data[axis];
            if (acc_data[axis] > acc_max[axis]) acc_max[axis] = acc_data[axis];
            if (gyro_data[axis] < gyro_min[axis]) gyro_min[axis] = gyro_data[axis];
            if (gyro_data[axis] > gyro_max[axis]) gyro_max[axis] = gyro_data[axis];
        }
        sample_count++;
        HAL_Delay(IMU_CALIBRATION_DELAY_MS);
    }

    if (sample_count >= IMU_CALIBRATION_MIN_SAMPLES)
    {
        uint16_t divisor = sample_count;

        /* Remove one high and one low sample per axis. This inexpensive
         * trimmed mean prevents a single vibration spike from becoming a
         * permanent boot-time offset. */
        if (sample_count > 2U)
        {
            divisor = (uint16_t)(sample_count - 2U);
            for (axis = 0U; axis < 3U; ++axis)
            {
                acc_sum[axis] -= (int64_t)acc_min[axis] + acc_max[axis];
                gyro_sum[axis] -= (int64_t)gyro_min[axis] + gyro_max[axis];
            }
        }

        for (axis = 0U; axis < 3U; ++axis)
        {
            s_gyro_bias[axis] = (float)gyro_sum[axis] / (float)divisor;
        }
        s_accel_offset[0] = (float)acc_sum[0] / (float)divisor;
        s_accel_offset[1] = (float)acc_sum[1] / (float)divisor;
        s_accel_offset[2] = ((float)acc_sum[2] / (float)divisor) -
                            MPU6500_ACCEL_LSB_PER_G;
    }
}
