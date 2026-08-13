#include "mecanum_odometry.h"

#include "dc_motor_ol.h"
#include "mecanum.h"
#include "main.h"
#include <math.h>
#include <string.h>

#define MECANUM_ODOMETRY_NOMINAL_PERIOD_MS       10U
#define MECANUM_ODOMETRY_MAX_PERIOD_MS           100U
#define MECANUM_ODOMETRY_MAX_COUNTS_PER_10MS     200LL
#define MECANUM_ODOMETRY_VELOCITY_LPF_ALPHA      0.35f
#define MECANUM_ODOMETRY_PI                       3.14159265358979323846f
#define MECANUM_ODOMETRY_RAD_TO_DEG               \
    (180.0f / MECANUM_ODOMETRY_PI)

static MecanumOdometrySnapshot s_odometry;
static int64_t s_previous_counts[4];
static float s_heading_rad = 0.0f;
static volatile uint8_t s_reset_requested = 0U;

static float mecanum_odometry_wrap_degrees(float angle)
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

static void mecanum_odometry_read_counts(int64_t counts[4])
{
    uint8_t i;

    for (i = 0U; i < 4U; ++i)
    {
        counts[i] = DCMotor_OL_GetPositionPulses((uint8_t)(i + 1U));
    }
}

static uint8_t mecanum_odometry_encoder_ready_mask(void)
{
    uint8_t mask = 0U;
    uint8_t i;

    for (i = 0U; i < 4U; ++i)
    {
        if (DCMotor_OL_GetEncoderPolarity((uint8_t)(i + 1U)) != 0)
        {
            mask |= (uint8_t)(1U << i);
        }
    }
    return mask;
}

static void mecanum_odometry_reset_pose(const int64_t counts[4],
                                         uint32_t now)
{
    uint8_t i;

    s_odometry.x_mm = 0.0f;
    s_odometry.y_mm = 0.0f;
    s_odometry.heading_deg = 0.0f;
    s_odometry.heading_total_deg = 0.0f;
    s_odometry.body_forward_total_mm = 0.0f;
    s_odometry.body_right_total_mm = 0.0f;
    s_odometry.body_vx_mm_s = 0.0f;
    s_odometry.body_vy_mm_s = 0.0f;
    s_odometry.body_wz_deg_s = 0.0f;
    s_odometry.travel_distance_mm = 0.0f;
    memset(s_odometry.wheel_delta_counts, 0,
           sizeof(s_odometry.wheel_delta_counts));
    for (i = 0U; i < 4U; ++i)
    {
        s_previous_counts[i] = counts[i];
    }
    s_heading_rad = 0.0f;
    s_odometry.last_update_tick = now;
    s_odometry.last_sample_accepted = 1U;
    s_odometry.initialized = 1U;
    s_odometry.reset_sequence++;
}

void MecanumOdometry_Init(void)
{
    int64_t counts[4];

    memset(&s_odometry, 0, sizeof(s_odometry));
    mecanum_odometry_read_counts(counts);
    mecanum_odometry_reset_pose(counts, HAL_GetTick());
    s_odometry.update_count = 0U;
    s_odometry.rejected_count = 0U;
    s_odometry.timing_gap_count = 0U;
    s_odometry.encoder_ready_mask = mecanum_odometry_encoder_ready_mask();
    s_reset_requested = 0U;
}

void MecanumOdometry_Update10ms(void)
{
    int64_t counts[4];
    int64_t raw_delta[4];
    int64_t max_delta;
    uint32_t now = HAL_GetTick();
    uint32_t elapsed_ms;
    float dt_s;
    float mm_per_count;
    float wheel_mm[4];
    float body_dx;
    float body_dy;
    float body_dtheta;
    float heading_mid;
    float cos_heading;
    float sin_heading;
    float measured_vx;
    float measured_vy;
    float measured_wz;
    uint8_t sample_accepted = 1U;
    uint8_t any_delta = 0U;
    uint8_t i;

    mecanum_odometry_read_counts(counts);
    if ((s_odometry.initialized == 0U) || (s_reset_requested != 0U))
    {
        s_reset_requested = 0U;
        mecanum_odometry_reset_pose(counts, now);
        s_odometry.encoder_ready_mask = mecanum_odometry_encoder_ready_mask();
        return;
    }

    elapsed_ms = now - s_odometry.last_update_tick;
    if ((elapsed_ms == 0U) || (elapsed_ms > MECANUM_ODOMETRY_MAX_PERIOD_MS))
    {
        s_odometry.timing_gap_count++;
        elapsed_ms = MECANUM_ODOMETRY_NOMINAL_PERIOD_MS;
    }
    dt_s = (float)elapsed_ms * 0.001f;

    max_delta = MECANUM_ODOMETRY_MAX_COUNTS_PER_10MS;
    if (elapsed_ms > MECANUM_ODOMETRY_NOMINAL_PERIOD_MS)
    {
        max_delta = (max_delta * (int64_t)elapsed_ms) /
                    (int64_t)MECANUM_ODOMETRY_NOMINAL_PERIOD_MS;
    }

    for (i = 0U; i < 4U; ++i)
    {
        raw_delta[i] = counts[i] - s_previous_counts[i];
        s_previous_counts[i] = counts[i];
        if ((raw_delta[i] > max_delta) || (raw_delta[i] < -max_delta))
        {
            sample_accepted = 0U;
        }
        if (raw_delta[i] != 0)
        {
            any_delta = 1U;
        }
    }

    s_odometry.last_update_tick = now;
    s_odometry.encoder_ready_mask = mecanum_odometry_encoder_ready_mask();
    s_odometry.update_count++;
    if (sample_accepted == 0U)
    {
        memset(s_odometry.wheel_delta_counts, 0,
               sizeof(s_odometry.wheel_delta_counts));
        s_odometry.body_vx_mm_s *=
            (1.0f - MECANUM_ODOMETRY_VELOCITY_LPF_ALPHA);
        s_odometry.body_vy_mm_s *=
            (1.0f - MECANUM_ODOMETRY_VELOCITY_LPF_ALPHA);
        s_odometry.body_wz_deg_s *=
            (1.0f - MECANUM_ODOMETRY_VELOCITY_LPF_ALPHA);
        s_odometry.rejected_count++;
        s_odometry.last_sample_accepted = 0U;
        return;
    }

    if (any_delta == 0U)
    {
        memset(s_odometry.wheel_delta_counts, 0,
               sizeof(s_odometry.wheel_delta_counts));
        s_odometry.body_vx_mm_s *=
            (1.0f - MECANUM_ODOMETRY_VELOCITY_LPF_ALPHA);
        s_odometry.body_vy_mm_s *=
            (1.0f - MECANUM_ODOMETRY_VELOCITY_LPF_ALPHA);
        s_odometry.body_wz_deg_s *=
            (1.0f - MECANUM_ODOMETRY_VELOCITY_LPF_ALPHA);
        if (fabsf(s_odometry.body_vx_mm_s) < 0.1f)
            s_odometry.body_vx_mm_s = 0.0f;
        if (fabsf(s_odometry.body_vy_mm_s) < 0.1f)
            s_odometry.body_vy_mm_s = 0.0f;
        if (fabsf(s_odometry.body_wz_deg_s) < 0.1f)
            s_odometry.body_wz_deg_s = 0.0f;
        s_odometry.last_sample_accepted = 1U;
        return;
    }

    mm_per_count = MECANUM_WHEEL_CIRCUMFERENCE_MM /
                   ((float)DCMOTOR_OL_ENCODER_COUNTS_PER_REV *
                    MECANUM_REDUCTION_RATIO);
    for (i = 0U; i < 4U; ++i)
    {
        s_odometry.wheel_delta_counts[i] = (int32_t)raw_delta[i];
        wheel_mm[i] = (float)raw_delta[i] * mm_per_count;
    }

    /* Exact inverse of mecanum_body_to_wheels(). Encoder counts have already
     * been normalized to each motor's commanded positive direction. */
    body_dx = (wheel_mm[0] - wheel_mm[1] - wheel_mm[2] + wheel_mm[3]) * 0.25f;
    body_dy = (wheel_mm[0] + wheel_mm[1] - wheel_mm[2] - wheel_mm[3]) * 0.25f;
    body_dtheta = (wheel_mm[0] + wheel_mm[1] + wheel_mm[2] + wheel_mm[3]) /
                  (4.0f * MECANUM_K_ROTATION_COEFF_MM);

    /* Midpoint integration is noticeably better than rotating the complete
     * translation increment by either the old or the new heading. */
    heading_mid = s_heading_rad + body_dtheta * 0.5f;
    cos_heading = cosf(heading_mid);
    sin_heading = sinf(heading_mid);
    s_odometry.x_mm += cos_heading * body_dx - sin_heading * body_dy;
    s_odometry.y_mm += sin_heading * body_dx + cos_heading * body_dy;
    s_heading_rad += body_dtheta;
    s_odometry.heading_total_deg = s_heading_rad * MECANUM_ODOMETRY_RAD_TO_DEG;
    s_odometry.heading_deg =
        mecanum_odometry_wrap_degrees(s_odometry.heading_total_deg);
    s_odometry.body_forward_total_mm += body_dx;
    s_odometry.body_right_total_mm += body_dy;
    s_odometry.travel_distance_mm += sqrtf(body_dx * body_dx + body_dy * body_dy);

    measured_vx = body_dx / dt_s;
    measured_vy = body_dy / dt_s;
    measured_wz = body_dtheta * MECANUM_ODOMETRY_RAD_TO_DEG / dt_s;
    s_odometry.body_vx_mm_s += MECANUM_ODOMETRY_VELOCITY_LPF_ALPHA *
                              (measured_vx - s_odometry.body_vx_mm_s);
    s_odometry.body_vy_mm_s += MECANUM_ODOMETRY_VELOCITY_LPF_ALPHA *
                              (measured_vy - s_odometry.body_vy_mm_s);
    s_odometry.body_wz_deg_s += MECANUM_ODOMETRY_VELOCITY_LPF_ALPHA *
                               (measured_wz - s_odometry.body_wz_deg_s);
    s_odometry.last_sample_accepted = 1U;
}

void MecanumOdometry_RequestReset(void)
{
    s_reset_requested = 1U;
    __DMB();
}

void MecanumOdometry_GetSnapshot(MecanumOdometrySnapshot *snapshot)
{
    uint32_t primask;

    if (snapshot == NULL)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *snapshot = s_odometry;
    __DMB();
    if (primask == 0U)
    {
        __enable_irq();
    }
}
