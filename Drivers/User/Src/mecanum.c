#include "mecanum.h"
#include "dc_motor_ol.h"
#include "imu_service.h"
#include "mecanum_odometry.h"
#include "safety_manager.h"

#define MECANUM_IMU_STALE_TIMEOUT_MS          30U
#define MECANUM_TRAJECTORY_SPEED_EPSILON      0.001f
#define MECANUM_GYRO_SPIN_SLEW_DPS_PER_TICK   4.0f
#define MECANUM_ODOM_LINEAR_TOLERANCE_MM       5.0f
#define MECANUM_ODOM_ANGULAR_TOLERANCE_DEG     2.0f
#define MECANUM_HEADING_CONTROL_DT_S            0.010f
#define MECANUM_HEADING_KP                      1.5f
#define MECANUM_HEADING_KI                      0.05f
#define MECANUM_HEADING_DEADBAND_DEG            0.5f
#define MECANUM_HEADING_INTEGRAL_LIMIT          30.0f
#define MECANUM_HEADING_CORRECTION_LIMIT_DPS    25.0f
#define MECANUM_HEADING_SLEW_DPS_PER_TICK       2.0f
#define MECANUM_HEADING_WZ_DEADBAND_DPS         1.0f
#define MECANUM_TRAJECTORY_TIMEOUT_MARGIN_MS   5000U
#define MECANUM_TRAJECTORY_TIMEOUT_SCALE       3.0f
#define MECANUM_TRAJECTORY_TIMEOUT_MAX_MS      300000U
#define MECANUM_MAX_WHEEL_SPEED_MM_S          \
    (((float)DCMOTOR_OL_MAX_TARGET_RPM * MECANUM_WHEEL_CIRCUMFERENCE_MM) / 60.0f)

typedef struct
{
    float vx_spd;
    float vy_spd;
    float wz_spd;
    float dx_dist;
    float dy_dist;
    float dw_deg;
    uint8_t cancel_only;
} MecanumCommand_t;

static MecanumCommand_t s_pending_command;
static volatile uint8_t s_pending_command_valid = 0U;

static float s_user_vx = 0.0f;
static float s_user_vy = 0.0f;
static float s_user_wz_raw = 0.0f;
static uint8_t s_is_speed_mode = 0;

/* Encoder-odometry heading reference. A zero user Wz holds the reference;
 * non-zero Wz moves it, so translation plus intentional rotation remains a
 * valid command instead of being cancelled by heading hold. */
static uint8_t s_heading_control_active = 0U;
static float s_heading_reference_deg = 0.0f;
static float s_heading_integral = 0.0f;
static float s_heading_error_deg = 0.0f;
static float s_heading_requested_wz_dps = 0.0f;
static float s_heading_correction_wz_dps = 0.0f;
static float s_heading_output_wz_dps = 0.0f;

/* Gyro mode keeps Vx/Vy in the field frame established at enable time while
 * the chassis continuously rotates. Requested fields are written outside the
 * 10 ms ISR and consumed atomically by that ISR. */
static volatile uint8_t s_gyro_requested_enabled = 0U;
static volatile int8_t s_gyro_requested_direction = MECANUM_GYRO_DIRECTION_CW;
static volatile uint8_t s_gyro_requested_speed_percent = 0U;
static uint8_t s_gyro_active = 0U;
static float s_gyro_reference_yaw = 0.0f;
static float s_gyro_spin_dps = 0.0f;

/* Hybrid Trajectory Mode State */
static uint8_t s_hybrid_active = 0;
static uint8_t s_hybrid_bound_x = 0;
static uint8_t s_hybrid_bound_y = 0;
static uint8_t s_hybrid_bound_w = 0;
static float s_hybrid_rem_dx = 0.0f;
static float s_hybrid_rem_dy = 0.0f;
static float s_hybrid_rem_dw = 0.0f;
static float s_hybrid_tgt_vx = 0.0f;
static float s_hybrid_tgt_vy = 0.0f;
static float s_hybrid_tgt_wz = 0.0f;
static uint8_t s_hybrid_profile_complete = 0U;
static float s_hybrid_goal_dx = 0.0f;
static float s_hybrid_goal_dy = 0.0f;
static float s_hybrid_goal_dw = 0.0f;
static float s_hybrid_origin_forward_mm = 0.0f;
static float s_hybrid_origin_right_mm = 0.0f;
static float s_hybrid_origin_heading_deg = 0.0f;
static uint32_t s_hybrid_deadline_tick = 0U;

static void mecanum_clear_gyro_state(void)
{
    s_gyro_requested_enabled = 0U;
    s_gyro_requested_speed_percent = 0U;
    s_gyro_active = 0U;
    s_gyro_reference_yaw = 0.0f;
    s_gyro_spin_dps = 0.0f;
}

static float mecanum_wrap_degrees(float angle)
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

static float mecanum_get_planar_heading(const ImuServiceSnapshot_t *snapshot)
{
    uint32_t age_ms;
    float heading;

    if (snapshot == NULL)
    {
        return 0.0f;
    }

    age_ms = (uint32_t)(HAL_GetTick() - snapshot->sample_tick);
    if (age_ms > MECANUM_IMU_STALE_TIMEOUT_MS)
    {
        age_ms = MECANUM_IMU_STALE_TIMEOUT_MS;
    }

    /* Extrapolate the last sample to the current 10 ms control instant. At
     * 200 dps, one stale sample would otherwise add about two degrees of
     * field-coordinate phase lag. */
    heading = snapshot->planar_yaw +
              snapshot->planar_yaw_rate_dps * ((float)age_ms * 0.001f);
    return mecanum_wrap_degrees(heading);
}

static float mecanum_clampf(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static void mecanum_heading_control_reset(uint8_t activate)
{
    MecanumOdometrySnapshot odometry;

    MecanumOdometry_GetSnapshot(&odometry);
    s_heading_reference_deg = odometry.heading_total_deg;
    s_heading_integral = 0.0f;
    s_heading_error_deg = 0.0f;
    s_heading_requested_wz_dps = 0.0f;
    s_heading_correction_wz_dps = 0.0f;
    s_heading_output_wz_dps = 0.0f;
    s_heading_control_active = (activate != 0U) ? 1U : 0U;
}

static float mecanum_heading_control_update(float requested_wz_dps,
                                            uint8_t chassis_driven)
{
    MecanumOdometrySnapshot odometry;
    float error;
    float correction_target;
    float correction_delta;

    MecanumOdometry_GetSnapshot(&odometry);
    if (chassis_driven == 0U)
    {
        mecanum_heading_control_reset(0U);
        return 0.0f;
    }

    if (s_heading_control_active == 0U)
    {
        s_heading_reference_deg = odometry.heading_total_deg;
        s_heading_integral = 0.0f;
        s_heading_correction_wz_dps = 0.0f;
        s_heading_control_active = 1U;
    }

    if (fabsf(requested_wz_dps) < MECANUM_HEADING_WZ_DEADBAND_DPS)
    {
        requested_wz_dps = 0.0f;
    }
    else
    {
        s_heading_reference_deg += requested_wz_dps *
                                   MECANUM_HEADING_CONTROL_DT_S;
    }

    /* heading_total_deg is unwrapped, so long multi-turn commands do not
     * encounter a discontinuity at +/-180 degrees. */
    error = s_heading_reference_deg - odometry.heading_total_deg;
    if (fabsf(error) <= MECANUM_HEADING_DEADBAND_DEG)
    {
        error = 0.0f;
        s_heading_integral *= 0.90f;
    }
    else
    {
        s_heading_integral += error * MECANUM_HEADING_CONTROL_DT_S;
        s_heading_integral = mecanum_clampf(
            s_heading_integral, -MECANUM_HEADING_INTEGRAL_LIMIT,
            MECANUM_HEADING_INTEGRAL_LIMIT);
    }

    correction_target = MECANUM_HEADING_KP * error +
                        MECANUM_HEADING_KI * s_heading_integral;
    correction_target = mecanum_clampf(
        correction_target, -MECANUM_HEADING_CORRECTION_LIMIT_DPS,
        MECANUM_HEADING_CORRECTION_LIMIT_DPS);
    correction_delta = correction_target - s_heading_correction_wz_dps;
    correction_delta = mecanum_clampf(
        correction_delta, -MECANUM_HEADING_SLEW_DPS_PER_TICK,
        MECANUM_HEADING_SLEW_DPS_PER_TICK);
    s_heading_correction_wz_dps += correction_delta;

    s_heading_error_deg = error;
    s_heading_requested_wz_dps = requested_wz_dps;
    s_heading_output_wz_dps = requested_wz_dps +
                              s_heading_correction_wz_dps;
    return s_heading_output_wz_dps;
}

static float mecanum_heading_limit_to_wheel_margin(
    float requested_wz_dps, float controlled_wz_dps,
    const float requested_wheel_speed[4])
{
    float lower_mm_s = -MECANUM_MAX_WHEEL_SPEED_MM_S;
    float upper_mm_s = MECANUM_MAX_WHEEL_SPEED_MM_S;
    float rotation_mm_per_dps = MECANUM_RAD_PER_DEG *
                                MECANUM_K_ROTATION_COEFF_MM;
    float desired_correction_dps = controlled_wz_dps - requested_wz_dps;
    float lower_correction_dps;
    float upper_correction_dps;
    float applied_correction_dps;
    uint8_t i;

    for (i = 0U; i < 4U; ++i)
    {
        float lower = -MECANUM_MAX_WHEEL_SPEED_MM_S -
                      requested_wheel_speed[i];
        float upper = MECANUM_MAX_WHEEL_SPEED_MM_S -
                      requested_wheel_speed[i];
        if (lower > lower_mm_s) lower_mm_s = lower;
        if (upper < upper_mm_s) upper_mm_s = upper;
    }

    lower_correction_dps = lower_mm_s / rotation_mm_per_dps;
    upper_correction_dps = upper_mm_s / rotation_mm_per_dps;
    applied_correction_dps = mecanum_clampf(
        desired_correction_dps, lower_correction_dps,
        upper_correction_dps);

    /* Feed the actually achievable correction back into the slew state.
     * This prevents a large hidden correction from being released suddenly
     * when a saturated translation command is reduced. */
    s_heading_correction_wz_dps = applied_correction_dps;
    s_heading_output_wz_dps = requested_wz_dps + applied_correction_dps;
    return s_heading_output_wz_dps;
}

void Mecanum_GetHeadingDiagnostics(MecanumHeadingDiagnostics *diagnostics)
{
    MecanumOdometrySnapshot odometry;
    uint32_t primask;

    if (diagnostics == NULL) return;
    primask = __get_PRIMASK();
    __disable_irq();
    MecanumOdometry_GetSnapshot(&odometry);
    diagnostics->reference_deg = s_heading_reference_deg;
    diagnostics->actual_deg = odometry.heading_total_deg;
    diagnostics->error_deg = s_heading_error_deg;
    diagnostics->requested_wz_dps = s_heading_requested_wz_dps;
    diagnostics->correction_wz_dps = s_heading_correction_wz_dps;
    diagnostics->output_wz_dps = s_heading_output_wz_dps;
    diagnostics->active = s_heading_control_active;
    __DMB();
    if (primask == 0U) __enable_irq();
}

static void mecanum_reset_hybrid_state(void)
{
    s_hybrid_active = 0U;
    s_hybrid_bound_x = 0U;
    s_hybrid_bound_y = 0U;
    s_hybrid_bound_w = 0U;
    s_hybrid_rem_dx = 0.0f;
    s_hybrid_rem_dy = 0.0f;
    s_hybrid_rem_dw = 0.0f;
    s_hybrid_tgt_vx = 0.0f;
    s_hybrid_tgt_vy = 0.0f;
    s_hybrid_tgt_wz = 0.0f;
    s_hybrid_profile_complete = 0U;
    s_hybrid_goal_dx = 0.0f;
    s_hybrid_goal_dy = 0.0f;
    s_hybrid_goal_dw = 0.0f;
    s_hybrid_origin_forward_mm = 0.0f;
    s_hybrid_origin_right_mm = 0.0f;
    s_hybrid_origin_heading_deg = 0.0f;
    s_hybrid_deadline_tick = 0U;
}

static float mecanum_axis_remaining(float goal, float progress,
                                    float tolerance)
{
    float directed_progress;
    float remaining;

    directed_progress = (goal >= 0.0f) ? progress : -progress;
    remaining = fabsf(goal) - directed_progress;
    return (remaining <= tolerance) ? 0.0f : remaining;
}

static void mecanum_update_hybrid_odometry_progress(void)
{
    MecanumOdometrySnapshot odometry;
    float progress;

    MecanumOdometry_GetSnapshot(&odometry);
    if (odometry.last_sample_accepted != 0U)
    {
        if (s_hybrid_bound_x != 0U)
        {
            progress = odometry.body_forward_total_mm -
                       s_hybrid_origin_forward_mm;
            s_hybrid_rem_dx = mecanum_axis_remaining(
                s_hybrid_goal_dx, progress, MECANUM_ODOM_LINEAR_TOLERANCE_MM);
        }
        if (s_hybrid_bound_y != 0U)
        {
            progress = odometry.body_right_total_mm - s_hybrid_origin_right_mm;
            s_hybrid_rem_dy = mecanum_axis_remaining(
                s_hybrid_goal_dy, progress, MECANUM_ODOM_LINEAR_TOLERANCE_MM);
        }
        if (s_hybrid_bound_w != 0U)
        {
            progress = odometry.heading_total_deg - s_hybrid_origin_heading_deg;
            s_hybrid_rem_dw = mecanum_axis_remaining(
                s_hybrid_goal_dw, progress, MECANUM_ODOM_ANGULAR_TOLERANCE_DEG);
        }
    }

    if (((s_hybrid_bound_x == 0U) || (s_hybrid_rem_dx <= 0.0f)) &&
        ((s_hybrid_bound_y == 0U) || (s_hybrid_rem_dy <= 0.0f)) &&
        ((s_hybrid_bound_w == 0U) || (s_hybrid_rem_dw <= 0.0f)))
    {
        s_hybrid_profile_complete = 1U;
    }

    /* A disconnected encoder or a mechanically blocked chassis must not be
     * driven forever. The deadline is deliberately much longer than the
     * nominal trajectory and is not used by continuous speed mode. */
    if ((s_hybrid_deadline_tick != 0U) &&
        ((int32_t)(HAL_GetTick() - s_hybrid_deadline_tick) >= 0))
    {
        s_hybrid_profile_complete = 1U;
    }
}

static uint32_t mecanum_calculate_trajectory_timeout_ms(float dx, float dy,
                                                         float dw,
                                                         float vx, float vy,
                                                         float wz)
{
    float duration_s = 0.0f;
    float axis_duration;
    float timeout_ms;

    if ((dx != 0.0f) && (fabsf(vx) > MECANUM_TRAJECTORY_SPEED_EPSILON))
    {
        duration_s = fabsf(dx / vx);
    }
    if ((dy != 0.0f) && (fabsf(vy) > MECANUM_TRAJECTORY_SPEED_EPSILON))
    {
        axis_duration = fabsf(dy / vy);
        if (axis_duration > duration_s) duration_s = axis_duration;
    }
    if ((dw != 0.0f) && (fabsf(wz) > MECANUM_TRAJECTORY_SPEED_EPSILON))
    {
        axis_duration = fabsf(dw / wz);
        if (axis_duration > duration_s) duration_s = axis_duration;
    }

    timeout_ms = duration_s * 1000.0f * MECANUM_TRAJECTORY_TIMEOUT_SCALE +
                 (float)MECANUM_TRAJECTORY_TIMEOUT_MARGIN_MS;
    if (timeout_ms > (float)MECANUM_TRAJECTORY_TIMEOUT_MAX_MS)
    {
        timeout_ms = (float)MECANUM_TRAJECTORY_TIMEOUT_MAX_MS;
    }
    return (uint32_t)timeout_ms;
}

static void mecanum_body_to_wheels(float *vx, float *vy, float *wz, float wheel_speed[4])
{
    float wz_comp_spd = (*wz) * MECANUM_RAD_PER_DEG * MECANUM_K_ROTATION_COEFF_MM;
    float max_abs_speed = 0.0f;
    float scale = 1.0f;
    uint8_t i;

    wheel_speed[0] =  *vx + *vy + wz_comp_spd;
    wheel_speed[1] = -*vx + *vy + wz_comp_spd;
    wheel_speed[2] = -*vx - *vy + wz_comp_spd;
    wheel_speed[3] =  *vx - *vy + wz_comp_spd;

    for (i = 0U; i < 4U; ++i)
    {
        float abs_speed = fabsf(wheel_speed[i]);
        if (abs_speed > max_abs_speed)
        {
            max_abs_speed = abs_speed;
        }
    }

    /* Preserve the kinematic ratio. Per-wheel clipping changes the requested
     * direction whenever translation and rotation are combined. */
    if (max_abs_speed > MECANUM_MAX_WHEEL_SPEED_MM_S)
    {
        scale = MECANUM_MAX_WHEEL_SPEED_MM_S / max_abs_speed;
        *vx *= scale;
        *vy *= scale;
        *wz *= scale;
        for (i = 0U; i < 4U; ++i)
        {
            wheel_speed[i] *= scale;
        }
    }
}

static void mecanum_apply_wheel_speeds(const float wheel_speed[4])
{
    Mecanum_HW_SetSpeed(MECANUM_MOTOR_FL, wheel_speed[0]);
    Mecanum_HW_SetSpeed(MECANUM_MOTOR_FR, wheel_speed[1]);
    Mecanum_HW_SetSpeed(MECANUM_MOTOR_RR, wheel_speed[2]);
    Mecanum_HW_SetSpeed(MECANUM_MOTOR_RL, wheel_speed[3]);
}


/* 
 * 纭欢鍥炶皟瀹炵幇
 * 灏嗚繍鍔ㄥ绠楁硶璁＄畻鍑虹殑杞绾块€熷害/璺濈鎸囦护锛屼笅鍙戠粰瀹為檯鐨勫簳灞傜數鏈烘帶鍒跺櫒 (DC Motor)
 */
void Mecanum_HW_SetSpeed(uint8_t motor_id, float speed_val) {
    /* 1. 灏嗚疆绔墍闇€鐨勭嚎閫熷害 (mm/s) 杞崲涓虹數鏈鸿浆閫?(RPM)
     *    鍏紡: RPM = 閫熷害(mm/s) * 60(绉? / 楹﹁疆鍛ㄩ暱(mm) 
     */
    float target_rpm = (speed_val * 60.0f) / MECANUM_WHEEL_CIRCUMFERENCE_MM;

    /* 2. 鍦ㄧ洰鍓嶇殑 dc_motor 鎺у埗鍣ㄤ腑锛岃緭鍏ョ殑鏄€熷害鐨勭櫨鍒嗘瘮 (-100 鍒?100)锛?
     *    涓?100% 瀵瑰簲 DCMOTOR_OL_MAX_TARGET_RPM (瀹忓畾涔変负 300 RPM)銆?
     */
    float percent_f = (target_rpm * 100.0f) / (float)DCMOTOR_OL_MAX_TARGET_RPM;

    /* 3. 闄愬箙澶勭悊骞朵笅鍙戝簳灞傛帴鍙?*/
    if (percent_f > 100.0f)  percent_f = 100.0f;
    if (percent_f < -100.0f) percent_f = -100.0f;
    
    DCMotor_OL_SetSpeed(motor_id, (int16_t)percent_f);
}

void Mecanum_HW_SetDistance(uint8_t motor_id, float dist_val, float speed_val) {
    /* 1. 灏嗚疆绔墍闇€鐨勪綅绉?(mm) 杞崲涓虹數鏈虹紪鐮佸櫒鑴夊啿鏁?
     *    鍏紡: Pulses = (璺濈(mm) / 楹﹁疆鍛ㄩ暱(mm)) * 杞竴鍦堢殑鑴夊啿鏁?
     */
    float pulses = (dist_val / MECANUM_WHEEL_CIRCUMFERENCE_MM) * DCMOTOR_OL_ENCODER_COUNTS_PER_REV;

    /* 2. 鑾峰彇褰撳墠鐢垫満浣嶇疆锛屽苟璁＄畻鐩爣鑴夊啿鍊?*/
    int64_t current_pulses = DCMotor_OL_GetPositionPulses(motor_id);
    int64_t target_pulses = current_pulses + (int64_t)pulses;

    /* 3. 灏嗚疆绔墍闇€鐨勭嚎閫熷害 (mm/s) 杞崲涓洪€熷害鐧惧垎姣?*/
    float target_rpm = (speed_val * 60.0f) / MECANUM_WHEEL_CIRCUMFERENCE_MM;
    float percent_f = (target_rpm * 100.0f) / (float)DCMOTOR_OL_MAX_TARGET_RPM;
    
    if (percent_f < 0.0f) percent_f = -percent_f;
    if (percent_f > 100.0f) percent_f = 100.0f;

    /* 4. 涓嬪彂鐩爣缁欏簳灞傛帶鍒跺櫒 */
    DCMotor_OL_SetTargetPosition(motor_id, target_pulses, (int16_t)percent_f);
}

/* =================================================================================
 * Mecanum_MixedControl
 * [ 缁煎悎鎺у埗涓诲弬鏁板垎閰嶇畻娉?]
 * 
 * 鎺ㄥ锛?
 * 璁惧簳鐩樻爣鍑嗗潗鏍囩郴锛氬悜鍓嶄负X姝ｏ紝鍚戝彸涓篩姝ｏ紝椤烘椂閽?淇)涓鸿嚜鏃媁姝ｃ€?
 * 绾溅杞垏鍚戦€熷害锛?vL_x = Vx + Vy + W_rad * K
 * 鏍规嵁鐢ㄦ埛瀹氫箟鐨勭墿鐞嗚浆鍚戯細1/4 鐢垫満姝ｈ浆灏忚溅寰€鍓嶈繘锛?/3 鐢垫満鍙嶈浆灏忚溅寰€鍓嶈繘銆?
 * 鏁呰溅杞€熷害锛?
 * 銆愬乏鍓?锛氱數鏈烘杞紝鐩存帴璺熼殢鍓嶈繘銆?M1 = +(Vx + Vy + W * K)
 * 銆愬彸鍓?锛氱數鏈哄弽杞紝琛ュ伩鎴愯礋鏁般€?  M2 = -(Vx - Vy - W * K) = -Vx + Vy + W * K
 * 銆愬彸鍚?锛氱數鏈哄弽杞紝琛ュ伩鎴愯礋鏁般€?  M3 = -(Vx + Vy - W * K) = -Vx - Vy + W * K
 * 銆愬乏鍚?锛氱數鏈烘杞紝鐩存帴璺熼殢鍓嶈繘銆?M4 = +(Vx - Vy + W * K)
 * 楠岃瘉锛?
 * 1. 绾墠杩?Vx>0): M1=+Vx, M2=-Vx, M3=-Vx, M4=+Vx  -> 婊¤冻鐢ㄦ埛瑙勫畾锛氬墠杩涙椂14姝ｈ浆锛?3鍙嶈浆銆?
 * 2. 绾彸绉?Vy>0): M1=+Vy, M2=+Vy, M3=-Vy, M4=-Vy  -> 杩欏浜庨害杞殑X鍨嬪畨瑁呮垨瀵归《瀹夎涓烘爣鍑嗙殑鍙冲钩绉诲舰寮忋€?
 * 3. 绾『杞?Wz>0): M1=+Wz, M2=+Wz, M3=+Wz, M4=+Wz  -> 鍥涜疆鍏ㄩ儴姝ｈ浆锛屽乏渚у悜鍓嶏紝鍙充晶鍚戝悗锛屽皬杞﹁揪鎴愬師鍦伴『鏃堕拡鏃嬭浆銆?
 * 
 * 姝ゅ叕寮忎篃鍚屾椂鏀寔缁撳悎浣跨敤銆傛瘮濡傚墠杩涗笖瑕佸彸杞?寮х嚎): 杈撳叆Vx 鍜?Wz銆?
 *=================================================================================*/
static void mecanum_apply_command(float vx_spd, float vy_spd, float wz_spd,
                                  float dx_dist, float dy_dist, float dw_deg) {
    /* 1. Determine Control Mode (Speed vs Hybrid/Distance) */
    int has_dist = (dx_dist != 0.0f || dy_dist != 0.0f || dw_deg != 0.0f);
    uint8_t was_speed_mode = s_is_speed_mode;
    uint8_t was_hybrid_active = s_hybrid_active;
    
    if (!has_dist) {
        // Pure speed control mode
        mecanum_reset_hybrid_state();
        s_user_vx = vx_spd;
        s_user_vy = vy_spd;
        s_user_wz_raw = wz_spd;
        s_is_speed_mode = 1;
        s_hybrid_active = 0;
        if (((was_speed_mode == 0U) || (was_hybrid_active != 0U)) &&
            (s_gyro_requested_enabled == 0U) &&
            ((fabsf(vx_spd) > MECANUM_TRAJECTORY_SPEED_EPSILON) ||
             (fabsf(vy_spd) > MECANUM_TRAJECTORY_SPEED_EPSILON) ||
             (fabsf(wz_spd) > MECANUM_TRAJECTORY_SPEED_EPSILON)))
        {
            mecanum_heading_control_reset(1U);
        }
    } else {
        MecanumOdometrySnapshot odometry;
        uint32_t timeout_ms;

        // Hybrid trajectory generation mode
        mecanum_clear_gyro_state();
        s_is_speed_mode = 0;
        s_hybrid_active = 1;
        s_hybrid_profile_complete = 0U;
        
        // Save bounds for axes with non-zero distance commands
        s_hybrid_bound_x = (dx_dist != 0.0f);
        s_hybrid_bound_y = (dy_dist != 0.0f);
        s_hybrid_bound_w = (dw_deg != 0.0f);
        
        s_hybrid_rem_dx = fabs(dx_dist);
        s_hybrid_rem_dy = fabs(dy_dist);
        s_hybrid_rem_dw = fabs(dw_deg);
        s_hybrid_goal_dx = dx_dist;
        s_hybrid_goal_dy = dy_dist;
        s_hybrid_goal_dw = dw_deg;

        MecanumOdometry_GetSnapshot(&odometry);
        s_hybrid_origin_forward_mm = odometry.body_forward_total_mm;
        s_hybrid_origin_right_mm = odometry.body_right_total_mm;
        s_hybrid_origin_heading_deg = odometry.heading_total_deg;
        /* Every bounded trajectory owns a fresh relative pose command. */
        mecanum_heading_control_reset(1U);
        
        // Decide trajectory direction and raw velocity per axis.
        // Axes with distance bounds use the signed speed as the travel rate.
        // Axes without distance bounds keep their speed offset for this command.
        s_hybrid_tgt_vx = s_hybrid_bound_x ? ((dx_dist > 0.0f) ? fabs(vx_spd) : -fabs(vx_spd)) : vx_spd;
        s_hybrid_tgt_vy = s_hybrid_bound_y ? ((dy_dist > 0.0f) ? fabs(vy_spd) : -fabs(vy_spd)) : vy_spd;
        s_hybrid_tgt_wz = s_hybrid_bound_w ? ((dw_deg > 0.0f) ? fabs(wz_spd) : -fabs(wz_spd)) : wz_spd;

        if ((s_hybrid_bound_x && (fabsf(vx_spd) < MECANUM_TRAJECTORY_SPEED_EPSILON)) ||
            (s_hybrid_bound_y && (fabsf(vy_spd) < MECANUM_TRAJECTORY_SPEED_EPSILON)) ||
            (s_hybrid_bound_w && (fabsf(wz_spd) < MECANUM_TRAJECTORY_SPEED_EPSILON)))
        {
            mecanum_reset_hybrid_state();
            s_user_vx = 0.0f;
            s_user_vy = 0.0f;
            s_user_wz_raw = 0.0f;
            DCMotor_OL_StopAll();
            return;
        }

        timeout_ms = mecanum_calculate_trajectory_timeout_ms(
            dx_dist, dy_dist, dw_deg, s_hybrid_tgt_vx,
            s_hybrid_tgt_vy, s_hybrid_tgt_wz);
        s_hybrid_deadline_tick = HAL_GetTick() + timeout_ms;

    }
}

void Mecanum_MixedControl(float vx_spd, float vy_spd, float wz_spd,
                          float dx_dist, float dy_dist, float dw_deg) {
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    s_pending_command.vx_spd = vx_spd;
    s_pending_command.vy_spd = vy_spd;
    s_pending_command.wz_spd = wz_spd;
    s_pending_command.dx_dist = dx_dist;
    s_pending_command.dy_dist = dy_dist;
    s_pending_command.dw_deg = dw_deg;
    s_pending_command.cancel_only = 0U;
    __DMB();
    s_pending_command_valid = 1U;
    if (primask == 0U) {
        __enable_irq();
    }
}

uint8_t Mecanum_GyroEnable(int8_t direction, uint8_t speed_percent)
{
    uint32_t primask;

    if (((direction != MECANUM_GYRO_DIRECTION_CW) &&
         (direction != MECANUM_GYRO_DIRECTION_CCW)) ||
        (speed_percent > 100U))
    {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    s_gyro_requested_direction = direction;
    s_gyro_requested_speed_percent = speed_percent;
    __DMB();
    s_gyro_requested_enabled = 1U;
    if (primask == 0U)
    {
        __enable_irq();
    }

    return 1U;
}

void Mecanum_GyroDisable(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    s_gyro_requested_enabled = 0U;
    s_gyro_requested_speed_percent = 0U;
    __DMB();
    if (primask == 0U)
    {
        __enable_irq();
    }
}

uint8_t Mecanum_IsGyroModeEnabled(void)
{
    return s_gyro_requested_enabled;
}

int8_t Mecanum_GetGyroDirection(void)
{
    return s_gyro_requested_direction;
}

uint8_t Mecanum_GetGyroSpeedPercent(void)
{
    return s_gyro_requested_speed_percent;
}

float Mecanum_GetGyroSpinDps(void)
{
    return (float)s_gyro_requested_direction *
           ((float)s_gyro_requested_speed_percent / 100.0f) *
           MECANUM_GYRO_BASE_SPIN_DPS;
}

void Mecanum_CancelControl(void) {
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    s_pending_command.cancel_only = 1U;
    __DMB();
    s_pending_command_valid = 1U;
    if (primask == 0U) {
        __enable_irq();
    }
}

void Mecanum_EmergencyStop(void) {
    s_pending_command_valid = 0U;
    mecanum_clear_gyro_state();
    s_user_vx = 0.0f;
    s_user_vy = 0.0f;
    s_user_wz_raw = 0.0f;
    s_is_speed_mode = 0U;
    mecanum_reset_hybrid_state();
    mecanum_heading_control_reset(0U);
    DCMotor_OL_StopAll();
}

uint8_t Mecanum_IsMotionActive(void) {
    if ((s_hybrid_active != 0U) ||
        (s_gyro_requested_enabled != 0U) ||
        (s_gyro_active != 0U)) {
        return 1U;
    }

    return ((s_user_vx != 0.0f) || (s_user_vy != 0.0f) || (s_user_wz_raw != 0.0f)) ? 1U : 0U;
}

/* =================================================================================
 * 鍗曠嫭鍓ョ骞跺皝瑁呭嚭鐨勭嫭绔嬫帶鍒惰皟鐢ㄦ帴鍙?
 * =================================================================================*/

void Mecanum_Translate_Forward(float speed, float dist) {
    Mecanum_MixedControl(speed, 0.0f, 0.0f, dist, 0.0f, 0.0f);
}

void Mecanum_Translate_Backward(float speed, float dist) {
    Mecanum_MixedControl(-speed, 0.0f, 0.0f, -dist, 0.0f, 0.0f);
}

void Mecanum_Translate_Right(float speed, float dist) {
    Mecanum_MixedControl(0.0f, speed, 0.0f, 0.0f, dist, 0.0f);
}

void Mecanum_Translate_Left(float speed, float dist) {
    Mecanum_MixedControl(0.0f, -speed, 0.0f, 0.0f, -dist, 0.0f);
}

void Mecanum_Rotate_CW(float speed, float dist) {
    Mecanum_MixedControl(0.0f, 0.0f, speed, 0.0f, 0.0f, dist);
}

void Mecanum_Rotate_CCW(float speed, float dist) {
    Mecanum_MixedControl(0.0f, 0.0f, -speed, 0.0f, 0.0f, -dist);
}

void Mecanum_Tick10ms(void) {
    ImuServiceSnapshot_t imu_snapshot;
    float planar_heading = 0.0f;
    uint8_t has_imu;
    uint8_t imu_current;

    if (s_pending_command_valid != 0U) {
        MecanumCommand_t command = s_pending_command;
        s_pending_command_valid = 0U;

        if (command.cancel_only != 0U) {
            mecanum_clear_gyro_state();
            s_user_vx = 0.0f;
            s_user_vy = 0.0f;
            s_user_wz_raw = 0.0f;
            s_is_speed_mode = 0U;
            mecanum_reset_hybrid_state();
            mecanum_heading_control_reset(0U);
        } else {
            mecanum_apply_command(command.vx_spd, command.vy_spd, command.wz_spd,
                                  command.dx_dist, command.dy_dist, command.dw_deg);
        }
    }

    has_imu = IMU_Service_GetSnapshot(&imu_snapshot);
    if (has_imu != 0U) {
        planar_heading = mecanum_get_planar_heading(&imu_snapshot);
    }
    imu_current = ((has_imu != 0U) &&
        ((uint32_t)(HAL_GetTick() - imu_snapshot.sample_tick) <=
         MECANUM_IMU_STALE_TIMEOUT_MS)) ? 1U : 0U;
    Safety_SetWarning(SAFETY_WARNING_IMU_DEGRADED,
                      (imu_current == 0U) ? 1U : 0U);

    if (s_gyro_requested_enabled != 0U) {
        float requested_spin_dps;

        if ((s_gyro_active == 0U) && (has_imu != 0U)) {
            s_gyro_reference_yaw = planar_heading;
            s_gyro_active = 1U;
            s_is_speed_mode = 1U;
            s_user_wz_raw = 0.0f;
            mecanum_reset_hybrid_state();
            mecanum_heading_control_reset(0U);
        }

        requested_spin_dps = (float)s_gyro_requested_direction *
                             ((float)s_gyro_requested_speed_percent / 100.0f) *
                             MECANUM_GYRO_BASE_SPIN_DPS;
        if (s_gyro_spin_dps < (requested_spin_dps - MECANUM_GYRO_SPIN_SLEW_DPS_PER_TICK))
        {
            s_gyro_spin_dps += MECANUM_GYRO_SPIN_SLEW_DPS_PER_TICK;
        }
        else if (s_gyro_spin_dps > (requested_spin_dps + MECANUM_GYRO_SPIN_SLEW_DPS_PER_TICK))
        {
            s_gyro_spin_dps -= MECANUM_GYRO_SPIN_SLEW_DPS_PER_TICK;
        }
        else
        {
            s_gyro_spin_dps = requested_spin_dps;
        }
    } else if (s_gyro_active != 0U) {
        /* Coordinate frames change when gyro mode is disabled. Stop first so
         * an old field-frame Vx/Vy cannot become an unexpected body command. */
        mecanum_clear_gyro_state();
        s_user_vx = 0.0f;
        s_user_vy = 0.0f;
        s_user_wz_raw = 0.0f;
        s_is_speed_mode = 0U;
        mecanum_reset_hybrid_state();
        mecanum_heading_control_reset(0U);
        DCMotor_OL_StopAll();
    }

    /* IMU is mandatory only for field-coordinate gyro mode.  Plain wheel,
     * body-frame translation and distance control degrade to open loop and
     * keep running while exposing a warning. */
    if (((s_gyro_requested_enabled != 0U) || (s_gyro_active != 0U)) &&
        (imu_current == 0U)) {
        Mecanum_EmergencyStop();
        return;
    }
    
    if (!s_is_speed_mode) {
        if (s_hybrid_active) {
            const float dt = 0.010f;

            mecanum_update_hybrid_odometry_progress();
            if (s_hybrid_profile_complete != 0U) {
                /* The last commanded 10 ms interval has now elapsed. Stop all
                 * wheels together; independent wheel position correction can
                 * leave one wheel driving the chassis after the path is done. */
                DCMotor_OL_StopAll();
                mecanum_reset_hybrid_state();
                s_user_vx = 0.0f;
                s_user_vy = 0.0f;
                s_user_wz_raw = 0.0f;
            } else {
                float step_vx = s_hybrid_tgt_vx;
                float step_vy = s_hybrid_tgt_vy;
                float step_wz = s_hybrid_tgt_wz;
                float wheel_speed[4];
                float dist_step;
                uint8_t chassis_driven;

                if (s_hybrid_bound_x && s_hybrid_rem_dx > 0.0f) {
                    dist_step = fabsf(step_vx) * dt;
                    if (dist_step >= s_hybrid_rem_dx) {
                        step_vx = (step_vx >= 0.0f) ? (s_hybrid_rem_dx / dt)
                                                    : -(s_hybrid_rem_dx / dt);
                    }
                } else if (s_hybrid_bound_x) {
                    step_vx = 0.0f;
                }

                if (s_hybrid_bound_y && s_hybrid_rem_dy > 0.0f) {
                    dist_step = fabsf(step_vy) * dt;
                    if (dist_step >= s_hybrid_rem_dy) {
                        step_vy = (step_vy >= 0.0f) ? (s_hybrid_rem_dy / dt)
                                                    : -(s_hybrid_rem_dy / dt);
                    }
                } else if (s_hybrid_bound_y) {
                    step_vy = 0.0f;
                }

                if (s_hybrid_bound_w && s_hybrid_rem_dw > 0.0f) {
                    dist_step = fabsf(step_wz) * dt;
                    if (dist_step >= s_hybrid_rem_dw) {
                        step_wz = (step_wz >= 0.0f) ? (s_hybrid_rem_dw / dt)
                                                    : -(s_hybrid_rem_dw / dt);
                    }
                } else if (s_hybrid_bound_w) {
                    step_wz = 0.0f;
                }

                /* Limit the complete body vector before consuming remaining
                 * distance. The heading reference advances by this feasible
                 * user Wz, not by an unattainable pre-saturation command. */
                mecanum_body_to_wheels(&step_vx, &step_vy, &step_wz, wheel_speed);

                chassis_driven = ((fabsf(step_vx) > MECANUM_TRAJECTORY_SPEED_EPSILON) ||
                                  (fabsf(step_vy) > MECANUM_TRAJECTORY_SPEED_EPSILON) ||
                                  (fabsf(step_wz) > MECANUM_TRAJECTORY_SPEED_EPSILON)) ? 1U : 0U;
                step_wz = mecanum_heading_control_update(step_wz,
                                                         chassis_driven);
                step_wz = mecanum_heading_limit_to_wheel_margin(
                    s_heading_requested_wz_dps, step_wz, wheel_speed);
                /* Recalculate after adding the limited heading correction. */
                mecanum_body_to_wheels(&step_vx, &step_vy, &step_wz, wheel_speed);

                /* Track the trajectory with the closed speed loops. */
                mecanum_apply_wheel_speeds(wheel_speed);
            }
        } else {
            mecanum_heading_control_reset(0U);
        }
        return;
    }
    
    float final_vx = s_user_vx;
    float final_vy = s_user_vy;
    float final_wz = s_user_wz_raw;

    if (s_gyro_active != 0U) {
        float relative_yaw = mecanum_wrap_degrees(
            planar_heading - s_gyro_reference_yaw);
        float yaw_rad;
        float cos_yaw;
        float sin_yaw;

        yaw_rad = relative_yaw * MECANUM_RAD_PER_DEG;
        cos_yaw = cosf(yaw_rad);
        sin_yaw = sinf(yaw_rad);

        /* Rotate the fixed-frame command back into the body frame. Both the
         * planar heading and mecanum Wz use positive-clockwise convention. */
        final_vx = cos_yaw * s_user_vx + sin_yaw * s_user_vy;
        final_vy = -sin_yaw * s_user_vx + cos_yaw * s_user_vy;
        final_wz = s_gyro_spin_dps;
        mecanum_heading_control_reset(0U);
    } else {
        uint8_t chassis_driven;
        float preview_wheel_speed[4];

        /* First enforce the wheel-speed envelope on the user's vector. This
         * keeps a sustained saturated joystick command from moving the
         * heading reference faster than the chassis can physically rotate. */
        mecanum_body_to_wheels(&final_vx, &final_vy, &final_wz,
                               preview_wheel_speed);
        chassis_driven = ((fabsf(final_vx) > MECANUM_TRAJECTORY_SPEED_EPSILON) ||
                          (fabsf(final_vy) > MECANUM_TRAJECTORY_SPEED_EPSILON) ||
                          (fabsf(final_wz) > MECANUM_TRAJECTORY_SPEED_EPSILON)) ? 1U : 0U;
        final_wz = mecanum_heading_control_update(final_wz, chassis_driven);
        final_wz = mecanum_heading_limit_to_wheel_margin(
            s_heading_requested_wz_dps, final_wz, preview_wheel_speed);
    }
    
    float wheel_speed[4];

    mecanum_body_to_wheels(&final_vx, &final_vy, &final_wz, wheel_speed);
    mecanum_apply_wheel_speeds(wheel_speed);
}
