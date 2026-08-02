#include "mecanum.h"
#include "dc_motor_ol.h"
#include "imu_service.h"
#include "safety_manager.h"

#define MECANUM_IMU_STALE_TIMEOUT_MS          30U
#define MECANUM_POSITION_TOLERANCE_PULSES     20LL
#define MECANUM_SETTLE_SPEED_RPM              12L
#define MECANUM_SETTLE_REQUIRED_CYCLES        5U
#define MECANUM_TRAJECTORY_SPEED_EPSILON      0.001f

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

static uint8_t s_angle_closed_loop_en = 1;
static float s_target_yaw = 0.0f;
static float s_yaw_integral = 0.0f;

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
static uint8_t s_hybrid_settle_count = 0U;
static int64_t s_hybrid_start_pulses[4] = {0, 0, 0, 0};
static float s_hybrid_wheel_offset_pulses[4] = {0.0f, 0.0f, 0.0f, 0.0f};
static int64_t s_hybrid_target_pulses[4] = {0, 0, 0, 0};

static int64_t mecanum_round_pulses(float pulses)
{
    return (pulses >= 0.0f) ? (int64_t)(pulses + 0.5f) : (int64_t)(pulses - 0.5f);
}

static int64_t mecanum_abs_i64(int64_t value)
{
    return (value < 0) ? -value : value;
}

static int32_t mecanum_abs_i32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static void mecanum_reset_hybrid_state(void)
{
    uint8_t i;

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
    s_hybrid_settle_count = 0U;

    for (i = 0U; i < 4U; ++i)
    {
        s_hybrid_start_pulses[i] = 0;
        s_hybrid_wheel_offset_pulses[i] = 0.0f;
        s_hybrid_target_pulses[i] = 0;
    }
}

static void mecanum_submit_hybrid_targets(void)
{
    uint8_t i;

    for (i = 0U; i < 4U; ++i)
    {
        s_hybrid_target_pulses[i] =
            s_hybrid_start_pulses[i] + mecanum_round_pulses(s_hybrid_wheel_offset_pulses[i]);
        DCMotor_OL_SetTargetPosition((uint8_t)(i + 1U), s_hybrid_target_pulses[i], 100);
    }
}

static uint8_t mecanum_hybrid_wheels_settled(void)
{
    uint8_t i;

    for (i = 0U; i < 4U; ++i)
    {
        int64_t position_error = s_hybrid_target_pulses[i] -
                                 DCMotor_OL_GetPositionPulses((uint8_t)(i + 1U));
        int32_t actual_rpm = DCMotor_OL_GetSpeedRpm((uint8_t)(i + 1U));

        if ((mecanum_abs_i64(position_error) > MECANUM_POSITION_TOLERANCE_PULSES) ||
            (mecanum_abs_i32(actual_rpm) > MECANUM_SETTLE_SPEED_RPM))
        {
            return 0U;
        }
    }

    return 1U;
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
    
    if (!has_dist) {
        // Pure speed control mode
        mecanum_reset_hybrid_state();
        s_user_vx = vx_spd;
        s_user_vy = vy_spd;
        s_user_wz_raw = wz_spd;
        s_is_speed_mode = 1;
        s_hybrid_active = 0;
    } else {
        // Hybrid trajectory generation mode
        s_is_speed_mode = 0;
        s_hybrid_active = 1;
        s_hybrid_profile_complete = 0U;
        s_hybrid_settle_count = 0U;
        
        // Save bounds for axes with non-zero distance commands
        s_hybrid_bound_x = (dx_dist != 0.0f);
        s_hybrid_bound_y = (dy_dist != 0.0f);
        s_hybrid_bound_w = (dw_deg != 0.0f);
        
        s_hybrid_rem_dx = fabs(dx_dist);
        s_hybrid_rem_dy = fabs(dy_dist);
        s_hybrid_rem_dw = fabs(dw_deg);
        
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

        // Keep absolute encoder counts in int64 and integrate only relative float offsets.
        for (uint8_t i = 0U; i < 4U; ++i)
        {
            s_hybrid_start_pulses[i] = DCMotor_OL_GetPositionPulses((uint8_t)(i + 1U));
            s_hybrid_wheel_offset_pulses[i] = 0.0f;
            s_hybrid_target_pulses[i] = s_hybrid_start_pulses[i];
        }
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
    s_user_vx = 0.0f;
    s_user_vy = 0.0f;
    s_user_wz_raw = 0.0f;
    s_is_speed_mode = 0U;
    mecanum_reset_hybrid_state();
    s_yaw_integral = 0.0f;
    DCMotor_OL_StopAll();
}

uint8_t Mecanum_IsMotionActive(void) {
    if (s_hybrid_active != 0U) {
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
    eulerian_angles_t angles = {0.0f, 0.0f, 0.0f};
    uint8_t has_imu;

    if (s_pending_command_valid != 0U) {
        MecanumCommand_t command = s_pending_command;
        s_pending_command_valid = 0U;

        if (command.cancel_only != 0U) {
            s_user_vx = 0.0f;
            s_user_vy = 0.0f;
            s_user_wz_raw = 0.0f;
            s_is_speed_mode = 0U;
            mecanum_reset_hybrid_state();
            s_yaw_integral = 0.0f;
        } else {
            mecanum_apply_command(command.vx_spd, command.vy_spd, command.wz_spd,
                                  command.dx_dist, command.dy_dist, command.dw_deg);
        }
    }

    has_imu = IMU_Service_GetSnapshot(&imu_snapshot);
    if (has_imu != 0U) {
        angles = imu_snapshot.angles;
    }

    if (Mecanum_IsMotionActive() != 0U) {
        if ((has_imu == 0U) ||
            ((uint32_t)(HAL_GetTick() - imu_snapshot.sample_tick) > MECANUM_IMU_STALE_TIMEOUT_MS)) {
            Safety_LatchFault(SAFETY_FAULT_IMU_STALE);
            Mecanum_EmergencyStop();
            return;
        }
    }
    
    if (!s_is_speed_mode) {
        if (s_hybrid_active) {
            const float dt = 0.010f;

            if (s_hybrid_profile_complete == 0U) {
                float step_vx = s_hybrid_tgt_vx;
                float step_vy = s_hybrid_tgt_vy;
                float step_wz = s_hybrid_tgt_wz;
                float wz_comp_spd;
                float k_pulses;

                if (s_hybrid_bound_x && s_hybrid_rem_dx > 0.0f) {
                    float dist_step = fabsf(step_vx) * dt;
                    if (dist_step >= s_hybrid_rem_dx) {
                        dist_step = s_hybrid_rem_dx;
                        step_vx = (step_vx >= 0.0f) ? (dist_step / dt) : -(dist_step / dt);
                    }
                    s_hybrid_rem_dx -= dist_step;
                    if (s_hybrid_rem_dx <= MECANUM_TRAJECTORY_SPEED_EPSILON) {
                        s_hybrid_rem_dx = 0.0f;
                        s_hybrid_tgt_vx = 0.0f;
                    }
                }

                if (s_hybrid_bound_y && s_hybrid_rem_dy > 0.0f) {
                    float dist_step = fabsf(step_vy) * dt;
                    if (dist_step >= s_hybrid_rem_dy) {
                        dist_step = s_hybrid_rem_dy;
                        step_vy = (step_vy >= 0.0f) ? (dist_step / dt) : -(dist_step / dt);
                    }
                    s_hybrid_rem_dy -= dist_step;
                    if (s_hybrid_rem_dy <= MECANUM_TRAJECTORY_SPEED_EPSILON) {
                        s_hybrid_rem_dy = 0.0f;
                        s_hybrid_tgt_vy = 0.0f;
                    }
                }

                if (s_hybrid_bound_w && s_hybrid_rem_dw > 0.0f) {
                    float angle_step = fabsf(step_wz) * dt;
                    if (angle_step >= s_hybrid_rem_dw) {
                        angle_step = s_hybrid_rem_dw;
                        step_wz = (step_wz >= 0.0f) ? (angle_step / dt) : -(angle_step / dt);
                    }
                    s_hybrid_rem_dw -= angle_step;
                    if (s_hybrid_rem_dw <= MECANUM_TRAJECTORY_SPEED_EPSILON) {
                        s_hybrid_rem_dw = 0.0f;
                        s_hybrid_tgt_wz = 0.0f;
                    }
                }

                wz_comp_spd = step_wz * MECANUM_RAD_PER_DEG * MECANUM_K_ROTATION_COEFF_MM;
                k_pulses = (dt / MECANUM_WHEEL_CIRCUMFERENCE_MM) *
                           (float)DCMOTOR_OL_ENCODER_COUNTS_PER_REV;

                s_hybrid_wheel_offset_pulses[0] += ( step_vx + step_vy + wz_comp_spd) * k_pulses;
                s_hybrid_wheel_offset_pulses[1] += (-step_vx + step_vy + wz_comp_spd) * k_pulses;
                s_hybrid_wheel_offset_pulses[2] += (-step_vx - step_vy + wz_comp_spd) * k_pulses;
                s_hybrid_wheel_offset_pulses[3] += ( step_vx - step_vy + wz_comp_spd) * k_pulses;

                if (((!s_hybrid_bound_x) || (s_hybrid_rem_dx <= 0.0f)) &&
                    ((!s_hybrid_bound_y) || (s_hybrid_rem_dy <= 0.0f)) &&
                    ((!s_hybrid_bound_w) || (s_hybrid_rem_dw <= 0.0f)))
                {
                    s_hybrid_profile_complete = 1U;
                    s_hybrid_settle_count = 0U;
                }
            }

            /* Keep the final position command active until all four wheels converge. */
            mecanum_submit_hybrid_targets();

            if (s_hybrid_profile_complete != 0U) {
                if (mecanum_hybrid_wheels_settled() != 0U) {
                    if (s_hybrid_settle_count < MECANUM_SETTLE_REQUIRED_CYCLES) {
                        s_hybrid_settle_count++;
                    }
                } else {
                    s_hybrid_settle_count = 0U;
                }

                if (s_hybrid_settle_count >= MECANUM_SETTLE_REQUIRED_CYCLES) {
                    DCMotor_OL_StopAll();
                    mecanum_reset_hybrid_state();
                    s_user_vx = 0.0f;
                    s_user_vy = 0.0f;
                    s_user_wz_raw = 0.0f;
                }
            }

            // Hybrid position control owns heading; resynchronise speed-mode yaw target.
            s_target_yaw = angles.yaw;
            s_yaw_integral = 0.0f;
        } else {
            // Idle un-driven state
            s_target_yaw = angles.yaw;
            s_yaw_integral = 0.0f;
        }
        return;
    }
    
    float final_vx = s_user_vx;
    float final_vy = s_user_vy;
    float final_wz = s_user_wz_raw;

    if (s_angle_closed_loop_en) {
        // Integrate the user rotation command into the target yaw directly
        // Provide a small deadband for the joystick so it does not drift
        float eff_wz = s_user_wz_raw;
        if (eff_wz > -3.0f && eff_wz < 3.0f) eff_wz = 0.0f;
        
        if (s_user_vx == 0.0f && s_user_vy == 0.0f && eff_wz == 0.0f) {
            // Un-driven state (idle): Sync target_yaw to prevent rotating to 0 on startup
            // and prevent fighting when the user manually moves the car.
            s_target_yaw = angles.yaw;
            s_yaw_integral = 0.0f;
            final_wz = 0.0f;
        } else {
            if (eff_wz != 0.0f) {
                s_target_yaw += eff_wz * 0.010f; // 10ms period integration
                if (s_target_yaw > 180.0f) s_target_yaw -= 360.0f;
                else if (s_target_yaw < -180.0f) s_target_yaw += 360.0f;
            }

            float error = s_target_yaw - angles.yaw;
            if (error > 180.0f) error -= 360.0f;
            else if (error < -180.0f) error += 360.0f;
            
            if (error > -1.5f && error < 1.5f) {
                s_yaw_integral = 0.0f;
                final_wz = 0.0f; // Deadband to prevent low-speed whine!
            } else {
                s_yaw_integral += error * 0.010f;
                if (s_yaw_integral > 50.0f) s_yaw_integral = 50.0f;
                if (s_yaw_integral < -50.0f) s_yaw_integral = -50.0f;
                
                float kp = 3.0f; 
                float ki = 0.15f;
                float corr_wz = kp * error + ki * s_yaw_integral;
                
                // Add a friction break-away feedforward so we avoid stalling and whining
                if (corr_wz > 0.0f && corr_wz < 12.0f) corr_wz = 12.0f;
                else if (corr_wz < 0.0f && corr_wz > -12.0f) corr_wz = -12.0f;
                
                if (corr_wz > 100.0f) corr_wz = 100.0f;
                if (corr_wz < -100.0f) corr_wz = -100.0f;
                
                final_wz = corr_wz;
            }
        }
    } else {
        if (final_wz > -3.0f && final_wz < 3.0f) final_wz = 0.0f;
    }
    
    float wz_rad_spd = final_wz * (3.14159265f / 180.0f);
    float wz_comp_spd = wz_rad_spd * ((160.0f + 205.0f) / 2.0f);

    float ms1 = final_vx + final_vy + wz_comp_spd;
    float ms2 = -final_vx + final_vy + wz_comp_spd;
    float ms3 = -final_vx - final_vy + wz_comp_spd;
    float ms4 = final_vx - final_vy + wz_comp_spd;

    Mecanum_HW_SetSpeed(MECANUM_MOTOR_FL, ms1);
    Mecanum_HW_SetSpeed(MECANUM_MOTOR_FR, ms2);
    Mecanum_HW_SetSpeed(MECANUM_MOTOR_RR, ms3);
    Mecanum_HW_SetSpeed(MECANUM_MOTOR_RL, ms4);
}
