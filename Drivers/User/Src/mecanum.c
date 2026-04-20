#include "mecanum.h"
#include "dc_motor_ol.h"

/* 
 * 硬件回调实现
 * 将运动学算法计算出的轮端线速度/距离指令，下发给实际的底层电机控制器 (DC Motor)
 */
void Mecanum_HW_SetSpeed(uint8_t motor_id, float speed_val) {
    /* 1. 将轮端所需的线速度 (mm/s) 转换为电机转速 (RPM)
     *    公式: RPM = 速度(mm/s) * 60(秒) / 麦轮周长(mm) 
     */
    float target_rpm = (speed_val * 60.0f) / MECANUM_WHEEL_CIRCUMFERENCE_MM;

    /* 2. 在目前的 dc_motor 控制器中，输入的是速度的百分比 (-100 到 100)，
     *    且 100% 对应 DCMOTOR_OL_MAX_TARGET_RPM (宏定义为 300 RPM)。
     */
    float percent_f = (target_rpm * 100.0f) / (float)DCMOTOR_OL_MAX_TARGET_RPM;

    /* 3. 限幅处理并下发底层接口 */
    if (percent_f > 100.0f)  percent_f = 100.0f;
    if (percent_f < -100.0f) percent_f = -100.0f;
    
    DCMotor_OL_SetSpeed(motor_id, (int16_t)percent_f);
}

void Mecanum_HW_SetDistance(uint8_t motor_id, float dist_val, float speed_val) {
    /* 1. 将轮端所需的位移 (mm) 转换为电机编码器脉冲数
     *    公式: Pulses = (距离(mm) / 麦轮周长(mm)) * 转一圈的脉冲数 
     */
    float pulses = (dist_val / MECANUM_WHEEL_CIRCUMFERENCE_MM) * DCMOTOR_OL_ENCODER_COUNTS_PER_REV;

    /* 2. 获取当前电机位置，并计算目标脉冲值 */
    int64_t current_pulses = DCMotor_OL_GetPositionPulses(motor_id);
    int64_t target_pulses = current_pulses + (int64_t)pulses;

    /* 3. 将轮端所需的线速度 (mm/s) 转换为速度百分比 */
    float target_rpm = (speed_val * 60.0f) / MECANUM_WHEEL_CIRCUMFERENCE_MM;
    float percent_f = (target_rpm * 100.0f) / (float)DCMOTOR_OL_MAX_TARGET_RPM;
    
    if (percent_f < 0.0f) percent_f = -percent_f;
    if (percent_f > 100.0f) percent_f = 100.0f;

    /* 4. 下发目标给底层控制器 */
    DCMotor_OL_SetTargetPosition(motor_id, target_pulses, (int16_t)percent_f);
}

/* =================================================================================
 * Mecanum_MixedControl
 * [ 综合控制主参数分配算法 ]
 * 
 * 推导：
 * 设底盘标准坐标系：向前为X正，向右为Y正，顺时针(俯视)为自旋W正。
 * 纯车轮切向速度： vL_x = Vx + Vy + W_rad * K
 * 根据用户定义的物理转向：1/4 电机正转小车往前进，2/3 电机反转小车往前进。
 * 故车轮速度：
 * 【左前1：电机正转，直接跟随前进】 M1 = +(Vx + Vy + W * K)
 * 【右前2：电机反转，补偿成负数】   M2 = -(Vx - Vy - W * K) = -Vx + Vy + W * K
 * 【右后3：电机反转，补偿成负数】   M3 = -(Vx + Vy - W * K) = -Vx - Vy + W * K
 * 【左后4：电机正转，直接跟随前进】 M4 = +(Vx - Vy + W * K)
 * 验证：
 * 1. 纯前进(Vx>0): M1=+Vx, M2=-Vx, M3=-Vx, M4=+Vx  -> 满足用户规定：前进时14正转，23反转。
 * 2. 纯右移(Vy>0): M1=+Vy, M2=+Vy, M3=-Vy, M4=-Vy  -> 这对于麦轮的X型安装或对顶安装为标准的右平移形式。
 * 3. 纯顺转(Wz>0): M1=+Wz, M2=+Wz, M3=+Wz, M4=+Wz  -> 四轮全部正转，左侧向前，右侧向后，小车达成原地顺时针旋转。
 * 
 * 此公式也同时支持结合使用。比如前进且要右转(弧线): 输入Vx 和 Wz。
 *=================================================================================*/
void Mecanum_MixedControl(float vx_spd, float vy_spd, float wz_spd, float dx_dist, float dy_dist, float dw_deg) {
    /* 1. 将角度参数转换为弧度系数参与分配 */
    float wz_rad_spd = wz_spd * MECANUM_RAD_PER_DEG;
    float wz_comp_spd = wz_rad_spd * MECANUM_K_ROTATION_COEFF_MM;
    float dw_rad_dist = dw_deg * MECANUM_RAD_PER_DEG;
    float dw_comp_dist = dw_rad_dist * MECANUM_K_ROTATION_COEFF_MM;

    /* 2. 进行麦克纳姆轮驱动算法逆解计算 */
    float ms1 = vx_spd + vy_spd + wz_comp_spd;
    float ms2 = -vx_spd + vy_spd + wz_comp_spd;
    float ms3 = -vx_spd - vy_spd + wz_comp_spd;
    float ms4 = vx_spd - vy_spd + wz_comp_spd;

    float md1 = dx_dist + dy_dist + dw_comp_dist;
    float md2 = -dx_dist + dy_dist + dw_comp_dist;
    float md3 = -dx_dist - dy_dist + dw_comp_dist;
    float md4 = dx_dist - dy_dist + dw_comp_dist;

    /* 3. 分发给底层抽象控制器执行模式切换下达 */
    if (dx_dist == 0.0f && dy_dist == 0.0f && dw_deg == 0.0f) {
        Mecanum_HW_SetSpeed(MECANUM_MOTOR_FL, ms1);
        Mecanum_HW_SetSpeed(MECANUM_MOTOR_FR, ms2);
        Mecanum_HW_SetSpeed(MECANUM_MOTOR_RR, ms3);
        Mecanum_HW_SetSpeed(MECANUM_MOTOR_RL, ms4);
    } else {
        Mecanum_HW_SetDistance(MECANUM_MOTOR_FL, md1, ms1);
        Mecanum_HW_SetDistance(MECANUM_MOTOR_FR, md2, ms2);
        Mecanum_HW_SetDistance(MECANUM_MOTOR_RR, md3, ms3);
        Mecanum_HW_SetDistance(MECANUM_MOTOR_RL, md4, ms4);
    }
}

/* =================================================================================
 * 单独剥离并封装出的独立控制调用接口
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