#ifndef __MECANUM_H
#define __MECANUM_H

#include <stdint.h>
#include <math.h>

/* =================================================================================
 * 麦克纳姆轮 (Mecanum Wheel) 运动学与控制参数配置
 * 
 * 说明：
 * 1. 电机布局（顺时针）：
 *    [ 1 (左前FL) ]-----[ 2 (右前FR) ]
 *          |                 |
 *    [ 4 (左后RL) ]-----[ 3 (右后RR) ]
 *
 * 2. 正方向规定：
 *    - 前进 (Vx): 正值代表向前
 *    - 右平移 (Vy): 正值代表向右
 *    - 自旋 (Wz): 正值代表顺时针旋转(CW)
 * 
 * 3. 电机物理安装旋转对应关系（由用户提供）：
 *    - 前进时：1、4 号电机正转，2、3 号电机反转
 * =================================================================================*/

/* --- 机械参数 (单位：毫米 mm，角度：度 deg) ---
 * 以下参数需根据实际小车底盘规格进行测量并修改，暂填默认值 
 */
#define MECANUM_WHEEL_DIAMETER_MM       (60.0f)   /* 麦轮直径 */
#define MECANUM_WHEEL_CIRCUMFERENCE_MM  (MECANUM_WHEEL_DIAMETER_MM * 3.14159265f) /* 麦轮周长 */

#define MECANUM_WHEEL_BASE_MM           (160.0f)  /* 轴距：前后轮中心在X轴方向（车身纵向）的距离 L */
#define MECANUM_TRACK_WIDTH_MM          (205.0f)  /* 轮距：左右轮中心在Y轴方向（车身横向）的距离 W */

/* 减速比与编码器参数 (预留，用于速度(mm/s)与电机PWM/转速的转换) */
#define MECANUM_REDUCTION_RATIO         (1.0f)    /* 面向车轮的减速比 */
#define MECANUM_PULSES_PER_REV          (1000.0f) /* 电机每圈编码器脉冲数/或步进电机细分步数 */

/* --- 运动学换算参数 --- */
/* 自旋系数 K = (轴距 L + 轮距 W) / 2 */
#define MECANUM_K_ROTATION_COEFF_MM     ((MECANUM_WHEEL_BASE_MM + MECANUM_TRACK_WIDTH_MM) / 2.0f)
#define MECANUM_RAD_PER_DEG             (3.14159265f / 180.0f)

/* 电机 ID 定义 */
typedef enum {
    MECANUM_MOTOR_FL = 1, /* 左前 */
    MECANUM_MOTOR_FR = 2, /* 右前 */
    MECANUM_MOTOR_RR = 3, /* 右后 */
    MECANUM_MOTOR_RL = 4  /* 左后 */
} MecanumMotor_t;

/* 控制模式定义 */
// typedef enum {
//     MECANUM_MODE_SPEED    = 0, /* 速度模式：参数单位为 mm/s, deg/s */
//     MECANUM_MODE_DISTANCE = 1  /* 距离位置模式：参数单位为 mm, deg */
// } MecanumControlMode_t;

/* --- 核心运动学混合控制接口 ---
 * 参数：
 *   vx_spd  : 向前速度
 *   vy_spd  : 向右速度
 *   wz_spd  : 顺时针自旋速度
 *   dx_dist : 向前距离
 *   dy_dist : 向右距离
 *   dw_deg  : 顺时针自旋角度
 */
void Mecanum_MixedControl(float vx_spd, float vy_spd, float wz_spd, float dx_dist, float dy_dist, float dw_deg);

/* --- 单独分离的控制指令 (基于 MixedControl 封装) --- */
void Mecanum_Translate_Forward(float speed, float dist);
void Mecanum_Translate_Backward(float speed, float dist);
void Mecanum_Translate_Right(float speed, float dist);
void Mecanum_Translate_Left(float speed, float dist);
void Mecanum_Rotate_CW(float speed, float dist);
void Mecanum_Rotate_CCW(float speed, float dist);

/* --- 底层硬件挂载函数 (由用户在其他文件中实现或弱定义覆盖) --- */
/* 设置电机速度目标，传入的 speed 单位需用户结合物理参数转换为底层 PWM 或 RPM */
extern void Mecanum_HW_SetSpeed(uint8_t motor_id, float speed_val);
/* 设置电机位置目标，传入的 dist 单位需用户结合物理参数转换为底层脉冲数，同时限制速度 */
extern void Mecanum_HW_SetDistance(uint8_t motor_id, float dist_val, float speed_val);

#endif /* __MECANUM_H */