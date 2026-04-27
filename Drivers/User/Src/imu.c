/**
 ****************************************************************************************************
 * @file        imu.c
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2022-04-20
 * @brief       ��̬���� ����
 *              ���Ĵ���ο���:https://gitee.com/brimon-zzy/icm20602forstm32f103?_from=gitee_search
 * @license     
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:����ԭ�� ������ F429������
 * ������Ƶ:www.yuanzige.com
 * ������̳:www.openedv.com
 * ��˾��ַ:www.alientek.com
 * �����ַ:openedv.taobao.com
 *
 * �޸�˵��
 * V1.0 20220420
 * ��һ�η���
 *
 ****************************************************************************************************
 */

#include "imu.h"
#include "mpu6500.h"
#include <math.h>


#define IMU_DELTA_T         0.005f     /* 5ms����һ�� */
#define IMU_M_PI            3.1425f

#define IMU_NEW_WEIGHT      0.35f      /* ��ֵȨ�� */
#define IMU_OLD_WEIGHT      0.65f      /* ��ֵȨ�� */

quater_info_t g_q_info = {1, 0, 0, 0};    /* ȫ����Ԫ�� */

float g_param_kp = 10.0f;              /* ���ٶȼ�(������)���������ʱ�������50 */
float g_param_ki = 0.02f;              /* �������������ʵĻ������� 0.2 */

short g_acc_avg[3];                    /* ���ٶ�ƽ��ֵ */
short g_gyro_avg[3];                   /* ������ƽ��ֵ */

/**
 * @brief       ��������
 * @param       x           : ��������ֵ
 * @retval      �������
 */
static float imu_inv_sqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/**
 * @brief       ����ת��
 * @note        �Լ��ٶ�������һ�׵�ͨ�˲�(�ο�����)����gyroת�ɻ���ÿ��(2000dps)
 * @param       gx, gy, gz  : 3������������ָ��
 * @param       ax, ay, az  : 3����ٶ�����ָ��
 * @retval      �� 
 */
static void imu_data_transform(float *gx, float *gy, float *gz, float *ax, float *ay, float *az)
{
    static double lastax = 0;
    static double lastay = 0;
    static double lastaz = 0;

    *ax = *ax * IMU_NEW_WEIGHT + lastax * IMU_OLD_WEIGHT;
    *ay = *ay * IMU_NEW_WEIGHT + lastay * IMU_OLD_WEIGHT;
    *az = *az * IMU_NEW_WEIGHT + lastaz * IMU_OLD_WEIGHT;
    
    lastax = *ax;
    lastay = *ay;
    lastaz = *az;

    *gx = *gx * IMU_M_PI / 180 / 16.4f;
    *gy = *gy * IMU_M_PI / 180 / 16.4f;
    *gz = *gz * IMU_M_PI / 180 / 16.4f;
}

/**
 * @brief       ��̬�����ں�, �����㷨
 * @note        ʹ�õ��ǻ����˲��㷨��û��ʹ��Kalman�˲��㷨
 *              ������֤�ú����ĵ���Ƶ��Ϊ: IMU_DELTA_T , ����YAW����Ӧ��ƫ��/ƫС
 * @param       gx, gy, gz  : 3������������
 * @param       ax, ay, az  : 3����ٶ�����
 * @retval      �� 
 */
static void imu_ahrsupdate_nomagnetic(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    static float i_ex, i_ey, i_ez;    /* ������ */

    float half_t = 0.5f * dt;
    float vx, vy, vz;      /* ��ǰ�Ļ�������ϵ�ϵ�������λ���� */
    float ex, ey, ez;      /* ��Ԫ������ֵ����ٶȼƲ���ֵ����� */
    float q0 = g_q_info.q0;
    float q1 = g_q_info.q1;
    float q2 = g_q_info.q2;
    float q3 = g_q_info.q3;
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;
    float delta_2 = 0;

    /* �Լ��ٶ����ݽ��й�һ�� �õ���λ���ٶ� */
    float norm = imu_inv_sqrt(ax * ax + ay * ay + az * az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;
    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = 0.0f; /* No yaw reference to correct ez! Force to 0 to prevent drift */

    /* �ò���������PI������������ƫ��
     * ͨ������ g_param_kp��g_param_ki ����������
     * ���Կ��Ƽ��ٶȼ����������ǻ�����̬���ٶȡ�*/
    i_ex += dt * ex;   /* integral error scaled by Ki */
    i_ey += dt * ey;
    i_ez = 0.0f; // Force reset integrated error on Yaw (Z-axis)

    gx = gx + g_param_kp * ex + g_param_ki * i_ex;
    gy = gy + g_param_kp * ey + g_param_ki * i_ey;
    gz = gz; // Do not apply Z-axis gravity error compensation to Yaw


    /*����������ɣ���������Ԫ��΢�ַ���*/

    /* ��Ԫ��΢�ַ��̣�����half_tΪ�������ڵ�1/4��gx gy gzΪ�����ǽ��ٶȣ�
       ���¶�����֪��������ʹ����һ��������������Ԫ��΢�ַ��� */
//    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * half_t;
//    q1 = q1 + ( q0 * gx + q2 * gz - q3 * gy) * half_t;
//    q2 = q2 + ( q0 * gy - q1 * gz + q3 * gx) * half_t;
//    q3 = q3 + ( q0 * gz + q1 * gy - q2 * gx) * half_t;
    delta_2 = (half_t * gx) * (half_t * gx) + (half_t * gy) * (half_t * gy) + (half_t * gz) * (half_t * gz);

    /* ������Ԫ����    ��Ԫ��΢�ַ���  ��Ԫ�������㷨�����ױϿ��� */
    q0 = (1 - delta_2 / 2.0f) * q0 + (-q1 * gx - q2 * gy - q3 * gz) * half_t;
    q1 = (1 - delta_2 / 2.0f) * q1 + (q0 * gx + q2 * gz - q3 * gy) * half_t;
    q2 = (1 - delta_2 / 2.0f) * q2 + (q0 * gy - q1 * gz + q3 * gx) * half_t;
    q3 = (1 - delta_2 / 2.0f) * q3 + (q0 * gz + q1 * gy - q2 * gx) * half_t;

    /* normalise quaternion */
    norm = imu_inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    g_q_info.q0 = q0 * norm;
    g_q_info.q1 = q1 * norm;
    g_q_info.q2 = q2 * norm;
    g_q_info.q3 = q3 * norm;
}

/**
 * @brief       �õ���̬������ŷ����
 * @param       gx, gy, gz  : 3������������
 * @param       ax, ay, az  : 3����ٶ�����
 * @retval      ����ֵ : ŷ����
 */
eulerian_angles_t imu_get_eulerian_angles(float gx, float gy, float gz, float ax, float ay, float az)
{
    eulerian_angles_t eulerangle;

    imu_data_transform(&gx, &gy, &gz, &ax, &ay, &az);       /* ����ת�� */
    static uint32_t last_tick = 0;
    uint32_t now = HAL_GetTick();
    float dt = (last_tick > 0) ? ((now - last_tick) / 1000.0f) : 0.005f;
    if (dt <= 0.0f || dt > 0.5f) dt = 0.005f;
    last_tick = now;
    imu_ahrsupdate_nomagnetic(gx, gy, gz, ax, ay, az, dt);      /* ��̬���� */
    
    float q0 = g_q_info.q0;
    float q1 = g_q_info.q1;
    float q2 = g_q_info.q2;
    float q3 = g_q_info.q3;

    eulerangle.pitch = -asin(- 2 * q1 * q3 + 2 * q0 * q2) * 180 / IMU_M_PI;
    eulerangle.roll = atan2(2 * q2 * q3 + 2 * q0 * q1, - 2 * q1 * q1 - 2 * q2 * q2 + 1) * 180 / IMU_M_PI;
    eulerangle.yaw = -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 180 / IMU_M_PI;

    /* ���Բ�������̬�޶ȵ����� */
    if (eulerangle.roll > 90 || eulerangle.roll < -90)
    {
        if (eulerangle.pitch > 0)
        {
            eulerangle.pitch = 180 - eulerangle.pitch;
        }

        if (eulerangle.pitch < 0)
        {
            eulerangle.pitch = -(180 + eulerangle.pitch);
        }
    }

    if (eulerangle.yaw > 180)
    {
        eulerangle.yaw -= 360;
    }
    else if (eulerangle.yaw < -180)
    {
        eulerangle.yaw += 360;
    }

    return eulerangle;
}

/**
 * @brief       ����У׼
 * @note        �����ݼ�ȥ��ֵ, ���ٶȼ�ȥ�������ٶ�Ӱ��
 * @param       gx, gy, gz  : 3������������ָ��
 * @param       ax, ay, az  : 3����ٶ�����ָ��
 * @retval      �� 
 */
void imu_data_calibration(short *gx, short *gy, short *gz, short *ax, short *ay, short *az)
{
    *gx -= g_gyro_avg[0];
    *gy -= g_gyro_avg[1];
    *gz -= g_gyro_avg[2];
    *ax -= g_acc_avg[0];
    *ay -= g_acc_avg[1];
    *az -= (g_acc_avg[2] - 2048);

    if (*gx >= -15 && *gx <= 15) *gx = 0;
    if (*gy >= -15 && *gy <= 15) *gy = 0;
    if (*gz >= -15 && *gz <= 15) *gz = 0;
}

/**
 * @brief       ��̬�����ʼ��
 * @note        �ú��������ڴ�������ʼ��֮���ٵ���
 * @param       ��
 * @retval      ��
 */
void imu_init(void)
{
    uint16_t i = 0;
    int acc_sum[3] = {0}, gyro_sum[3] = {0};

    short acc_data[3];          /* ���ٶȴ�����ԭʼ���� */
    short gyro_data[3];         /* ������ԭʼ���� */

    HAL_Delay(100);
    for (i = 0; i < 250; i++)   /* 循锟斤拷锟斤拷取250锟斤拷 取平锟斤拷 */
    {
        MPU6500_GetData(&acc_data[0], &acc_data[1], &acc_data[2], &gyro_data[0], &gyro_data[1], &gyro_data[2]);

        acc_sum[0] += acc_data[0];
        acc_sum[1] += acc_data[1];
        acc_sum[2] += acc_data[2];
        gyro_sum[0] += gyro_data[0];
        gyro_sum[1] += gyro_data[1];
        gyro_sum[2] += gyro_data[2];

        HAL_Delay(5);
    }

    g_acc_avg[0] = acc_sum[0] / 250;
    g_acc_avg[1] = acc_sum[1] / 250;
    g_acc_avg[2] = acc_sum[2] / 250;
    g_gyro_avg[0] = gyro_sum[0] / 250;
    g_gyro_avg[1] = gyro_sum[1] / 250;
    g_gyro_avg[2] = gyro_sum[2] / 250;
}

