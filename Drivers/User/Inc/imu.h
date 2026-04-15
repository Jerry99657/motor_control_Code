/**
 ****************************************************************************************************
 * @file        imu.h
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

#ifndef __IMU_H
#define __IMU_H

#include "main.h"


/* ��Ԫ���ṹ������ */
typedef struct{
    float q0;
    float q1;
    float q2;
    float q3;
}quater_info_t;

/* ŷ���ǽṹ������ */
typedef struct{
    float pitch;
    float roll;
    float yaw;
}eulerian_angles_t;

/******************************************************************************************/

void imu_init(void);
eulerian_angles_t imu_get_eulerian_angles(float gx, float gy, float gz, float ax, float ay, float az);
void imu_data_calibration(short *gx, short *gy, short *gz, short *ax, short *ay, short *az);

#endif
