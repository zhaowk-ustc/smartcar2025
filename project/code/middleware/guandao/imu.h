/*
 * imu.h
 *
 *  Created on: 2024��12��30��
 *      Author: ������
 */

#ifndef CODE_IMU_H_
#define CODE_IMU_H_
#include "zf_common_headfile.h"

#define pi (PI)

typedef struct
{
    float Xdata;
    float Ydata;
    float Zdata;
} gyro_zero_paramTypedef; // ��Ʈ������

typedef struct
{
    float Xdata;
    float Ydata;
    float Zdata;
} acc_zero_paramTypedef; // ��Ʈ������

typedef struct
{
    float gyro_x;
    float gyro_y;
    float gyro_z;
} gyro_paramTypedef; // �Ǽ��ٶ�

typedef struct
{
    float acc_x;
    float acc_y;
    float acc_z;
} acc_paramTypedef; // ���ٶȼ�

extern volatile gyro_paramTypedef gyro_param; // �Ǽ��ٶ�
extern volatile acc_paramTypedef acc_param;   // ���ٶȼ�
extern uint8_t gyro_zero_flag, acc_zero_flag; // ��������ƮУ׼��ɱ�־��
extern uint8_t acc_zero_flag;                 // ��������ƮУ׼��ɱ�־��
extern float yaw_memery[FLASH_PAGE_LENGTH];   // ��¼�ĺ�ƫ��
extern float yaw_store[FLASH_PAGE_LENGTH];    // ��㴢�����ʷ��ƫ��
extern volatile float yaw, yaw_plus;          // ʵʱ��ƫ��
extern volatile float acc_y_speed;

void acc_y_integral(void);
void gyro_yaw_integral(void);    // ��ֵ����
void gyro_transform_value(void); // תΪʵ������ֵ
void gyro_zero_param_init(void); // �궨��Ʈ����
void acc_transform_value(void);  // תΪʵ������ֵ
void acc_zero_param_init(void);  // �궨��Ʈ����
#endif                           
/* CODE_IMU_H_ */
