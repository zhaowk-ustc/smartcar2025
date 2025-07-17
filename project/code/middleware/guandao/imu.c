/*
 * imu.c
 *
 *  Created on: 2024��12��30��
 *      Author: ������
 */
#include "zf_common_headfile.h"
#include "imu.h"
#include "filter.h"

uint8_t gyro_zero_flag = 0, acc_zero_flag=0; // ��Ʈ�궨��ɱ�־λ��ʼ��
volatile float yaw_plus = 0; // С����������ϵy���붫�챱orȫ������ϵy��н�(��ʱ��Ϊ����˳ʱ��Ϊ������Χ[-pi,pi])

volatile float yaw = 0;                    // ʵʱ��ƫ�ǳ�ʼ��
volatile float yaw_last = 0;               // ��ֵ���ֺ�ƫ��ǰ��״̬��ʼ��
volatile float yaw_now;                    // ��ֵ���ֺ�ƫ��ǰ��״̬��ʼ��

volatile float acc_y_speed = 0;      // ʵʱС��ʵ���ٶȳ�ʼ��
volatile float acc_last = 0;         // ��ֵ���ּ��ٶ�ǰ��״̬��ʼ��
volatile float acc_now;      // ��ֵ���ּ��ٶ�ǰ��״̬��ʼ��

float yaw_memery[FLASH_PAGE_LENGTH] = {0}; // ��¼�ĺ�ƫ��
float yaw_store[FLASH_PAGE_LENGTH] = {0};  // ��㴢�����ʷ��ƫ��
gyro_zero_paramTypedef gyro_zero_param;    // ��Ʈ����
acc_zero_paramTypedef acc_zero_param;      // ��Ʈ����

volatile gyro_paramTypedef gyro_param = {
    .gyro_x = 0,
    .gyro_y = 0,
    .gyro_z = 0

}; // �Ǽ��ٶ�
volatile acc_paramTypedef acc_param = {
    .acc_x = 0,
    .acc_y = 0,
    .acc_z = 0}; // ���ٶȼ�

// ��Ʈ�궨
void gyro_zero_param_init(void)
{

    gyro_zero_param.Xdata = 0;
    gyro_zero_param.Ydata = 0; // ��Ʈ����
    gyro_zero_param.Zdata = 0;
    for (uint16_t i = 0; i < 100; i++)
    {
        imu660ra_get_gyro();
        gyro_zero_param.Xdata += imu660ra_gyro_x;
        gyro_zero_param.Ydata += imu660ra_gyro_y;
        gyro_zero_param.Zdata += imu660ra_gyro_z;
        system_delay_ms(5);
    }
    gyro_zero_param.Xdata /= 100;
    gyro_zero_param.Ydata /= 100;
    gyro_zero_param.Zdata /= 100;
    gyro_zero_flag = 1; // ��Ʈ�궨���,�����Ҫ1s����
}

void acc_zero_param_init(void)
{

    acc_zero_param.Xdata = 0;
    acc_zero_param.Ydata = 0; // ��Ʈ����
    acc_zero_param.Zdata = 0;
    for (uint16_t i = 0; i < 100; i++)
    {
        imu660ra_get_acc();
        acc_zero_param.Xdata += imu660ra_acc_x;
        acc_zero_param.Ydata += imu660ra_acc_y;
        acc_zero_param.Zdata += imu660ra_acc_z;
        system_delay_ms(5);
    }
    acc_zero_param.Xdata /= 100;
    acc_zero_param.Ydata /= 100;
    acc_zero_param.Zdata /= 100;
    acc_zero_flag = 1; // ��Ʈ�궨���,�����Ҫ1s����
}

// ��λת��Ϊ����
void gyro_transform_value(void)
{
    gyro_param.gyro_x = imu660ra_gyro_transition((float)imu660ra_gyro_x - gyro_zero_param.Xdata);
    gyro_param.gyro_y = imu660ra_gyro_transition((float)imu660ra_gyro_y - gyro_zero_param.Ydata);
    gyro_param.gyro_z = imu660ra_gyro_transition((float)imu660ra_gyro_z - gyro_zero_param.Zdata);
    // תΪʵ������ֵ����λ��
}

// ��λת��Ϊ����
void acc_transform_value(void)
{
    acc_param.acc_x = imu660ra_acc_transition((float)imu660ra_acc_x - acc_zero_param.Xdata);
    acc_param.acc_y = imu660ra_acc_transition((float)imu660ra_acc_y - acc_zero_param.Ydata);
    acc_param.acc_z = imu660ra_acc_transition((float)imu660ra_acc_z - acc_zero_param.Zdata);
    // תΪʵ������ֵ����λm^2/s
}

// ��ֵ������Ƕ�
void gyro_yaw_integral(void)
{

    if (fabsf(gyro_param.gyro_z) < 5.0f)
        gyro_param.gyro_z = 0;

    yaw_now = gyro_param.gyro_z;
    yaw += -(yaw_last + yaw_now) / 2 * 0.005 * 0.0633; // ��ֵ���ַ�

   

    yaw_last = yaw_now;

    yaw_plus += ((yaw_last + yaw_now) / 2 * 0.005 * 0.0633) * pi / 180.00; // ��ʱ��Ϊ����˳ʱ��Ϊ��

    if (yaw_plus >= pi)
    {
        yaw_plus -= 2 * pi;
    }
    else if (yaw_plus < -pi)
    {
        yaw_plus += 2 * pi;
    }
    // �淶��[-180,180]��
}


// ��ֵ�������ٶ�
void acc_y_integral(void)
{

    if (fabsf(acc_param.acc_y) < 50.0f)
        acc_param.acc_y = 0;

    acc_param.acc_y = LPF_Update(&velocity_filter, acc_param.acc_y);

    acc_now = acc_param.acc_y;
    acc_y_speed += (acc_last + acc_now) / 2 * 0.005*0.01; // ��ֵ���ַ�

    acc_last = acc_now;

  
}