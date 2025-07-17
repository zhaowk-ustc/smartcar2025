/*
 * Guandao_Plus.h
 *
 *  Created on: 2025��3��26��
 *      Author: ������
 */

#ifndef CODE_GUANDAO_PLUS_H_
#define CODE_GUANDAO_PLUS_H_

#include "zf_common_headfile.h"
#include "imu.h"
#include "filter.h"
#include <stdbool.h>

extern uint8_t road_memery_finish_Plus_flag; // ·��������ɱ�־λ
extern uint8_t road_memery_start_Plus_flag;  // ·�����俪ʼ��־λ
extern uint8_t road_recurrent_Plus_flag;     // ·�����ֱ�־λ

extern uint16_t NUM_L_Plus, NUM_R_Plus;
extern volatile float X, Y;
extern float X_Memery_Plus[FLASH_PAGE_LENGTH * 6];
extern float Y_Memery_Plus[FLASH_PAGE_LENGTH * 6];
extern float X_Memery_Store_Plus[FLASH_PAGE_LENGTH * 6];
extern float Y_Memery_Store_Plus[FLASH_PAGE_LENGTH * 6];

typedef struct
{
    volatile int32_t total_pulses;       // �ۼ�������������Ϊ������ʾ����ת����
    volatile int32_t last_total_pulses;  // ��һ�ε��ۼ������������ڼ�����ֵ
    volatile int16_t encoder_last;       // ��һ�α�����ֵ������Update_Wheel_Pulses��
    volatile int16_t encoder_prev_speed; // �ٶȼ����׼ֵ������Get_Wheel_Speed��
    const int32_t target_pulses;         // ����������ֵ����������ʼ���󲻿��޸ģ�
} WheelData;

extern WheelData wheel_left, wheel_right;
extern volatile int32_t speed_left;  // �����ٶȣ�����/5ms��
extern volatile int32_t speed_right; // �����ٶȣ�����/5ms��
extern volatile int16 err_guandao_plus;
extern volatile float e_lat;
/* �򻬼�����ò��� */
#define WHEEL_BASE_CM 15.0f               // �����ּ�ࣨ��λ�����ף�
#define ENCODER_PULSES_PER_REVOLUTION 366 // ÿ1cm������������
#define SAMPLING_INTERVAL_MS 5            // ���ݲ����������λ�����룩�����ж������й�

/* ����򻬼����ֵ */
#define LATERAL_SLIP_YAW_RATE_THRESHOLD 10.0f // ƫ�����ٶ�ƫ����ֵ����λ������/�룩
#define MIN_LATERAL_DETECT_SPEED_CMPS 200.0f  // ��ͺ������ٶȣ���λ������/�룩

/* ����򻬼����ֵ */
#define LONGITUDINAL_ACCEL_DIFF_THRESHOLD 1.0f  // ���ٶȲ�����ֵ����λ����/��?��
#define MIN_LONGITUDINAL_DETECT_SPEED_CMPS 1.0f // ����������ٶȣ���λ����/�룩

/* ���Ʊ�־λ���� */
typedef enum
{
    SLIP_FLAG_NONE = 0x00,               // �޻���
    SLIP_FLAG_LATERAL_LEFT = 0x01,       // �������ƣ����ִ򻬣�
    SLIP_FLAG_LATERAL_RIGHT = 0x02,      // �Ҳ�����ƣ����ִ򻬣�
    SLIP_FLAG_LONGITUDINAL_ACCEL = 0x04, // ������ٻ��ƣ������ֿ�ת��
    SLIP_FLAG_LONGITUDINAL_DECEL = 0x08  // ������ٻ��ƣ���̥������
} SlipFlags;

/* �ⲿ�������� */
extern volatile SlipFlags wheelSlipFlags; // ����״̬��־λ

extern volatile float leftWheelSpeedCmps;  // �������ٶȣ���λ������/�룩
extern volatile float rightWheelSpeedCmps; // �������ٶȣ���λ������/�룩
extern volatile float currentAvgSpeedCmps; // ��ǰƽ�����٣���λ���� /�룩
extern volatile int locate_index;          // С����λ��
extern uint16_t road_destination;

void Distance_Get_Plus(void);
int32_t Get_Wheel_Speed(WheelData *wheel, int16_t current_encoder);
int32_t Update_Wheel_Pulses(WheelData *wheel, int16_t current_encoder);
void Slip_Check(void);
int16_t pure_pursuit_control(void);
#endif /* CODE_GUANDAO_PLUS_H_ */
