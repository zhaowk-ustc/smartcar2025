/*
 * control.c
 *
 *  Created on: 2024��10��24��
 *      Author: drama
 */
#include "zf_common_headfile.h"
#include "control.h"
#include "flash.h"
#include "imu.h"
#include "motor.h"
#include "PID.h"
#include "Display.h"
#include "Guandao_Plus.h"
#include "key.h"

#include "filter.h"

//----------------------------------------------------------------------------------------------------------------
// �������� CodeInit
// ������� ȫ�������ʼ��
// ����˵��
// ���ز���
// ʹ��ʾ�� CodeInit();
// ��ע��Ϣ
//----------------------------------------------------------------------------------------------------------------
uint8_t count = 0;
int Run_Flag = 0; // ������־λ
int MID_W = 94;   // ���߱�׼ֵ
void CodeInit(void)
{
    encoder_Init();         // ��������ʼ��
    motor_Init();           // �����ʼ��
    imu660ra_init();        // ��̬��������ʼ��
    gyro_zero_param_init(); // �궨��Ʈ���µģ�
    acc_zero_param_init();
    flash_init();

    //    ips200_init(IPS200_TYPE_SPI);
    // IPS200_Cammer_Init();       //����ͷ��ʼ��
    // ips200_full(RGB565_BLACK); // ��Ļ��ʼ��
    ips200_clear();
    ips200_init(IPS200_TYPE_SPI);
    ips200_full(RGB565_BLACK); // ��Ļ��ʼ��

    KEY_Init(); // ������ʼ��

    pit_ms_init(PIT_CH0, 5);    // ���5ms�ж�
    pit_ms_init(PIT_CH1, 5);    // ������5ms�ж�
    pit_ms_init(PIT_CH2, 2000); //

    // ��ʼ���˲�������ֹƵ��15Hz������ʱ��0.005�룩
    LPF_InitByFrequency(&velocity_filter, 15.0f, 0.005f);

    //    FuzzyPID_Init(&pos_pd,2,0.01); // λ��ʽpd�ṹ���ʼ����ֵ
    PI_Init(&L_inc_pi, 11, 0.1, 0); // ������ʽpid�ṹ��   11   0.1
    PI_Init(&R_inc_pi, 11, 0.1, 0); //  ������ʽpid�ṹ��   11   0.1
}

void Motor_control(float L_expect, float L_actual, float R_expect, float R_actual)
{
    // pos_pd.expect = MID_W;              // ��������ֵ
    // pos_pd.actual = Mid_Line_Calculate; // ʵ������ֵ

    pos_pd.expect = 0.0f;  // �����ߵ��������Ϊ0
    pos_pd.actual = e_lat; // ��ǰʵ�ʹߵ��������

    //    speed_diff = (int)FuzzyPID_control(&pos_pd);
    speed_diff = err_guandao_plus; //-(int) PD_control(&pos_pd, 1.0, 0.05)�ĸ��ű�ʾȡ��

    //    	L_inc_pi.expect = - speed_diff;  //L_expect
    //        R_inc_pi.expect =   speed_diff;  //R_expect

    L_inc_pi.expect = L_expect - speed_diff; // L_expect
    R_inc_pi.expect = R_expect + speed_diff; // R_expect

    //    L_inc_pi.expect = L_expect; // L_expect
    //    R_inc_pi.expect = R_expect; // R_expect

    L_inc_pi.actual = L_actual;
    R_inc_pi.actual = R_actual;

    l_out = PI_control(&L_inc_pi);
    r_out = PI_control(&R_inc_pi);

    if (locate_index >= road_destination && road_recurrent_Plus_flag == 1)
    {
        l_out = 0;
        r_out = 0;
    } // ����·���յ��Զ�ͣ��

    motor_run((int16)l_out, (int16)r_out);
}

#define MID_FILTER_SIEZ (21) // ѡȡ��������
int dataBuf[MID_FILTER_SIEZ];
uint16 index;

int16 Mid_filter(int16 data)
{
    uint16 i, j;
    int16 temp;
    dataBuf[index] = data;
    index = (index + 1) % MID_FILTER_SIEZ;

    for (i = 0; i < MID_FILTER_SIEZ - 1; i++)
    {
        for (j = 0; j < MID_FILTER_SIEZ - 1 - i; j++)
        {
            if (dataBuf[j] > dataBuf[j + 1])
            {
                temp = dataBuf[j];
                dataBuf[j] = dataBuf[j + 1];
                dataBuf[j + 1] = temp;
            }
        }
    }
    return dataBuf[(MID_FILTER_SIEZ - 1) / 2];
}