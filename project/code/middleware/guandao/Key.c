/*
 * key.c
 *
 *  Created on: 2024��10��30��
 *      Author: 86152
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
int16 KEY_State;
int16 Key_Flag = 1;

// ������ʼ��
void KEY_Init(void)
{
    gpio_init(KEY1, GPI, GPIO_HIGH, GPI_PULL_UP); // ��ʼ�� KEY1 ���� Ĭ�ϸߵ�ƽ ��������
    gpio_init(KEY2, GPI, GPIO_HIGH, GPI_PULL_UP); // ��ʼ�� KEY2 ���� Ĭ�ϸߵ�ƽ ��������
    gpio_init(KEY3, GPI, GPIO_HIGH, GPI_PULL_UP); // ��ʼ�� KEY3 ���� Ĭ�ϸߵ�ƽ ��������
    gpio_init(KEY4, GPI, GPIO_HIGH, GPI_PULL_UP); // ��ʼ�� KEY4 ���� Ĭ�ϸߵ�ƽ ��������

    gpio_init(SWITCH1, GPI, GPIO_HIGH, GPI_PULL_UP); // ��ʼ�� SWITCH1 ���� Ĭ�ϸߵ�ƽ ��������
    gpio_init(SWITCH2, GPI, GPIO_HIGH, GPI_PULL_UP); // ��ʼ�� SWITCH2 ���� Ĭ�ϸߵ�ƽ ��������
}

// ����ɨ��
void KEY_Scan(void)
{
    // �жϱ��뿪��״̬
    if (gpio_get_level(SWITCH1) && gpio_get_level(SWITCH2))
        KEY_State = 0;
    if (!(gpio_get_level(SWITCH1)) && gpio_get_level(SWITCH2))
        KEY_State = 1;
    if (gpio_get_level(SWITCH1) && !(gpio_get_level(SWITCH2)))
        KEY_State = 2;
    if (!(gpio_get_level(SWITCH1)) && !gpio_get_level(SWITCH2))
        KEY_State = 3;

    // ģʽ�ж�
    if (KEY_State == 0)
    {
        if (!gpio_get_level(KEY1))
        {
            system_delay_ms(1);
            if (!gpio_get_level(KEY1))
            {
                road_memery_start_Plus_flag = 1;
                while (!gpio_get_level(KEY1))
                    ;
            }
        }
        if (!gpio_get_level(KEY2))
        {
            system_delay_ms(1);
            if (!gpio_get_level(KEY2))
            {
                road_recurrent_Plus_flag = 1;
                while (!gpio_get_level(KEY2))
                    ;
            }
        }
        if (!gpio_get_level(KEY3))
        {
            system_delay_ms(1);
            if (!gpio_get_level(KEY3))
            {

                Run_Flag = 1;
                while (!gpio_get_level(KEY3))
                    ;
            }
        }
        if (!gpio_get_level(KEY4))
        {
            system_delay_ms(1);
            if (!gpio_get_level(KEY4))
            {

                flash_flag_Plus = 2;

                while (!gpio_get_level(KEY4))
                    ;
            }
        }
    }
    if (KEY_State == 3)
    {
        if (!gpio_get_level(KEY1))
        {
            system_delay_ms(1);
            if (!gpio_get_level(KEY1))
            {
                Display_Flag = 0; // �ر�ͼ����ʾ
                while (!gpio_get_level(KEY1))
                    ;
            }
        }
        if (!gpio_get_level(KEY2))
        {
            system_delay_ms(1);
            if (!gpio_get_level(KEY2))
            {

                Key_Flag = 0; // �رհ���
                while (!gpio_get_level(KEY2))
                    ;
            }
        }
        if (!gpio_get_level(KEY3))
        {
            system_delay_ms(1);
            if (!gpio_get_level(KEY3))
            {
                flash_flag_Plus = 1; // 0Ϊ��ʼ״̬��1Ϊ��ʼ�棬2Ϊ��ʼȡ��3Ϊ�����־��4Ϊȡ���־��
                while (!gpio_get_level(KEY3))
                    ;
            }
        }
    }

    if (KEY_State == 1)
    {
        if (!gpio_get_level(KEY1))
        {
            system_delay_ms(1);
            if (!gpio_get_level(KEY1))
            {

                while (!gpio_get_level(KEY1))
                    ;
            }
        }
        if (!gpio_get_level(KEY2))
        {
            system_delay_ms(1);
            if (!gpio_get_level(KEY2))
            {

                while (!gpio_get_level(KEY2))
                    ;
            }
        }
        if (!gpio_get_level(KEY3))
        {
            system_delay_ms(1);
            if (!gpio_get_level(KEY3))
            {

                while (!gpio_get_level(KEY3))
                    ;
            }
        }
    }
    if (KEY_State == 2)
    {
        if (!gpio_get_level(KEY1))
        {
            system_delay_ms(1);
            if (!gpio_get_level(KEY1))
            {

                while (!gpio_get_level(KEY1))
                    ;
            }
        }
        if (!gpio_get_level(KEY2))
        {
            system_delay_ms(1);
            if (!gpio_get_level(KEY2))
            {

                while (!gpio_get_level(KEY2))
                    ;
            }
        }
        if (!gpio_get_level(KEY3))
        {
            system_delay_ms(1);
            if (!gpio_get_level(KEY3))
            {

                while (!gpio_get_level(KEY3))
                    ;
            }
        }
        if (!gpio_get_level(KEY4))
        {
            system_delay_ms(1);
            if (!gpio_get_level(KEY4))
            {

                while (!gpio_get_level(KEY4))
                    ;
            }
        }
    }
}
