/*
 * key.c
 *
 *  Created on: 2024年10月30日
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

// 按键初始化
void KEY_Init(void)
{
    gpio_init(KEY1, GPI, GPIO_HIGH, GPI_PULL_UP); // 初始化 KEY1 输入 默认高电平 上拉输入
    gpio_init(KEY2, GPI, GPIO_HIGH, GPI_PULL_UP); // 初始化 KEY2 输入 默认高电平 上拉输入
    gpio_init(KEY3, GPI, GPIO_HIGH, GPI_PULL_UP); // 初始化 KEY3 输入 默认高电平 上拉输入
    gpio_init(KEY4, GPI, GPIO_HIGH, GPI_PULL_UP); // 初始化 KEY4 输入 默认高电平 上拉输入

    gpio_init(SWITCH1, GPI, GPIO_HIGH, GPI_PULL_UP); // 初始化 SWITCH1 输入 默认高电平 上拉输入
    gpio_init(SWITCH2, GPI, GPIO_HIGH, GPI_PULL_UP); // 初始化 SWITCH2 输入 默认高电平 上拉输入
}

// 按键扫描
void KEY_Scan(void)
{
    // 判断编码开关状态
    if (gpio_get_level(SWITCH1) && gpio_get_level(SWITCH2))
        KEY_State = 0;
    if (!(gpio_get_level(SWITCH1)) && gpio_get_level(SWITCH2))
        KEY_State = 1;
    if (gpio_get_level(SWITCH1) && !(gpio_get_level(SWITCH2)))
        KEY_State = 2;
    if (!(gpio_get_level(SWITCH1)) && !gpio_get_level(SWITCH2))
        KEY_State = 3;

    // 模式判断
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
                Display_Flag = 0; // 关闭图像显示
                while (!gpio_get_level(KEY1))
                    ;
            }
        }
        if (!gpio_get_level(KEY2))
        {
            system_delay_ms(1);
            if (!gpio_get_level(KEY2))
            {

                Key_Flag = 0; // 关闭按键
                while (!gpio_get_level(KEY2))
                    ;
            }
        }
        if (!gpio_get_level(KEY3))
        {
            system_delay_ms(1);
            if (!gpio_get_level(KEY3))
            {
                flash_flag_Plus = 1; // 0为初始状态，1为开始存，2为开始取，3为存完标志，4为取完标志；
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
