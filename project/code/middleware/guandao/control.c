/*
 * control.c
 *
 *  Created on: 2024年10月24日
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
// 函数名称 CodeInit
// 函数简介 全局外设初始化
// 参数说明
// 返回参数
// 使用示例 CodeInit();
// 备注信息
//----------------------------------------------------------------------------------------------------------------
uint8_t count = 0;
int Run_Flag = 0; // 开车标志位
int MID_W = 94;   // 中线标准值
void CodeInit(void)
{
    encoder_Init();         // 编码器初始化
    motor_Init();           // 电机初始化
    imu660ra_init();        // 姿态传感器初始化
    gyro_zero_param_init(); // 标定零飘（新的）
    acc_zero_param_init();
    flash_init();

    //    ips200_init(IPS200_TYPE_SPI);
    // IPS200_Cammer_Init();       //摄像头初始化
    // ips200_full(RGB565_BLACK); // 屏幕初始化
    ips200_clear();
    ips200_init(IPS200_TYPE_SPI);
    ips200_full(RGB565_BLACK); // 屏幕初始化

    KEY_Init(); // 按键初始化

    pit_ms_init(PIT_CH0, 5);    // 电机5ms中断
    pit_ms_init(PIT_CH1, 5);    // 陀螺仪5ms中断
    pit_ms_init(PIT_CH2, 2000); //

    // 初始化滤波器（截止频率15Hz，采样时间0.005秒）
    LPF_InitByFrequency(&velocity_filter, 15.0f, 0.005f);

    //    FuzzyPID_Init(&pos_pd,2,0.01); // 位置式pd结构体初始化赋值
    PI_Init(&L_inc_pi, 11, 0.1, 0); // 左增量式pid结构体   11   0.1
    PI_Init(&R_inc_pi, 11, 0.1, 0); //  右增量式pid结构体   11   0.1
}

void Motor_control(float L_expect, float L_actual, float R_expect, float R_actual)
{
    // pos_pd.expect = MID_W;              // 理论中线值
    // pos_pd.actual = Mid_Line_Calculate; // 实际中线值

    pos_pd.expect = 0.0f;  // 期望惯导横向误差为0
    pos_pd.actual = e_lat; // 当前实际惯导横向误差

    //    speed_diff = (int)FuzzyPID_control(&pos_pd);
    speed_diff = err_guandao_plus; //-(int) PD_control(&pos_pd, 1.0, 0.05)的负号表示取反

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
    } // 到达路径终点自动停车

    motor_run((int16)l_out, (int16)r_out);
}

#define MID_FILTER_SIEZ (21) // 选取的数据量
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