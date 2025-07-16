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
/* 有刷电机1 */
#define DIR_R (P09_1)
#define PWM_R (TCPWM_CH24_P09_0)
#define DIR_L (P10_3)
#define PWM_L (TCPWM_CH30_P10_2)

#define Duty_MAX 5000
#define Duty_MIN -5000

float l_out, r_out;
// 电机初始化
void motor_Init(void)
{
	gpio_init(DIR_L, GPO, 1, GPO_PUSH_PULL);
	pwm_init(PWM_L, 17 * 1000, 0);
	gpio_init(DIR_R, GPO, 1, GPO_PUSH_PULL);
	pwm_init(PWM_R, 17 * 1000, 0);
}
// 电机设置占空比输出
void motor_run(int16 L_duty, int16 R_duty)
{
	if (L_duty >= Duty_MAX)
	{
		L_duty = Duty_MAX;
	}
	else if (L_duty <= Duty_MIN)
	{
		L_duty = Duty_MIN;
	}
	if (R_duty >= Duty_MAX)
	{
		R_duty = Duty_MAX;
	}
	else if (R_duty <= Duty_MIN)
	{
		R_duty = Duty_MIN;
	}
	// 左轮
	if (L_duty >= 0)
	{
		gpio_set_level(DIR_L, 1);
		pwm_set_duty(PWM_L, L_duty);
	}
	else
	{
		gpio_set_level(DIR_L, 0);
		pwm_set_duty(PWM_L, -L_duty);
	}
	// 右轮
	if (R_duty >= 0)
	{
		gpio_set_level(DIR_R, 1);
		pwm_set_duty(PWM_R, R_duty);
	}
	else
	{
		gpio_set_level(DIR_R, 0);
		pwm_set_duty(PWM_R, -R_duty);
	}
}

/*
----------------------------------------------------------------------------------------------------------------
// 函数名称 encoder_Init
// 函数简介 编码器初始化
// 参数说明
// 返回参数
// 使用示例 encoder_Init();
// 备注信息
//----------------------------------------------------------------------------------------------------------------*/

void encoder_Init(void)
{
	encoder_dir_init(ENCODER_DIR1, ENCODER_DIR1_PULSE, ENCODER_DIR1_DIR); // 初始化编码器模块与引脚 带方向增量编码器模式
	encoder_dir_init(ENCODER_DIR2, ENCODER_DIR2_PULSE, ENCODER_DIR2_DIR); // 初始化编码器模块与引脚 带方向增量编码器模式
	encoder_clear_count(ENCODER_DIR1);
	encoder_clear_count(ENCODER_DIR2); // 清除左右轮计数
}