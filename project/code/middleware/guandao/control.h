/*
 * control.h
 *
 *  Created on: 2024年10月24日
 *      Author: drama
 */

#ifndef CONTROL_H_
#define CONTROL_H_

/*************************************
 *
 *
 *           变量及常数定义
 *
 *
 *************************************/

// 编码器引脚定义
#define ENCODER_DIR1 (TC_CH58_ENCODER)                 // 编码器接口  TC_CH58_ENCODER
#define ENCODER_DIR1_PULSE (TC_CH58_ENCODER_CH1_P17_3) // PULSE 对应的引脚  TC_CH58_ENCODER_CH1_P17_3
#define ENCODER_DIR1_DIR (TC_CH58_ENCODER_CH2_P17_4)   // DIR 对应的引脚  TC_CH58_ENCODER_CH2_P17_4

#define ENCODER_DIR2 (TC_CH27_ENCODER)                 // 编码器接口
#define ENCODER_DIR2_PULSE (TC_CH27_ENCODER_CH1_P19_2) // PULSE 对应的引脚
#define ENCODER_DIR2_DIR (TC_CH27_ENCODER_CH2_P19_3)   // DIR 对应的引脚

/*************************************
 *
 *
 *            函数声明
 *
 *
 ************************************/
extern int Run_Flag; // 开车标志位

void CodeInit(void); // 所有外设初始化
void Motor_control(float L_expect, float L_actual, float R_expect, float R_actual);
void Motor_control(float L_expect, float L_actual, float R_expect, float R_actual);
int16 Mid_filter(int16 data);

#endif /* CODE_CONTROL_H_ */
