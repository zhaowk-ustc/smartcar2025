/*
 * control.h
 *
 *  Created on: 2024��10��24��
 *      Author: drama
 */

#ifndef CONTROL_H_
#define CONTROL_H_

/*************************************
 *
 *
 *           ��������������
 *
 *
 *************************************/

// ���������Ŷ���
#define ENCODER_DIR1 (TC_CH58_ENCODER)                 // �������ӿ�  TC_CH58_ENCODER
#define ENCODER_DIR1_PULSE (TC_CH58_ENCODER_CH1_P17_3) // PULSE ��Ӧ������  TC_CH58_ENCODER_CH1_P17_3
#define ENCODER_DIR1_DIR (TC_CH58_ENCODER_CH2_P17_4)   // DIR ��Ӧ������  TC_CH58_ENCODER_CH2_P17_4

#define ENCODER_DIR2 (TC_CH27_ENCODER)                 // �������ӿ�
#define ENCODER_DIR2_PULSE (TC_CH27_ENCODER_CH1_P19_2) // PULSE ��Ӧ������
#define ENCODER_DIR2_DIR (TC_CH27_ENCODER_CH2_P19_3)   // DIR ��Ӧ������

/*************************************
 *
 *
 *            ��������
 *
 *
 ************************************/
extern int Run_Flag; // ������־λ

void CodeInit(void); // ���������ʼ��
void Motor_control(float L_expect, float L_actual, float R_expect, float R_actual);
void Motor_control(float L_expect, float L_actual, float R_expect, float R_actual);
int16 Mid_filter(int16 data);

#endif /* CODE_CONTROL_H_ */
