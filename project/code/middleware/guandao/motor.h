#ifndef motor_H__
#define motor_H__

extern float l_out, r_out;

void motor_Init(void);
void encoder_Init(void); // ��������ʼ��
void motor_run(int16 L_duty, int16 R_duty);

#endif