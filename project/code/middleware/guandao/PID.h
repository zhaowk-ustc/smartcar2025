#ifndef PID_H__
#define PID_H__
//------------------------������--------------------------------//
// PID��ʼ�������ṹ��
typedef struct
{
	float expect; // ����ֵ
	float actual; // ʵ��ֵ
	float ek;	  // ���
	float ek1;	  // ��һ�����
	float ek2;	  // ���ϴ����
	float ec;	  // ���仯��

	float kp; // PID����kp
	float ki;
	float kd;

	float e_max; // ������ֵ
	float e_min;
	float ec_max;
	float ec_min;
	float kp_max;
	float kp_min;
	float ki_max;
	float ki_min;
	float kd_max;
	float kd_min;

	float output;
	float out_max;
	float out_min;
} PIDInit;

extern PIDInit pos_pd;	 // λ��ʽpid
extern PIDInit L_inc_pi; // ������ʽpid
extern PIDInit R_inc_pi; // ������ʽpid
extern int speed_diff;
//------------------------������--------------------------------//

//------------------------������--------------------------------//

void FuzzyPID_Init(PIDInit *pid, float KP_M, float KD_M); // λ��ʽpdԪ�ظ�ֵ
void PI_Init(PIDInit *pid, float kp, float ki, float kd); // piԪ�ظ�ֵ
float PD_control(PIDInit *PID, float kp, float kd);
float FuzzyPID_control(PIDInit *PID);
float PI_control(PIDInit *PID);
//------------------------��ֵ��--------------------------------//
#endif