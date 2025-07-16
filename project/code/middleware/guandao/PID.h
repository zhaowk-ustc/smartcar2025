#ifndef PID_H__
#define PID_H__
//------------------------变量区--------------------------------//
// PID初始化参数结构体
typedef struct
{
	float expect; // 期望值
	float actual; // 实际值
	float ek;	  // 误差
	float ek1;	  // 上一次误差
	float ek2;	  // 上上次误差
	float ec;	  // 误差变化率

	float kp; // PID参数kp
	float ki;
	float kd;

	float e_max; // 误差最大值
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

extern PIDInit pos_pd;	 // 位置式pid
extern PIDInit L_inc_pi; // 左增量式pid
extern PIDInit R_inc_pi; // 右增量式pid
extern int speed_diff;
//------------------------变量区--------------------------------//

//------------------------函数区--------------------------------//

void FuzzyPID_Init(PIDInit *pid, float KP_M, float KD_M); // 位置式pd元素赋值
void PI_Init(PIDInit *pid, float kp, float ki, float kd); // pi元素赋值
float PD_control(PIDInit *PID, float kp, float kd);
float FuzzyPID_control(PIDInit *PID);
float PI_control(PIDInit *PID);
//------------------------赋值区--------------------------------//
#endif