#include "PID.h"

PIDInit pos_pd;	  // 位置式pid
PIDInit L_inc_pi; // 左增量式pid
PIDInit R_inc_pi; // 右

int speed_diff = 0;

static float e_membership_values[7] = {-3, -2, -1, 0, 1, 2, 3};	 // 输入e的隶属值
static float ec_membership_values[7] = {-3, -2, -1, 0, 1, 2, 3}; // 输入de/dt的隶属值
static float kp_membership_values[7] = {-3, -2, -1, 0, 1, 2, 3}; // 输入kp的隶属值
// static float ki_membership_values[7] = {-3,-2,-1,0,1,2,3};  //输入ki的隶属值
static float kd_membership_values[7] = {-3, -2, -1, 0, 1, 2, 3}; // 输入kd的隶属值

float q_e;					// 输入e对应论域中的值
float q_ec;					// 输入de/dt对应论域中的值
float e_gradmembership[2];	// 输入e的隶属度
float ec_gradmembership[2]; // 输入de/dt的隶属度
int e_grad_index[2];		// 输入e隶属值在规则表的索引
int ec_grad_index[2];		// 输入de/dt隶属值在规则表的索引

float q_inc_kp; // 增量kp对应论域中的值
// float q_inc_ki;
float q_inc_kd;
float inc_kp; // 输出增量kp
// float inc_ki;
float inc_kd;

float kpsum[7] = {0, 0, 0, 0, 0, 0, 0}; // 输出kp总的隶属度
// float kisum[7] = {0, 0, 0, 0, 0, 0, 0};
float kdsum[7] = {0, 0, 0, 0, 0, 0, 0};
float gradsum[7] = {0, 0, 0, 0, 0, 0, 0};

// 论域隶属值
#define NB -3
#define NM -2
#define NS -1
#define ZO 0
#define PS 1
#define PM 2
#define PB 3

const int Kp_rule_list[7][7] =
	{
		{PB, PB, PM, PM, PS, ZO, ZO}, // kp规则表
		{PB, PB, PM, PS, PS, ZO, NS},
		{PM, PM, PM, PS, ZO, NS, NS},
		{PM, PM, PS, ZO, NS, NM, NM},
		{PS, PS, ZO, NS, NS, NM, NM},
		{PS, ZO, NS, NM, NM, NM, NB},
		{ZO, ZO, NM, NM, NM, NB, NB}};
const int Ki_rule_list[7][7] =
	{
		{NB, NB, NM, NM, NS, ZO, ZO}, // ki规则表
		{NB, NB, NM, NS, NS, ZO, ZO},
		{NB, NM, NS, NS, ZO, PS, PS},
		{NM, NM, NS, ZO, PS, PM, PM},
		{NM, NS, ZO, PS, PS, PM, PB},
		{ZO, ZO, PS, PS, PM, PB, PB},
		{ZO, ZO, PS, PM, PM, PB, PB}};
const int Kd_rule_list[7][7] =
	{
		{PS, NS, NB, NB, NB, NM, PS}, // kd规则表
		{PS, NS, NB, NM, NM, NS, ZO},
		{ZO, NS, NM, NM, NS, NS, ZO},
		{ZO, NS, NS, NS, NS, NS, ZO},
		{ZO, ZO, ZO, ZO, ZO, ZO, ZO},
		{PB, NS, PS, PS, PS, PS, PB},
		{PB, PM, PM, PM, PS, PS, PB}};

// 映射函数,将e，ec映射在论域
float Quantization(float xmax, float xmin, float x)
{
	if (x > xmax)
	{
		x = xmax;
	}
	if (x < xmin)
	{
		x = xmin;
	}
	float q_value = 6 * (xmax - x) / (xmax - xmin) - 3;
	return q_value;
}

// 反映射函数
float Inverse_Quantization(float xmax, float xmin, float q_value)
{
	float x = (q_value + 3) * (xmax - xmin) / 6 + xmin;
	return x;
}
// 计算隶属度函数,求e，ec在规则表中的位置
void Get_Gradmembership(float ek, float ec)
{
	// ek
	if (ek > e_membership_values[0] && ek < e_membership_values[6])
	{
		for (int i = 0; i < 5; i++)
		{
			if (ek > e_membership_values[i] && ek < e_membership_values[i + 1])
			{
				e_gradmembership[0] = -(ek - e_membership_values[i + 1]) / (e_membership_values[i + 1] - e_membership_values[i]);
				e_gradmembership[1] = 1 + (ek - e_membership_values[i + 1]) / (e_membership_values[i + 1] - e_membership_values[i]);
				e_grad_index[0] = i;
				e_grad_index[1] = i + 1;
				break;
			}
		}
	}
	else
	{
		if (ek < e_membership_values[0])
		{
			e_gradmembership[0] = 1;
			e_gradmembership[1] = 0;
			e_grad_index[0] = 0;
			e_grad_index[1] = -1;
		}
		else if (ek > e_membership_values[6])
		{
			e_gradmembership[0] = 1;
			e_gradmembership[1] = 0;
			e_grad_index[0] = 6;
			e_grad_index[1] = -1;
		}
	}
	// ec
	if (ec > ec_membership_values[0] && ec < ec_membership_values[6])
	{
		for (int i = 0; i < 5; i++)
		{
			if (ec > ec_membership_values[i] && ec < ec_membership_values[i + 1])
			{
				ec_gradmembership[0] = -(ec - ec_membership_values[i + 1]) / (ec_membership_values[i + 1] - ec_membership_values[i]);
				ec_gradmembership[1] = 1 + (ec - ec_membership_values[i + 1]) / (ec_membership_values[i + 1] - ec_membership_values[i]);
				ec_grad_index[0] = i;
				ec_grad_index[1] = i + 1;
				break;
			}
		}
	}
	else
	{
		if (ec < ec_membership_values[0])
		{
			ec_gradmembership[0] = 1;
			ec_gradmembership[1] = 0;
			ec_grad_index[0] = 0;
			ec_grad_index[1] = -1;
		}
		else if (ec > ec_membership_values[6])
		{
			ec_gradmembership[0] = 1;
			ec_gradmembership[1] = 0;
			ec_grad_index[0] = 6;
			ec_grad_index[1] = -1;
		}
	}
}

// 获取输出kp，ki,kd的总隶属度
void Get_SumGrad()
{
	for (int i = 0; i <= 6; i++)
	{
		kpsum[i] = 0;
		//		kisum[i]=0;
		kdsum[i] = 0;
	}
	for (int i = 0; i < 2; i++)
	{
		if (e_grad_index[i] == -1)
		{
			continue;
		}
		for (int j = 0; j < 2; j++)
		{
			if (e_grad_index[j] != -1)
			{
				int indexKp = Kp_rule_list[e_grad_index[i]][ec_grad_index[j]] + 3;
				//				int indexKi = Ki_rule_list[e_grad_index[i]][ec_grad_index[j]] + 3;
				int indexKd = Kd_rule_list[e_grad_index[i]][ec_grad_index[j]] + 3;

				kpsum[indexKp] = kpsum[indexKp] + e_gradmembership[i] * ec_gradmembership[j];
				//				kisum[indexKi] = kpsum[indexKi] + e_gradmembership[i] *ec_gradmembership[j];
				kdsum[indexKd] = kpsum[indexKd] + e_gradmembership[i] * ec_gradmembership[j];
			}
			else
			{
				continue;
			}
		}
	}
}

// 计算输出增量kp,ki,kd对应论域值
void Get_Out()
{
	for (int i = 0; i < 6; i++)
	{
		q_inc_kp += kp_membership_values[i] * kpsum[i];
		//		q_inc_ki += ki_membership_values[i] * kisum[i];
		q_inc_kd += kd_membership_values[i] * kdsum[i];
	}
}

// PI参数初始化
void PI_Init(PIDInit *pid, float kp, float ki, float kd)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;

	//	pid->kp_max = 20;
	//	pid->kp_min = 0;
	//	pid->ki_max = 1;
	//	pid->ki_min = 0;
	//	pid->kd_max = 0;
	//	pid->kd_min = 0;

	pid->out_max = 5000;
	pid->out_min = 0;
}

// 模糊PID参数初始化
void FuzzyPID_Init(PIDInit *pid, float KP_M, float KD_M)
{
	pid->kp = 0;
	//	pid->ki = ki;
	pid->kd = 0;

	pid->e_max = 40;
	pid->e_min = -40;
	pid->ec_max = 20;
	pid->ec_min = -20;

	pid->kp_max = KP_M;
	pid->kp_min = -KP_M;
	//	pid->ki_max = 1;
	//	pid->ki_min = 0;
	pid->kd_max = KD_M;
	pid->kd_min = -KD_M;

	pid->out_max = 100;
	pid->out_min = -100;
}

// 模糊PID实现函数
float FuzzyPID_control(PIDInit *PID)
{
	PID->ek1 = PID->ek;
	PID->ek = PID->expect - PID->actual;
	PID->ec = PID->ek - PID->ek1;

	// 映射
	q_e = Quantization(PID->e_max, PID->e_min, PID->ek);
	q_ec = Quantization(PID->ec_max, PID->ec_min, PID->ec);
	// 计算隶属度
	Get_Gradmembership(q_e, q_ec);
	// 计算kp，ki，kd分别总的隶属度
	Get_SumGrad();
	// 计算kp,ki,kd对应的隶属值
	Get_Out();

	inc_kp = Inverse_Quantization(PID->kp_max, PID->kp_min, q_inc_kp);
	//	inc_ki = Inverse_Quantization(PID->ki_max,PID->ki_min,q_inc_ki);
	inc_kd = Inverse_Quantization(PID->kd_max, PID->kd_min, q_inc_kd);
	q_inc_kp = 0;
	//	q_inc_ki = 0;
	q_inc_kd = 0;

	PID->kp = PID->kp + inc_kp;
	//	PID->ki = PID->ki + inc_ki;
	PID->ki = PID->ki + inc_kd;
	if (PID->kp < 0)
		PID->kp = 0;
	//	if(PID->ki <0 )
	//		PID->ki = 0;
	if (PID->kd < 0)
		PID->kd = 0;
	inc_kp = 0;
	//	inc_ki = 0;
	inc_kd = 0;

	//	float output = PID->kp * (PID->ek - PID->ek1) + PID->ki * PID->ek +PID->kd *(PID->ek - 2*PID->ek1+PID->ek2);

	PID->output = PID->kp * PID->ek + PID->kd * (PID->ek - PID->ek1);

	if (PID->output > PID->out_max)
	{
		PID->output = PID->out_max;
	}
	if (PID->output < PID->out_min)
	{
		PID->output = PID->out_min;
	}
	return PID->output;
}

float PD_control(PIDInit *PID, float kp, float kd)
{

	PID->kp = kp;
	PID->kd = kd;

	PID->ek1 = PID->ek;
	PID->ek = PID->expect - PID->actual;
	PID->ec = PID->ek - PID->ek1;

	PID->output = PID->kp * PID->ek + PID->kd * (PID->ek - PID->ek1);

	//	if (PID->output > PID->out_max)
	//	{
	//		PID->output = PID->out_max;
	//	}
	//	if (PID->output < PID->out_min)
	//	{
	//		PID->output = PID->out_min;
	//	}
	return PID->output;
}

float PI_control(PIDInit *PID)
{

	PID->ek1 = PID->ek;
	PID->ek = PID->expect - PID->actual;

	float out_increment = PID->kp * (PID->ek - PID->ek1) + PID->ki * PID->ek;
	PID->output += out_increment;

	if (PID->output > PID->out_max)
	{
		PID->output = PID->out_max;
	}
	if (PID->output < PID->out_min)
	{
		PID->output = PID->out_min;
	}

	return PID->output;
}