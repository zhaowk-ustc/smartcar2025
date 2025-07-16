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

int16 Display_Flag = 1;

void show_data(void)
{
	ips200_show_string(0, 180, "L_Speed:");
	ips200_show_int(65, 180, speed_left, 3);
	ips200_show_string(0, 200, "R_Speed:");
	ips200_show_int(65, 200, speed_right, 3);

	ips200_show_string(0, 220, "l_out:");
	ips200_show_float(65, 220, l_out, 5, 1);
	ips200_show_string(0, 240, "r_out:");
	ips200_show_float(65, 240, r_out, 5, 1);

	ips200_show_string(0, 255, "err:");
	ips200_show_float(65, 255, pos_pd.ek, 3, 1);

	ips200_show_string(0, 270, "diff:");
	ips200_show_int(65, 270, speed_diff, 3);

	ips200_show_string(0, 285, "kp:");
	ips200_show_float(65, 285, pos_pd.kp, 3, 3);
	ips200_show_string(0, 300, "kd:");
	ips200_show_float(65, 300, pos_pd.kd, 3, 3);
}
void show_guandao(void)
{

	ips200_show_string(0, MT9V03X_H + 1, "R_M_F");
	ips200_show_uint(50, MT9V03X_H + 1, road_memery_finish_Plus_flag, 1);
	ips200_show_string(0, MT9V03X_H + 21, "R_M_S");
	ips200_show_uint(50, MT9V03X_H + 21, road_memery_start_Plus_flag, 1);
	ips200_show_string(0, MT9V03X_H + 41, "FLASH");
	ips200_show_uint(50, MT9V03X_H + 41, flash_flag_Plus, 1);
	ips200_show_string(100, MT9V03X_H + 41, "fu_xian");
	ips200_show_uint(160, MT9V03X_H + 41, road_recurrent_Plus_flag, 1);
	ips200_show_string(0, MT9V03X_H + 61, "NUM_L");
	ips200_show_uint(48, MT9V03X_H + 61, NUM_L_Plus, 4);
	ips200_show_string(0, MT9V03X_H + 81, "yaw");
	ips200_show_float(40, MT9V03X_H + 81, yaw, 4, 2);
	ips200_show_string(0, MT9V03X_H + 101, "yaw_Plus");
	ips200_show_float(80, MT9V03X_H + 101, yaw_plus, 4, 2);
	ips200_show_string(0, MT9V03X_H + 121, "err_guandao:");
	ips200_show_int(120, MT9V03X_H + 121, err_guandao_plus, 3);
	ips200_show_string(0, MT9V03X_H + 141, "locate_index:");
	ips200_show_int(120, MT9V03X_H + 141, locate_index, 3);
}
