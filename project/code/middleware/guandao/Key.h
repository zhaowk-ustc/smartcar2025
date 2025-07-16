/*
 * key.h
 *
 *  Created on: 2024Äê10ÔÂ30ÈÕ
 *      Author: 86152
 */

#ifndef CODE_KEY_H_
#define CODE_KEY_H_

#include "zf_common_headfile.h"

#define KEY1 (P20_0)
#define KEY2 (P20_1)
#define KEY3 (P20_2)
#define KEY4 (P20_3)

#define SWITCH1 (P21_5)
#define SWITCH2 (P21_6)

extern int16 KEY_State;
extern int16 Key_Flag;

void KEY_Init(void);
void KEY_Scan(void);

#endif /* CODE_KEY_H_ */
