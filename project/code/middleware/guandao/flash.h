/*
 * flash.h
 *
 *  Created on: 2024��12��30��
 *      Author: ������
 */

#ifndef CODE_FLASH_H_
#define CODE_FLASH_H_

#include "zf_common_headfile.h"

void flash_road_memery_store(void); // ��·������
void flash_road_memery_get(void);   // ȡ·������

void flash_road_memery_store_Plus(void);
void flash_road_memery_get_Plus(void);

extern uint8_t flash_flag;      // 0Ϊ��ʼ״̬��1Ϊ��ʼ�棬2Ϊ��ʼȡ��3Ϊ�����־��4Ϊȡ���־��
extern uint8_t flash_flag_Plus; // 0Ϊ��ʼ״̬��1Ϊ��ʼ�棬2Ϊ��ʼȡ��3Ϊ�����־��4Ϊȡ���־��

#define Y_memery_page_INDEX_6 (6) // y���ר������

#define X_memery_page_INDEX_7 (7)   // ����X����ר������
#define X_memery_page_INDEX_9 (9)   // ����X����ר������
#define X_memery_page_INDEX_13 (13) // ����X����ר������

#define Y_memery_page_INDEX_8 (8)   // ����Y����ר������
#define Y_memery_page_INDEX_10 (10) // ����Y����ר������
#define Y_memery_page_INDEX_12 (12) // ����Y����ר������

#endif /* CODE_FLASH_H_ */
