/*
 * flash.c
 *
 *  Created on: 2024��12��30��
 *      Author: ������
 */

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

#define FLASH_SECTION_INDEX (0)     // �洢�����õ�����
#define road_memery_page_INDEX (11) // ·�����ר������

#define X_memery_page_INDEX_1 (1) // x���ר������
#define X_memery_page_INDEX_3 (3) // x���ר������
#define X_memery_page_INDEX_5 (5) // x���ר������

#define Y_memery_page_INDEX_2 (2) // y���ר������
#define Y_memery_page_INDEX_4 (4) // y���ר������
#define Y_memery_page_INDEX_6 (6) // y���ר������

#define X_memery_page_INDEX_7 (7)   // ����X����ר������
#define X_memery_page_INDEX_9 (9)   // ����X����ר������
#define X_memery_page_INDEX_13 (13) // ����X����ר������

#define Y_memery_page_INDEX_8 (8)   // ����Y����ר������
#define Y_memery_page_INDEX_10 (10) // ����Y����ר������
#define Y_memery_page_INDEX_12 (12) // ����Y����ר������

uint8_t flash_flag = 0;      // 0Ϊ��ʼ״̬��1Ϊ��ʼ�棬2Ϊ��ʼȡ��3Ϊ�����־��4Ϊȡ���־��
uint8_t flash_flag_Plus = 0; // 0Ϊ��ʼ״̬��1Ϊ��ʼ�棬2Ϊ��ʼȡ��3Ϊ�����־��4Ϊȡ���־��
// ��·������
void flash_road_memery_store(void)
{
    // ��һ�����������
    flash_buffer_clear();
    // �ڶ��������ݴ浽������
    for (size_t i = 0; i < FLASH_PAGE_LENGTH - 1; i++)
    {

        flash_union_buffer[i].float_type = yaw_memery[i]; // �洢ƫ����
    }

    // �������ж�FLASH����û�����ݣ��оͰ�FLASH���ݲ���/�����ݴ浽������
    if (flash_check(FLASH_SECTION_INDEX, road_memery_page_INDEX))
    { // �ж��Ƿ�������
        flash_erase_page(FLASH_SECTION_INDEX, road_memery_page_INDEX);
    } // ������һҳ
    // ���Ĳ��浽FLASHָ������
    flash_write_page_from_buffer(FLASH_SECTION_INDEX, road_memery_page_INDEX, FLASH_PAGE_LENGTH - 1);
    // ��ȡ���̽��� ���� �ɹ���ȡ
    flash_flag = 3;
}

// ȡ·������
void flash_road_memery_get(void)
{
    if (flash_check(FLASH_SECTION_INDEX, road_memery_page_INDEX)) // ���flash�����Ƿ��������
    {
        flash_read_page_to_buffer(FLASH_SECTION_INDEX, road_memery_page_INDEX, FLASH_PAGE_LENGTH - 1);
        for (size_t i = 0; i < FLASH_PAGE_LENGTH - 1; i++)
        {
            yaw_store[i] = flash_union_buffer[i].float_type; // ȡ����ʷƫ����
        }
    }
    flash_flag = 4;
}

void flash_road_memery_store_Plus(void)
{

    /*��x�����ݷ������������δ�*/

    // ��һ�����������
    flash_buffer_clear();
    // �ڶ��������ݴ浽������
    for (size_t i = 0; i < FLASH_PAGE_LENGTH; i++)
    {
        flash_union_buffer[i].float_type = X_Memery_Plus[i];
    }

    // �������ж�FLASH����û�����ݣ��оͰ�FLASH���ݲ���/�����ݴ浽������
    if (flash_check(FLASH_SECTION_INDEX, X_memery_page_INDEX_1))
    { // �ж��Ƿ�������
        flash_erase_page(FLASH_SECTION_INDEX, X_memery_page_INDEX_1);
    } // ������һҳ
    // ���Ĳ��浽FLASHָ������
    flash_write_page_from_buffer(FLASH_SECTION_INDEX, X_memery_page_INDEX_1, FLASH_PAGE_LENGTH);

    // ��һ�����������
    flash_buffer_clear();
    // �ڶ��������ݴ浽������
    for (size_t i = FLASH_PAGE_LENGTH, j = 0; i < 2 * FLASH_PAGE_LENGTH; i++, j++)
    {
        flash_union_buffer[j].float_type = X_Memery_Plus[i];
    }

    // �������ж�FLASH����û�����ݣ��оͰ�FLASH���ݲ���/�����ݴ浽������
    if (flash_check(FLASH_SECTION_INDEX, X_memery_page_INDEX_3))
    { // �ж��Ƿ�������
        flash_erase_page(FLASH_SECTION_INDEX, X_memery_page_INDEX_3);
    } // ������һҳ
    // ���Ĳ��浽FLASHָ������
    flash_write_page_from_buffer(FLASH_SECTION_INDEX, X_memery_page_INDEX_3, FLASH_PAGE_LENGTH);

    // ��һ�����������
    flash_buffer_clear();
    // �ڶ��������ݴ浽������
    for (size_t i = 2 * FLASH_PAGE_LENGTH, j = 0; i < 3 * FLASH_PAGE_LENGTH; i++, j++)
    {
        flash_union_buffer[j].float_type = X_Memery_Plus[i];
    }

    // �������ж�FLASH����û�����ݣ��оͰ�FLASH���ݲ���/�����ݴ浽������
    if (flash_check(FLASH_SECTION_INDEX, X_memery_page_INDEX_5))
    { // �ж��Ƿ�������
        flash_erase_page(FLASH_SECTION_INDEX, X_memery_page_INDEX_5);
    } // ������һҳ
    // ���Ĳ��浽FLASHָ������
    flash_write_page_from_buffer(FLASH_SECTION_INDEX, X_memery_page_INDEX_5, FLASH_PAGE_LENGTH);

    // ����X�����ݴ洢������ 7
    flash_buffer_clear();
    for (size_t i = 3 * FLASH_PAGE_LENGTH, j = 0; i < 4 * FLASH_PAGE_LENGTH; i++, j++)
    {
        flash_union_buffer[j].float_type = X_Memery_Plus[i];
    }
    if (flash_check(FLASH_SECTION_INDEX, X_memery_page_INDEX_7))
    {
        flash_erase_page(FLASH_SECTION_INDEX, X_memery_page_INDEX_7);
    }
    flash_write_page_from_buffer(FLASH_SECTION_INDEX, X_memery_page_INDEX_7, FLASH_PAGE_LENGTH);

    // ����X�����ݴ洢������ 9
    flash_buffer_clear();
    for (size_t i = 4 * FLASH_PAGE_LENGTH, j = 0; i < 5 * FLASH_PAGE_LENGTH; i++, j++)
    {
        flash_union_buffer[j].float_type = X_Memery_Plus[i];
    }
    if (flash_check(FLASH_SECTION_INDEX, X_memery_page_INDEX_9))
    {
        flash_erase_page(FLASH_SECTION_INDEX, X_memery_page_INDEX_9);
    }
    flash_write_page_from_buffer(FLASH_SECTION_INDEX, X_memery_page_INDEX_9, FLASH_PAGE_LENGTH);

    // ����X�����ݴ洢������ 13
    flash_buffer_clear();
    for (size_t i = 5 * FLASH_PAGE_LENGTH, j = 0; i < 6 * FLASH_PAGE_LENGTH; i++, j++)
    {
        flash_union_buffer[j].float_type = X_Memery_Plus[i];
    }
    // �����һҳ�����һ��λ�ô洢NUM_L_Plus�����ڼ�¼ʵ�����ݳ��ȼ�·���յ�
    flash_union_buffer[FLASH_PAGE_LENGTH - 1].uint16_type = NUM_L_Plus;
    if (flash_check(FLASH_SECTION_INDEX, X_memery_page_INDEX_13))
    {
        flash_erase_page(FLASH_SECTION_INDEX, X_memery_page_INDEX_13);
    }
    flash_write_page_from_buffer(FLASH_SECTION_INDEX, X_memery_page_INDEX_13, FLASH_PAGE_LENGTH);

    /*��y������  �������������δ�*/

    // ��һ�����������
    flash_buffer_clear();
    // �ڶ��������ݴ浽������
    for (size_t i = 0; i < FLASH_PAGE_LENGTH; i++)
    {
        flash_union_buffer[i].float_type = Y_Memery_Plus[i];
    }

    // �������ж�FLASH����û�����ݣ��оͰ�FLASH���ݲ���/�����ݴ浽������
    if (flash_check(FLASH_SECTION_INDEX, Y_memery_page_INDEX_2))
    { // �ж��Ƿ�������
        flash_erase_page(FLASH_SECTION_INDEX, Y_memery_page_INDEX_2);
    } // ������һҳ
    // ���Ĳ��浽FLASHָ������
    flash_write_page_from_buffer(FLASH_SECTION_INDEX, Y_memery_page_INDEX_2, FLASH_PAGE_LENGTH);

    // ��һ�����������
    flash_buffer_clear();
    // �ڶ��������ݴ浽������
    for (size_t i = FLASH_PAGE_LENGTH, j = 0; i < 2 * FLASH_PAGE_LENGTH; i++, j++)
    {
        flash_union_buffer[j].float_type = Y_Memery_Plus[i];
    }

    // �������ж�FLASH����û�����ݣ��оͰ�FLASH���ݲ���/�����ݴ浽������
    if (flash_check(FLASH_SECTION_INDEX, Y_memery_page_INDEX_4))
    { // �ж��Ƿ�������
        flash_erase_page(FLASH_SECTION_INDEX, Y_memery_page_INDEX_4);
    } // ������һҳ
    // ���Ĳ��浽FLASHָ������
    flash_write_page_from_buffer(FLASH_SECTION_INDEX, Y_memery_page_INDEX_4, FLASH_PAGE_LENGTH);

    // ��һ�����������
    flash_buffer_clear();
    // �ڶ��������ݴ浽������
    for (size_t i = 2 * FLASH_PAGE_LENGTH, j = 0; i < 3 * FLASH_PAGE_LENGTH; i++, j++)
    {
        flash_union_buffer[j].float_type = Y_Memery_Plus[i];
    }

    // �������ж�FLASH����û�����ݣ��оͰ�FLASH���ݲ���/�����ݴ浽������
    if (flash_check(FLASH_SECTION_INDEX, Y_memery_page_INDEX_6))
    { // �ж��Ƿ�������
        flash_erase_page(FLASH_SECTION_INDEX, Y_memery_page_INDEX_6);
    } // ������һҳ
    // ���Ĳ��浽FLASHָ������
    flash_write_page_from_buffer(FLASH_SECTION_INDEX, Y_memery_page_INDEX_6, FLASH_PAGE_LENGTH);

    // ����Y�����ݴ洢������ 8
    flash_buffer_clear();
    for (size_t i = 3 * FLASH_PAGE_LENGTH, j = 0; i < 4 * FLASH_PAGE_LENGTH; i++, j++)
    {
        flash_union_buffer[j].float_type = Y_Memery_Plus[i];
    }
    if (flash_check(FLASH_SECTION_INDEX, Y_memery_page_INDEX_8))
    {
        flash_erase_page(FLASH_SECTION_INDEX, Y_memery_page_INDEX_8);
    }
    flash_write_page_from_buffer(FLASH_SECTION_INDEX, Y_memery_page_INDEX_8, FLASH_PAGE_LENGTH);

    // ����Y�����ݴ洢������ 10
    flash_buffer_clear();
    for (size_t i = 4 * FLASH_PAGE_LENGTH, j = 0; i < 5 * FLASH_PAGE_LENGTH; i++, j++)
    {
        flash_union_buffer[j].float_type = Y_Memery_Plus[i];
    }
    if (flash_check(FLASH_SECTION_INDEX, Y_memery_page_INDEX_10))
    {
        flash_erase_page(FLASH_SECTION_INDEX, Y_memery_page_INDEX_10);
    }
    flash_write_page_from_buffer(FLASH_SECTION_INDEX, Y_memery_page_INDEX_10, FLASH_PAGE_LENGTH);

    // ����Y�����ݴ洢������ 12
    flash_buffer_clear();
    for (size_t i = 5 * FLASH_PAGE_LENGTH, j = 0; i < 6 * FLASH_PAGE_LENGTH; i++, j++)
    {
        flash_union_buffer[j].float_type = Y_Memery_Plus[i];
    }
    if (flash_check(FLASH_SECTION_INDEX, Y_memery_page_INDEX_12))
    {
        flash_erase_page(FLASH_SECTION_INDEX, Y_memery_page_INDEX_12);
    }
    flash_write_page_from_buffer(FLASH_SECTION_INDEX, Y_memery_page_INDEX_12, FLASH_PAGE_LENGTH);

    // ��ȡ���̽��� ���� �ɹ���ȡ
    flash_flag_Plus = 3;
}

void flash_road_memery_get_Plus(void)
{

    /*ȡx������,����������ȡ*/
    if (flash_check(FLASH_SECTION_INDEX, X_memery_page_INDEX_1)) // ���flash�����Ƿ��������
    {
        flash_read_page_to_buffer(FLASH_SECTION_INDEX, X_memery_page_INDEX_1, FLASH_PAGE_LENGTH);
        for (size_t i = 0; i < FLASH_PAGE_LENGTH; i++)
        {
            X_Memery_Store_Plus[i] = flash_union_buffer[i].float_type; //
        }
    }

    /*ȡx������*/
    if (flash_check(FLASH_SECTION_INDEX, X_memery_page_INDEX_3)) // ���flash�����Ƿ��������
    {
        flash_read_page_to_buffer(FLASH_SECTION_INDEX, X_memery_page_INDEX_3, FLASH_PAGE_LENGTH);
        for (size_t i = 0, j = FLASH_PAGE_LENGTH; i < FLASH_PAGE_LENGTH; i++, j++)
        {
            X_Memery_Store_Plus[j] = flash_union_buffer[i].float_type; //
        }
    }

    /*ȡx������*/
    if (flash_check(FLASH_SECTION_INDEX, X_memery_page_INDEX_5)) // ���flash�����Ƿ��������
    {
        flash_read_page_to_buffer(FLASH_SECTION_INDEX, X_memery_page_INDEX_5, FLASH_PAGE_LENGTH);
        for (size_t i = 0, j = 2 * FLASH_PAGE_LENGTH; i < FLASH_PAGE_LENGTH; i++, j++)
        {
            X_Memery_Store_Plus[j] = flash_union_buffer[i].float_type; //
        }
    }

    // ����X�����ݶ�ȡ������ 7
    if (flash_check(FLASH_SECTION_INDEX, X_memery_page_INDEX_7))
    {
        flash_read_page_to_buffer(FLASH_SECTION_INDEX, X_memery_page_INDEX_7, FLASH_PAGE_LENGTH);
        for (size_t i = 0, j = 3 * FLASH_PAGE_LENGTH; i < FLASH_PAGE_LENGTH; i++, j++)
        {
            X_Memery_Store_Plus[j] = flash_union_buffer[i].float_type;
        }
    }

    // ����X�����ݶ�ȡ������ 9
    if (flash_check(FLASH_SECTION_INDEX, X_memery_page_INDEX_9))
    {
        flash_read_page_to_buffer(FLASH_SECTION_INDEX, X_memery_page_INDEX_9, FLASH_PAGE_LENGTH);
        for (size_t i = 0, j = 4 * FLASH_PAGE_LENGTH; i < FLASH_PAGE_LENGTH; i++, j++)
        {
            X_Memery_Store_Plus[j] = flash_union_buffer[i].float_type;
        }
    }

    // ����X�����ݶ�ȡ������ 13
    if (flash_check(FLASH_SECTION_INDEX, X_memery_page_INDEX_13))
    {
        flash_read_page_to_buffer(FLASH_SECTION_INDEX, X_memery_page_INDEX_13, FLASH_PAGE_LENGTH);
        for (size_t i = 0, j = 5 * FLASH_PAGE_LENGTH; i < FLASH_PAGE_LENGTH; i++, j++)
        {
            X_Memery_Store_Plus[j] = flash_union_buffer[i].float_type;
        }
        // �����һҳ��ȡ·���յ�ֵ
        road_destination = flash_union_buffer[FLASH_PAGE_LENGTH - 1].uint16_type;
    }

    /*ȡy������,����������ȡ*/
    if (flash_check(FLASH_SECTION_INDEX, Y_memery_page_INDEX_2)) // ���flash�����Ƿ��������
    {
        flash_read_page_to_buffer(FLASH_SECTION_INDEX, Y_memery_page_INDEX_2, FLASH_PAGE_LENGTH);
        for (size_t i = 0; i < FLASH_PAGE_LENGTH; i++)
        {
            Y_Memery_Store_Plus[i] = flash_union_buffer[i].float_type; //
        }
    }

    /*ȡy������*/
    if (flash_check(FLASH_SECTION_INDEX, Y_memery_page_INDEX_4)) // ���flash�����Ƿ��������
    {
        flash_read_page_to_buffer(FLASH_SECTION_INDEX, Y_memery_page_INDEX_4, FLASH_PAGE_LENGTH);
        for (size_t i = 0, j = FLASH_PAGE_LENGTH; i < FLASH_PAGE_LENGTH; i++, j++)
        {
            Y_Memery_Store_Plus[j] = flash_union_buffer[i].float_type; //
        }
    }

    /*ȡy������*/
    if (flash_check(FLASH_SECTION_INDEX, Y_memery_page_INDEX_6)) // ���flash�����Ƿ��������
    {
        flash_read_page_to_buffer(FLASH_SECTION_INDEX, Y_memery_page_INDEX_6, FLASH_PAGE_LENGTH);
        for (size_t i = 0, j = 2 * FLASH_PAGE_LENGTH; i < FLASH_PAGE_LENGTH; i++, j++)
        {
            Y_Memery_Store_Plus[j] = flash_union_buffer[i].float_type; //
        }
    }

    // ����Y�����ݶ�ȡ������ 8
    if (flash_check(FLASH_SECTION_INDEX, Y_memery_page_INDEX_8))
    {
        flash_read_page_to_buffer(FLASH_SECTION_INDEX, Y_memery_page_INDEX_8, FLASH_PAGE_LENGTH);
        for (size_t i = 0, j = 3 * FLASH_PAGE_LENGTH; i < FLASH_PAGE_LENGTH; i++, j++)
        {
            Y_Memery_Store_Plus[j] = flash_union_buffer[i].float_type;
        }
    }

    // ����Y�����ݶ�ȡ������ 10
    if (flash_check(FLASH_SECTION_INDEX, Y_memery_page_INDEX_10))
    {
        flash_read_page_to_buffer(FLASH_SECTION_INDEX, Y_memery_page_INDEX_10, FLASH_PAGE_LENGTH);
        for (size_t i = 0, j = 4 * FLASH_PAGE_LENGTH; i < FLASH_PAGE_LENGTH; i++, j++)
        {
            Y_Memery_Store_Plus[j] = flash_union_buffer[i].float_type;
        }
    }

    // ����Y�����ݶ�ȡ������ 12
    if (flash_check(FLASH_SECTION_INDEX, Y_memery_page_INDEX_12))
    {
        flash_read_page_to_buffer(FLASH_SECTION_INDEX, Y_memery_page_INDEX_12, FLASH_PAGE_LENGTH);
        for (size_t i = 0, j = 5 * FLASH_PAGE_LENGTH; i < FLASH_PAGE_LENGTH; i++, j++)
        {
            Y_Memery_Store_Plus[j] = flash_union_buffer[i].float_type;
        }
    }

    flash_flag_Plus = 4;
}