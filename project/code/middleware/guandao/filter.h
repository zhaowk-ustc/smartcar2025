/*
 * filter.h
 *
 *  Created on: 2024��11��8��
 *      Author: 86152
 */

#ifndef CODE_FILTER_H_
#define CODE_FILTER_H_

#include "zf_common_headfile.h"


#define MAX_SENSOR_NUM 9   //ʹ���˲�ʱ�Ĵ���������
#define MAX_DATA_NUM 9     //�����������������������ڳ���
#define WINDOW_DATA_NUM 5  //�˲����ڳ���
//ȥ�����������������Сֵ������������ȥ����������������С
#define REMOVE_MAXMIN_NUM ((MAX_DATA_NUM - WINDOW_DATA_NUM)/2)
// ȥ�����������������Сֵ������������ȥ����������������С
#define M_PI 3.14f







/* �˲����ṹ�壨�û���������ڲ���Ա�� */
typedef struct
{
    float alpha;         // �˲�ϵ����0.0-1.0��
    float prev_output;   // ��һ���˲����
    uint8_t initialized; // ��ʼ����־λ
} LowPassFilter;

    /**
     * @brief ��ʼ����ͨ�˲���
     * @param filter    �˲�������ָ��
     * @param alpha     �˲�ϵ����ֱ��ָ����
     */
    void LPF_InitByAlpha(LowPassFilter *filter, float alpha);

    /**
     * @brief ͨ����ֹƵ�ʺͲ���ʱ���ʼ���˲���
     * @param filter        �˲�������ָ��
     * @param cutoff_freq   ��ֹƵ�ʣ�Hz��
     * @param sample_time   ����ʱ�䣨�룩
     */
    void LPF_InitByFrequency(LowPassFilter *filter, float cutoff_freq, float sample_time);

    /**
     * @brief ִ�е�ͨ�˲�����
     * @param filter    �˲�������ָ��
     * @param input     ��ǰ����ֵ
     * @return float    �˲�������ֵ
     */
    float LPF_Update(LowPassFilter *filter, float input);

 extern  LowPassFilter velocity_filter;

    // ��������ĺ���
    int16 Filter_SlidingWindowAvg(int index, int16 data);

#endif /* CODE_FILTER_H_ */
