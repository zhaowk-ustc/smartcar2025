/*
 * filter.c
 *
 *  Created on: 2024��11��8��
 *      Author: 86152
 */

#include "zf_common_headfile.h"
#include "filter.h"

LowPassFilter velocity_filter={0};




void LPF_InitByAlpha(LowPassFilter *filter, float alpha)
{
    filter->alpha = alpha;
    filter->prev_output = 0.0f;
    filter->initialized = 0;
}

void LPF_InitByFrequency(LowPassFilter *filter, float cutoff_freq, float sample_time)
{
    // ����RCʱ�䳣��
    float rc = 1.0f / (2.0f * M_PI * cutoff_freq);
    // �����˲�ϵ��
    filter->alpha = sample_time / (rc + sample_time);
    filter->prev_output = 0.0f;
    filter->initialized = 0;
}

float LPF_Update(LowPassFilter *filter, float input)
{
    // �״ε���ʱֱ�ӳ�ʼ�����ֵ
    if (!filter->initialized)
    {
        filter->prev_output = input;
        filter->initialized = 1;
        return input;
    }

    // һ�׵�ͨ�˲���ʽ
    float output = filter->alpha * input + (1.0f - filter->alpha) * filter->prev_output;

    // ����״̬
    filter->prev_output = output;

    return output;
}