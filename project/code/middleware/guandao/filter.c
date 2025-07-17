/*
 * filter.c
 *
 *  Created on: 2024年11月8日
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
    // 计算RC时间常数
    float rc = 1.0f / (2.0f * M_PI * cutoff_freq);
    // 计算滤波系数
    filter->alpha = sample_time / (rc + sample_time);
    filter->prev_output = 0.0f;
    filter->initialized = 0;
}

float LPF_Update(LowPassFilter *filter, float input)
{
    // 首次调用时直接初始化输出值
    if (!filter->initialized)
    {
        filter->prev_output = input;
        filter->initialized = 1;
        return input;
    }

    // 一阶低通滤波公式
    float output = filter->alpha * input + (1.0f - filter->alpha) * filter->prev_output;

    // 更新状态
    filter->prev_output = output;

    return output;
}