
#include "binarization.h"

uint8 otsu_threshold_core(int32 hist[256], int total_pixels)
{
    float sumTotal = 0.0f;
    for (int t = 0; t <= 255; t++)
    {
        sumTotal += t * hist[t];
    }

    float weightB = 0.0f, sumB = 0.0f;
    float maxVar = 0.0f;
    uint8 threshold = 0;

    for (int t = 0; t <= 255; t++)
    {
        weightB += hist[t];
        sumB += t * hist[t];

        float weightF = total_pixels - weightB;
        if (weightB == 0 || weightF == 0) continue;

        float meanB = sumB / weightB;
        float meanF = (sumTotal - sumB) / weightF;
        float varBetween = weightB * weightF * (meanB - meanF) * (meanB - meanF);

        if (varBetween > maxVar)
        {
            maxVar = varBetween;
            threshold = t;
        }
    }
    return threshold;
}

uint8 calculate_otsu_threshold(const uint8* image_data, uint32 size)
{
    int32 hist[256] = { 0 };
    for (uint32 i = 0; i < size; i++)
    {
        hist[image_data[i]]++;
    }
    return otsu_threshold_core(hist, size);
}

uint8 calculate_otsu_threshold_with_mask(const uint8* image_data, const uint8* mask, uint32 size)
{
    int32 hist[256] = { 0 };
    uint32 valid_pixels = 0;
    for (uint32 i = 0; i < size; i++)
    {
        if (mask[i] != 0)  // 只统计mask区域内的像素
        {
            hist[image_data[i]]++;
            valid_pixels++;
        }
    }
    if (valid_pixels == 0) return 128;  // 如果没有有效像素，返回默认阈值
    return otsu_threshold_core(hist, valid_pixels);
}

void apply_binarization(const uint8* source_image_data, uint8* dest_image_data, uint32 size, uint8 threshold)
{
    for (uint32 i = 0; i < size; i++)
    {
        dest_image_data[i] = source_image_data[i] > threshold ? 255 : 0;
    }
}

void apply_binarization_with_mask(const uint8* source_image_data, uint8* dest_image_data, const uint8* mask, uint32 size, uint8 threshold)
{
    for (uint32 i = 0; i < size; i++)
    {
        if (mask[i] != 0)  // 只在mask区域内进行二值化
        {
            dest_image_data[i] = source_image_data[i] > threshold ? 255 : 0;
        }
        else
        {
            dest_image_data[i] = 0;  // mask外区域设为黑色
        }
    }
}

// Advanced Otsu 算法：先大津法，然后修改直方图，再进行大津法
uint8 advanced_otsu_threshold(const uint8* source_image_data, uint32 size)
{
    // 第一步：计算初始直方图和大津阈值
    int32 hist[256] = { 0 };
    for (uint32 i = 0; i < size; i++)
    {
        hist[source_image_data[i]]++;
    }
    uint8 first_threshold = otsu_threshold_core(hist, size);
    
    // 第二步：修改直方图，将小于阈值的像素统计合并到阈值位置
    int32 pixels_below_threshold = 0;
    for (int i = 0; i < first_threshold; i++)
    {
        pixels_below_threshold += hist[i];
        hist[i] = 0;  // 清零小于阈值的统计
    }
    hist[first_threshold] += pixels_below_threshold;  // 合并到阈值位置
    
    // 第三步：对修改后的直方图再次计算大津阈值
    uint8 final_threshold = otsu_threshold_core(hist, size);
    
    return final_threshold;
}

// 带mask的Advanced Otsu 算法
uint8 advanced_otsu_threshold_with_mask(const uint8* source_image_data, const uint8* mask, uint32 size)
{
    // 第一步：计算初始直方图和大津阈值（只统计mask区域）
    int32 hist[256] = { 0 };
    uint32 valid_pixels = 0;
    for (uint32 i = 0; i < size; i++)
    {
        if (mask[i] != 0)
        {
            hist[source_image_data[i]]++;
            valid_pixels++;
        }
    }
    if (valid_pixels == 0) return 128;  // 如果没有有效像素，返回默认阈值
    
    uint8 first_threshold = otsu_threshold_core(hist, valid_pixels);
    
    // 第二步：修改直方图，将小于阈值的像素统计合并到阈值位置
    int32 pixels_below_threshold = 0;
    for (int i = 0; i < first_threshold; i++)
    {
        pixels_below_threshold += hist[i];
        hist[i] = 0;  // 清零小于阈值的统计
    }
    hist[first_threshold] += pixels_below_threshold;  // 合并到阈值位置
    
    // 第三步：对修改后的直方图再次计算大津阈值
    uint8 final_threshold = otsu_threshold_core(hist, valid_pixels);
    
    return final_threshold;
}



uint8 image_binarization(const uint8* source_image_data, uint8* dest_image_data, uint32 size)
{
    // uint8 threshold = calculate_otsu_threshold(source_image_data, size);
    uint8 threshold = advanced_otsu_threshold(source_image_data, size);
    apply_binarization(source_image_data, dest_image_data, size, threshold);
    return threshold;
}

uint8 image_binarization_with_mask(const uint8* source_image_data, uint8* dest_image_data, const uint8* mask, uint32 size)
{
    // 只在mask区域计算阈值
    uint8 threshold = advanced_otsu_threshold_with_mask(source_image_data, mask, size);
    apply_binarization_with_mask(source_image_data, dest_image_data, mask, size, threshold);
    return threshold;
}



