
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

void apply_binarization(const uint8* source_image_data, uint8* dest_image_data, uint32 size, uint8 threshold)
{
    for (uint32 i = 0; i < size; i++)
    {
        dest_image_data[i] = source_image_data[i] > threshold ? 255 : 0;
    }
}

uint8 image_binarization(const uint8* source_image_data, uint8* dest_image_data, uint32 size)
{
    uint8 threshold = calculate_otsu_threshold(source_image_data, size);
    apply_binarization(source_image_data, dest_image_data, size, threshold);
    return threshold;
}



