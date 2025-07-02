
#ifndef BINARIZATION_H
#define BINARIZATION_H

#include "zf_common_headfile.h"

// 计算OTSU阈值函数
// 参数：image_data - 图像数据指针
//      size - 图像大小
// 返回：计算得到的阈值
uint8 calculate_otsu_threshold(const uint8* image_data, uint32 size);

// 执行二值化处理函数
// 参数：source_image_data - 原始图像数据指针
//      dest_image_data - 二值化后图像数据指针
//      size - 图像大小
//      threshold - 二值化阈值
void apply_binarization(const uint8* source_image_data, uint8* dest_image_data, uint32 size, uint8 threshold);

// 完整的二值化处理函数（包含阈值计算和二值化）
// 参数：source_image_data - 原始图像数据指针
//      dest_image_data - 二值化后图像数据指针
//      size - 图像大小
// 返回：使用的阈值
uint8 image_binarization(const uint8* source_image_data, uint8* dest_image_data, uint32 size);

#endif // BINARIZATION_H