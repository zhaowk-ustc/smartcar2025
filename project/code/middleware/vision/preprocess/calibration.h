#ifndef CALIBRATION_H
#define CALIBRATION_H

#include "zf_common_headfile.h"

// 图像校正函数
// 参数：source_image_data - 原始图像数据指针
//      dest_image_data - 校正后图像数据指针
void image_calibration(const uint8* source_image_data, uint8* dest_image_data);

#endif