#include "zf_common_headfile.h"
#include "core_shared.h"

#pragma location = 0x28031064
uint8 calibrated_image[49 * 80];
#pragma location = 0x28031FB4
uint8 calibrated_binary_image[49 * 80];

#pragma location = 0x28032F04
VisionConfigShared vision_config_shared = {
    // 这里可以添加相机配置参数
    // 例如：分辨率、帧率等
    
};

#pragma location = 0x28033000
VisionOutputsShared vision_outputs_shared = {

};

// #pragma location = 0x28033100
// VisionDebugShared vision_debug_shared = {
//     .raw_image = mt9v03x_image[0], // 原始图像数据
//     .calibrated_image = calibrated_image, // 校正后的图像数据
//     .calibrated_binary_image = calibrated_binary_image, // 二值化后的图像数据
// };