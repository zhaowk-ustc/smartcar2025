#include "zf_common_headfile.h"

extern uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];
extern uint8 calibrated_image[49 * 80];
extern uint8 calibrated_binary_image[49 * 80];

struct VisionConfigShared
{
    // 这里可以添加相机配置参数
    // 例如：分辨率、帧率等
};

struct VisionOutputsShared
{
    uint8* calibrated_image_data = calibrated_image;          // 校正后的图像数据
    uint8* calibrated_binary_image_data = calibrated_binary_image; // 二值化后的图像数据
};

// 除图像外只读调试数据
struct VisionDebugShared
{
    uint8* raw_image = mt9v03x_image[0]; // 原始图像数据
    uint8* calibrated_image = calibrated_image; // 校正后的图像数据
    uint8* calibrated_binary_image = calibrated_binary_image; // 二值化后的图像数据
};

extern VisionConfigShared vision_config_shared;
extern VisionOutputsShared vision_outputs_shared;
extern VisionDebugShared vision_debug_shared;