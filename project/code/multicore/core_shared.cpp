#include "zf_common_headfile.h"
#include "core_shared.h"


#pragma location = 0x28031064
uint8_t calibrated_image[calibrated_height * calibrated_width];
#pragma location = 0x28031FB4
uint8_t calibrated_binary_image[calibrated_height * calibrated_width];

#pragma location = 0x28032F04
VisionConfigShared vision_config_shared = {
    // 这里可以添加相机配置参数
    // 例如：分辨率、帧率等
};

#pragma location = 0x28033000
VisionOutputsShared vision_outputs_shared = {

};

#pragma location = 0x28033100
VisionDebugShared vision_debug_shared = {
    .left_bounds = {},
    .right_bounds = {}
};