#pragma once
#include "zf_common_headfile.h"
#include "middleware/vision/point.h"

#include "middleware/vision/road_element.h"

constexpr uint16_t calibrated_width = 80;
constexpr uint16_t calibrated_height = 49;

extern uint8_t mt9v03x_image[MT9V03X_H][MT9V03X_W];
extern uint8_t calibrated_image[calibrated_height * calibrated_width];
extern uint8_t calibrated_binary_image[calibrated_height * calibrated_width];

struct VisionConfigShared
{
    // 这里可以添加相机配置参数
    // 例如：分辨率、帧率等
};

struct VisionOutputsShared
{
    float bias;                        // 循迹偏差
    float bias_accel;                // 循迹偏差加速度
    float target_speed;               // 目标速度
    float target_speed_accel;        // 目标速度加速度
};

// 除图像外只读调试数据
struct VisionDebugShared
{
    int16 left_bounds[calibrated_height];
    int16 right_bounds[calibrated_height];
    int16 line_points[calibrated_height]; // 道路中心线点
    RoadElementType element_type;      // 检测到的道路元素类型
    int element_position;              // 元素位置
    float element_confidence;          // 元素检测置信度
};

extern VisionConfigShared vision_config_shared;
extern VisionOutputsShared vision_outputs_shared;
extern VisionDebugShared vision_debug_shared;