#pragma once
#include "zf_common_headfile.h"
#include "middleware/vision/common/point.h"
#include "middleware/vision/track/line_tracking.h"

constexpr uint16_t calibrated_width = 57;
constexpr uint16_t calibrated_height = 31;
constexpr uint16_t calibrated_size = calibrated_width * calibrated_height;
constexpr uint16_t aligned_calibrated_size = 80 * 49;

extern uint8_t mt9v03x_image[MT9V03X_H][MT9V03X_W];
extern uint8_t calibrated_image[aligned_calibrated_size];
extern uint8_t calibrated_binary_image[aligned_calibrated_size];

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

    LineTrackingGraph graph;          // 循迹图
};

// 除图像外只读调试数据
struct VisionDebugShared
{
    // Point2s left_bounds[calibrated_height];
    // Point2s right_bounds[calibrated_height];
    // Point2s bottom_bounds[calibrated_height];
    // float left_has_line;
    // float right_has_line;
    // float up_has_line;
    float no_element_confidence;
    // float left_turn_confidence;
    // float right_turn_confidence;
    // float cross_road_confidence;
    // float left_roundabout_confidence;
    // float right_roundabout_confidence;
};

extern VisionConfigShared vision_config_shared;
extern VisionOutputsShared vision_outputs_shared;
extern VisionDebugShared vision_debug_shared;