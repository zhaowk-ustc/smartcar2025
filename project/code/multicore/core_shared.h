#pragma once
#include "zf_common_headfile.h"
#include "middleware/vision/common/point.h"
#include "middleware/vision/track/line_tracking.h"
#include "middleware/vision/element/track_path.h"
#include "middleware/vision/preprocess/calibration.h"

extern uint8_t mt9v03x_image[MT9V03X_H][MT9V03X_W];
extern uint8_t calibrated_image[aligned_calibrated_size];
extern uint8_t calibrated_binary_image[aligned_calibrated_size];

extern LineTrackingGraph vision_line_tracking_graph;
extern uint8_t vision_ttl_map[aligned_calibrated_size];
extern uint8_t vision_depth_map[aligned_calibrated_size];
extern uint8_t vision_visited[aligned_calibrated_size];
extern int16_t vision_point_to_node_map[aligned_calibrated_size];

struct VisionConfigShared
{
    bool fixed_threshold_enable = false;
    int fixed_threshold;
};

struct VisionOutputsShared
{
    TrackPath track_path;

    bool miss_line = false;

    int update_count;
};

// 除图像外只读调试数据
struct VisionDebugShared
{
    Point2f pure_pursuit_target;
    LineTrackingGraph line_tracking_graph;
};

extern VisionConfigShared vision_config_shared;
extern VisionOutputsShared vision_outputs_shared;
extern VisionDebugShared vision_debug_shared;