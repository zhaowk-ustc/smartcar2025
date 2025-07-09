#include "zf_common_headfile.h"
#include "core_shared.h"


#pragma location = 0x28032000
uint8_t calibrated_image[aligned_calibrated_size];
#pragma location = 0x28036000
uint8_t calibrated_binary_image[aligned_calibrated_size];

#pragma location = 0x28040000
uint8_t vision_ttl_map[aligned_calibrated_size];
#pragma location = 0x28044000
uint8_t vision_depth_map[aligned_calibrated_size];
#pragma location = 0x28048000
uint8_t vision_visited[aligned_calibrated_size];
#pragma location = 0x2804c000
int16_t vision_point_to_node_map[aligned_calibrated_size];

#pragma location = 0x28050000
VisionConfigShared vision_config_shared = {
    // 这里可以添加相机配置参数
    // 例如：分辨率、帧率等
};

#pragma location = 0x28054000
VisionOutputsShared vision_outputs_shared = {

};

#pragma location = 0x28058000
VisionDebugShared vision_debug_shared = {


};

#pragma location = 0x2805c000
LineTrackingGraph vision_line_tracking_graph;