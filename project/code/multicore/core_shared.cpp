#include "zf_common_headfile.h"
#include "core_shared.h"


#pragma location = 0x28032000
uint8_t calibrated_image[aligned_calibrated_size];
#pragma location = 0x28036000
uint8_t calibrated_binary_image[aligned_calibrated_size];

#pragma location = 0x2803a000
uint8_t vision_ttl_map[aligned_calibrated_size];
#pragma location = 0x2803e000
uint8_t vision_depth_map[aligned_calibrated_size];
#pragma location = 0x28042000
uint8_t vision_visited[aligned_calibrated_size];
#pragma location = 0x28046000
int16_t vision_point_to_node_map[aligned_calibrated_size];

#pragma location = 0x2804a000
VisionConfigShared vision_config_shared = {

};

#pragma location = 0x2804c000
VisionOutputsShared vision_outputs_shared = {

};

#pragma location = 0x28050000
VisionDebugShared vision_debug_shared = {


};

#pragma location = 0x28058000
LineTrackingGraph vision_line_tracking_graph;