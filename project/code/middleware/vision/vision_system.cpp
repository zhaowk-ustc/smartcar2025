#include "vision_system.h"
#include "zf_common_headfile.h"
#include "multicore/core_shared.h"
#include "preprocess/calibration.h"
#include "preprocess/binarization.h"
#include "preprocess/calibration.h"
#include "track/line_tracking.h"
#include "element/track_path.h"

VisionSystem::VisionSystem(const Config& config)
{
    // 初始化相机配置
    // 例如：分辨率、帧率等
}

void VisionSystem::init()
{
    mt9v03x_init(); // 初始化相机
}

void VisionSystem::reset()
{

}

void VisionSystem::update()
{
    if (!mt9v03x_finish_flag)
    {
        return;
    }
    mt9v03x_finish_flag = 0;

    image_calibration(calibrated_image, mt9v03x_image[0]);

    image_binarization_with_mask(calibrated_image, calibrated_binary_image, calibration_mask, calibrated_size);

    static LineTrackingGraph vision_static_graph;
    create_line_tracking_graph(
        vision_static_graph,
        calibrated_binary_image, calibrated_width, calibrated_height,
        last_start_point,
        5, //search_seed_radius
        8, //min_region_size
        4, //dist_threshold
        6, //rdp_threshold
        20, //branch_merge_threshold
        10, //min_branch_length
        vision_ttl_map,
        vision_depth_map,
        vision_visited,
        vision_point_to_node_map);

    if (!vision_static_graph.valid)
    {
        last_start_point = Point(calibrated_width / 2, calibrated_height - 1);
        vision_static_graph.clear();
    }
    else
    {
        last_start_point = vision_static_graph.getNode(vision_static_graph.root()).data();
    }

    static TrackPath vision_static_track_path;
    vision_static_track_path.clear();

    extract_path(vision_static_graph, vision_static_track_path);

    memcpy(&vision_line_tracking_graph, &vision_static_graph, sizeof(vision_static_graph));
    memcpy(&vision_outputs_shared.track_path, &vision_static_track_path, sizeof(vision_static_track_path));
    SCB_CleanDCache_by_Addr((void*)&vision_line_tracking_graph, sizeof(vision_line_tracking_graph));
    SCB_CleanDCache_by_Addr((void*)&vision_debug_shared, sizeof(vision_debug_shared));
    SCB_CleanDCache_by_Addr((void*)&vision_outputs_shared, sizeof(vision_outputs_shared));
}