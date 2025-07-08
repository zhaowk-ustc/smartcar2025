#include "vision_system.h"
#include "zf_common_headfile.h"
#include "multicore/core_shared.h"
#include "preprocess/calibration.h"
#include "preprocess/binarization.h"
#include "preprocess/perspective.h"
#include "track/line_tracking.h"

#include "track/line_tracking.h"

static uint8 tmp_image[3920];

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

    image_calibration(mt9v03x_image[0], tmp_image);
    perspective_mapping(tmp_image, calibrated_image);

    image_binarization_with_mask(calibrated_image, calibrated_binary_image, perspective_mask, calibrated_size);

    static LineTrackingGraph graph;

    create_line_tracking_graph(graph, calibrated_binary_image, calibrated_width, calibrated_height);

    // 计算循迹偏差
    // vision_outputs_shared.bias = get_bias(road_line, element_result, 80);
    vision_outputs_shared.graph = graph;

    SCB_CleanDCache_by_Addr((void*)&vision_debug_shared, sizeof(vision_debug_shared));
    SCB_CleanDCache_by_Addr((void*)&vision_outputs_shared, sizeof(vision_outputs_shared));
}