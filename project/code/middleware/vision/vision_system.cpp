#include "vision_system.h"
#include "zf_common_headfile.h"
#include "multicore/core_shared.h"
#include "calibration.h"
#include "binarization.h"
#include "line_tracking.h"
#include "road_element.h"

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

    image_calibration(mt9v03x_image[0], calibrated_image);

    image_binarization(calibrated_image, calibrated_binary_image, 49 * 80);

    auto leftbounds = find_left_bounds(calibrated_binary_image, 80, 49, {});
    auto rightbounds = find_right_bounds(calibrated_binary_image, 80, 49, {});
    // 道路整理处理（综合左右边界，生成循迹线）
    vector<int16> road_line = process_road_line(leftbounds, rightbounds);

    for (size_t i = 0; i < leftbounds.size(); ++i)
    {
        vision_debug_shared.left_bounds[i] = leftbounds[i];
        vision_debug_shared.right_bounds[i] = rightbounds[i];
        vision_debug_shared.line_points[i] = road_line[i];
    }

    // 统一的道路元素检测
    RoadElementResult element_result = detect_road_element(leftbounds, rightbounds);
    

    
    // 更新共享内存中的元素检测结果
    vision_debug_shared.element_type = element_result.type;
    vision_debug_shared.element_position = element_result.position;
    vision_debug_shared.element_confidence = element_result.confidence;

    // 计算循迹偏差
    vision_outputs_shared.bias = get_bias(road_line, element_result, 80);

    SCB_CleanDCache_by_Addr((void*)&vision_debug_shared, sizeof(vision_debug_shared));
    SCB_CleanDCache_by_Addr((void*)&vision_outputs_shared, sizeof(vision_outputs_shared));
}