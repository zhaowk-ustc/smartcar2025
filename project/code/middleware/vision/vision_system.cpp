#include "vision_system.h"
#include "zf_common_headfile.h"
#include "multicore/core_shared.h"
#include "calibration.h"
#include "binarization.h"

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

}
