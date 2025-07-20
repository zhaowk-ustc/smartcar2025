#include "motion_planner.h"
#include "multicore/core_shared.h"


void MotionPlanner::update_element()
{

    if (planner_local_path.size() < 2 || planner_local_path.length() < 8.0f)
    {
        miss_line = true;
    }
    else
    {
        miss_line = false;

        // 检测环岛十字
        // 遍历planner_path找到第一个元素
        int first_element_idx = -1;
        ElementType first_element_type = ElementType::NORMAL;
        for (size_t i = 0; i < planner_local_path.size(); ++i)
        {
            if (planner_local_path[i].element == ElementType::CROSS ||
                planner_local_path[i].element == ElementType::LEFT_ROUNDABOUT ||
                planner_local_path[i].element == ElementType::RIGHT_ROUNDABOUT)
            {
                first_element_idx = i;
                first_element_type = planner_local_path[i].element;
                break;
            }
        }

        static ElementType last_detected_type = ElementType::NORMAL;
        static int detected_count = 0;
        const int DETECT_THRESHOLD = 5; // 连续检测阈值，可调整

        if (first_element_type == last_detected_type)
        {
            detected_count++;
        }
        else
        {
            detected_count = 1;
            last_detected_type = first_element_type;
        }
        if (detected_count >= DETECT_THRESHOLD)
        {
            current_element_point = planner_local_path[first_element_idx].pos;
            current_element_type = first_element_type;
        }
    }

    switch (current_element_type)
    {
        case ElementType::LEFT_ROUNDABOUT:
            if (current_element_point.y() > 0.5 * calibrated_height && current_element_point.y() < 0.9 * calibrated_height)
            {
                roundabout_remain_time = 50;
                roundabout_direction_ = false; // 左侧环岛
            }
            break;
        case ElementType::RIGHT_ROUNDABOUT:
            if (current_element_point.y() > 0.5 * calibrated_height && current_element_point.y() < 0.9 * calibrated_height)
            {
                roundabout_remain_time = 50;
                roundabout_direction_ = true; // 右侧环岛
            }
            break;
        default:
            break;
    }
    if (roundabout_remain_time > 0)
    {
        roundabout_remain_time -= 1;
    }
}