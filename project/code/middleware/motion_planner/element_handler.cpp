#include "motion_planner.h"
#include "multicore/core_shared.h"


void MotionPlanner::update_element()
{

    if (planner_path.size() < 2 || planner_path.length() < 8.0f)
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
        for (size_t i = 0; i < planner_path.size(); ++i)
        {
            if (planner_path[i].element == ElementType::CROSS ||
                planner_path[i].element == ElementType::LEFT_ROUNDABOUT ||
                planner_path[i].element == ElementType::RIGHT_ROUNDABOUT)
            {
                first_element_idx = i;
                first_element_type = planner_path[i].element;
                break;
            }
            {
                first_element_idx = i;
                first_element_type = planner_path[i].element;
                break;
            }
        }

        // 可选：记录第一个元素的点
        if (first_element_idx != -1)
        {
            current_element_point = planner_path[first_element_idx].pos;
            current_element_type = first_element_type;
        }


        // 检测u型转弯
        if (current_element_type != ElementType::NORMAL)
        {
            auto u_turn_res = detect_u_turn(planner_path);
            auto is_u_turn = std::get<0>(u_turn_res);
            u_turn_direction_ = std::get<1>(u_turn_res);
            current_element_point = std::get<2>(u_turn_res);
        }
    }


}

constexpr float U_TURN_DETECT_THRESHOLD = calibrated_height * 0.25f; // U型转弯的y阈值
std::tuple<bool, bool, Point2f> MotionPlanner::detect_u_turn(const TrackPath& path)
{
    if (path.size() < 2)
        return std::make_tuple(false, false, Point2f());
    Point2f start = path.start();
    Point2f end = path.end();

    // 1. 找到下降最大段的起点
    float max_fall = 0.0f;
    size_t max_fall_idx = 0;
    for (size_t i = 0; i + 1 < path.size(); ++i)
    {
        float fall = path[i].pos.y() - path[i + 1].pos.y();
        if (fall > max_fall)
        {
            max_fall = fall;
            max_fall_idx = i;
        }
    }

    bool is_u_turn = false;
    bool u_turn_direction_ = false; // 回弯的方向，false表示左
    // 2. 若终点y比起点y大且差值超过阈值，则判定U型转弯
    float fall_value = end.y() - start.y();
    if (fall_value > U_TURN_DETECT_THRESHOLD)
    {
        is_u_turn = true;
        u_turn_direction_ = (end.x() > start.x()); // x方向判断U型转弯方向
    }
    else
    {
        is_u_turn = false;
    }
    // 返回下降最大段的起点
    return std::make_tuple(is_u_turn, u_turn_direction_, path[max_fall_idx].pos);
}
