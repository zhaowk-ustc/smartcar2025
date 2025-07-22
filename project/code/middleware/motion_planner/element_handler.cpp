#include "motion_planner.h"
#include "multicore/core_shared.h"

void MotionPlanner::fix_path()
{
    // 修正路径，确保转角不大于120度，如果大于120度改成直线
    if (planner_local_path.size() < 3) return;
    const float cos_threshold = -0.5f; // cos(120°)
    const float extend_length = 60.0f; // 射线长度，可根据需求调整
    for (size_t i = 1; i + 1 < planner_local_path.size(); ++i)
    {
        auto& prev = planner_local_path[i - 1].pos;
        auto& curr = planner_local_path[i].pos;
        auto& next = planner_local_path[i + 1].pos;
        auto v1 = curr - prev;
        auto v2 = next - curr;
        float len1 = std::abs(v1);
        float len2 = std::abs(v2);
        if (len1 < 1e-3f || len2 < 1e-3f) continue;
        float cos_theta = (v1.real() * v2.real() + v1.imag() * v2.imag()) / (len1 * len2);


        if (cos_theta < cos_threshold)
        {
            // 计算射线终点
            auto dir = v1 / len1;
            next = curr + dir * extend_length;
            // 截断路径
            planner_local_path.nodes[i + 1].element = ElementType::NORMAL;
            planner_local_path.nodes[i + 1].next_dir = dir;
            planner_local_path.nodes[i + 1].next_length = extend_length;
            planner_local_path.clear();
            for (size_t j = 0; j <= i + 1; ++j)
            {
                planner_local_path.add_node(planner_local_path.nodes[j]);
            }
            break;
        }


        // 新增：环岛保护期内检测到环岛，替换为30度射线
        if (roundabout_protect_time > 0 && roundabout_remain_time == 0 &&
            (planner_local_path.nodes[i + 1].element == ElementType::LEFT_ROUNDABOUT || planner_local_path.nodes[i + 1].element == ElementType::RIGHT_ROUNDABOUT))
        {
            // 30度常量
            constexpr float cos30 = 0.8660254f;
            constexpr float sin30 = 0.5f;
            float sign = (planner_local_path.nodes[i + 1].element == ElementType::LEFT_ROUNDABOUT) ? 1.0f : -1.0f;
            auto dir = v1 / len1;
            // 旋转dir向量
            float cos_a = cos30;
            float sin_a = sign * sin30;
            auto dir_rot = decltype(dir)(dir.real() * cos_a - dir.imag() * sin_a, dir.real() * sin_a + dir.imag() * cos_a);
            next = curr + dir_rot * extend_length;
            planner_local_path.nodes[i + 1].element = ElementType::NORMAL;
            planner_local_path.nodes[i + 1].next_dir = dir_rot;
            planner_local_path.nodes[i + 1].next_length = extend_length;
            planner_local_path.clear();
            for (size_t j = 0; j <= i + 1; ++j)
            {
                planner_local_path.add_node(planner_local_path.nodes[j]);
            }
            break;
        }
    }
}

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


    if (roundabout_remain_time > 0)
    {
        roundabout_remain_time -= 1;
    }
    else if (roundabout_protect_time > 0)
    {
        roundabout_protect_time -= 1;
        return;
    }
    switch (current_element_type)
    {
        case ElementType::LEFT_ROUNDABOUT:
            if (current_element_point.y() > 0.5 * calibrated_height && current_element_point.y() < 0.9 * calibrated_height)
            {
                roundabout_remain_time = max_roundabout_remain_time;
                roundabout_protect_time = max_roundabout_remain_time * 10; // 环岛保护时间，单位为10ms
                roundabout_direction_ = false; // 左侧环岛
            }
            break;
        case ElementType::RIGHT_ROUNDABOUT:
            if (current_element_point.y() > 0.5 * calibrated_height && current_element_point.y() < 0.9 * calibrated_height)
            {
                roundabout_remain_time = max_roundabout_remain_time;
                roundabout_protect_time = max_roundabout_remain_time * 10;
                roundabout_direction_ = true; // 右侧环岛
            }
            break;
        default:
            break;
    }
}

bool MotionPlanner::detect_breakline()
{
    if (planner_local_path.size() < 2) return false;
    Point2f end = planner_local_path.end();
    if (
        end.y() > 0.2 * calibrated_height && end.y() < 0.7 * calibrated_height
        && end.x() > 0.2 * calibrated_width && end.x() < 0.8 * calibrated_width
        )
    {
        is_breakline = true;
        return true;
    }
    // is_breakline = false;
    return false;
}
