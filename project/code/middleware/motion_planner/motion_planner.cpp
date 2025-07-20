#include "motion_planner.h"
#include "multicore/core_shared.h"
#include <algorithm>

void MotionPlanner::init()
{
}

void MotionPlanner::reset()
{
}

void MotionPlanner::update()
{
    planner_local_path = *input_path_;

    update_element();
    update_angle();
    update_speed();
}

void MotionPlanner::update_angle()
{
    if (miss_line == true)
    {
        return;
    }

    // 1. 计算主前瞻曲率和3/4前瞻曲率和实际主前瞻距离
    if (current_element_type == ElementType::LEFT_ROUNDABOUT
        && current_element_point.y() > 0.5 * calibrated_height
        && current_element_point.y() < 0.9 * calibrated_height)
    {
        angle = -0.5f;
    }
    else if (current_element_type == ElementType::RIGHT_ROUNDABOUT
        && current_element_point.y() > 0.5 * calibrated_height
        && current_element_point.y() < 0.9 * calibrated_height)
    {
        angle = 0.3f;
    }
    else
    {
        if (roundabout_remain_time > 0)
        {
            switch (roundabout_direction_)
            {
                case false:
                    angle = -0.5f;
                    break;
                case true:
                    angle = 0.3f;
                    break;
                default:
                    break;
            }
        }
        else
        {
            Point2f target_point;
            float actual_lookahead;
            std::tie(target_point, angle, angle2, actual_lookahead) = pure_pursuit(planner_local_path, lookahead_distance);

            vision_debug_shared.pure_pursuit_target = target_point;
        }
    }

    *output_target_angle_ = angle_to_servo(angle);
    *output_target_angle_vel_ = angle_to_servo(angle_vel);
}

void MotionPlanner::update_speed()
{
    // 速度更新逻辑可以在这里实现
    // 目前只是简单地将目标速度设置为0.0f

    const float k = 45;
    const float max_speed = 70;
    speed = min(max_speed, k / (abs(angle) + 0.01f));
    speed_accel = 0.0f;
    *output_target_speed_ = speed;
    *output_target_speed_accel_ = speed_accel;
}

void MotionPlanner::connect_inputs(const TrackPath* path, const float* current_x, const float* current_y, const float* current_yaw)
{
    // 连接输入变量
    input_path_ = path;

    input_current_x_ = current_x;
    input_current_y_ = current_y;
    input_current_yaw_ = current_yaw;
}

void MotionPlanner::connect_outputs(float* target_speed, float* target_speed_accel,
    float* target_curvature, float* target_curvature_rate)
{
    // 连接输出变量
    output_target_speed_ = target_speed;
    output_target_speed_accel_ = target_speed_accel;
    output_target_angle_ = target_curvature;
    output_target_angle_vel_ = target_curvature_rate;
}


// 提取路径上距离起点指定路程的点
static Point2f interpolate_path_point(const TrackPath& path, float dist, float& out_actual_dist)
{
    float accum = 0.0f;
    Point2f target = path[path.size() - 1].pos;
    out_actual_dist = 0.0f;
    for (size_t i = 0; i + 1 < path.size(); ++i)
    {
        float seg = path[i].next_length;
        if (accum + seg >= dist)
        {
            float remain = dist - accum;
            float t = (seg > 1e-6f) ? (remain / seg) : 0.0f;
            const Point2f& p0 = path[i].pos;
            const Point2f& p1 = path[i + 1].pos;
            target = p0 + t * (p1 - p0);
            out_actual_dist = accum + t * seg;
            return target;
        }
        accum += seg;
    }
    out_actual_dist = accum;
    return target;
}

// 返回tuple<主前瞻target点, 主前瞻曲率, 3/4前瞻曲率, 实际主前瞻距离>
std::tuple<Point2f, float, float, float> MotionPlanner::pure_pursuit(const TrackPath& path, float lookahead)
{
    if (path.size() == 0 || path.length() < 1e-6f)
        return std::tuple<Point2f, float, float, float>(Point2f(), 0.0f, 0.0f, 0.0f);
    Point2f pos(calibrated_width / 2, calibrated_height);

    // 主前瞻点
    float actual_lookahead = 0.0f;
    Point2f target = interpolate_path_point(path, lookahead, actual_lookahead);
    if (actual_lookahead < 0.5f * lookahead)
        actual_lookahead = 0.5f * lookahead;
    float x_r = target.x() - pos.x();
    float curvature = actual_lookahead > 1e-6f ? x_r / (actual_lookahead) : 0;

    // 3/4前瞻点，使用主前瞻点实际距离的3/4
    float actual_lookahead2 = 0.0f;
    Point2f target2 = interpolate_path_point(path, 0.75f * actual_lookahead, actual_lookahead2);
    float x_r2 = target2.x() - pos.x();
    float curvature2 = x_r2 / (actual_lookahead2 > 1e-6f ? actual_lookahead2 : 1.0f);

    return std::tuple<Point2f, float, float, float>(target, curvature, curvature2, actual_lookahead);
}

void MotionPlanner::path_local_to_global(TrackPath& global_path, const TrackPath& local_path)
{
    global_path = local_path;
    // 将路径从局部坐标系转换到全局坐标系
    if (input_current_x_ && input_current_y_ && input_current_yaw_)
    {
        float global_x = *input_current_x_;
        float global_y = *input_current_y_;
        float global_yaw = *input_current_yaw_;

        float cos_yaw = cos(global_yaw);
        float sin_yaw = sin(global_yaw);

        for (size_t i = 0; i < global_path.size(); ++i)
        {
            Point2f& pos = global_path[i].pos;
            float x_new = global_x + pos.x() * cos_yaw - pos.y() * sin_yaw;
            float y_new = global_y + pos.x() * sin_yaw + pos.y() * cos_yaw;
            pos = Point2f(x_new, y_new);
        }
    }
    else
    {
        global_path.clear();
    }
}

void MotionPlanner::path_global_to_local(TrackPath& local_path, const TrackPath& global_path)
{
    local_path = global_path;
    // 将路径从全局坐标系转换到局部坐标系
    if (input_current_x_ && input_current_y_ && input_current_yaw_)
    {
        float global_x = *input_current_x_;
        float global_y = *input_current_y_;
        float global_yaw = *input_current_yaw_;

        float cos_yaw = cos(global_yaw);
        float sin_yaw = sin(global_yaw);

        for (size_t i = 0; i < local_path.size(); ++i)
        {
            Point2f& pos = local_path[i].pos;
            float x_new = (pos.x() - global_x) * cos_yaw + (pos.y() - global_y) * sin_yaw;
            float y_new = -(pos.x() - global_x) * sin_yaw + (pos.y() - global_y) * cos_yaw;
            pos = Point2f(x_new, y_new);
        }
    }
    else
    {
        local_path.clear();
    }
}
