#include "motion_planner.h"
#include "multicore/core_shared.h"
#include <algorithm>

MotionPlanner::MotionPlanner()
{
    setup_debug_vars();
}

void MotionPlanner::init()
{
}

void MotionPlanner::reset()
{
}

void MotionPlanner::update()
{
    planner_local_path = *input_path_;

    fix_path();
    update_element();
    update_angle();
    update_speed();
}

void MotionPlanner::update_angle()
{

    if (miss_line == true)
    {
        *output_target_angle_vel_ = 0;
        return;
    }
    angle_vel = 0;
    angle = 0;
    if (roundabout_remain_time > 0 && roundabout_remain_time < max_roundabout_remain_time)
    {
        angle_vel = 0;
        switch (roundabout_direction_)
        {
            case false:
                angle = left_round_dir/10.0f;
                break;
            case true:
                angle = right_round_dir/10.0f;
                break;
            default:
                break;
        }
    }
    else
    {
        Point2f target_point;
        std::tie(target_point, angle, actual_lookahead) = pure_pursuit(planner_local_path, base_lookahead);

        Point2f target_point2;
        std::tie(target_point2, angle2, actual_lookahead2) = pure_pursuit(planner_local_path, base_lookahead * 1.5f);

        angle_vel = (angle2 - angle);
        vision_debug_shared.pure_pursuit_target = target_point;
    }
    *output_target_angle_ = angle_to_servo(angle);
    *output_target_angle_vel_ = angle_to_servo(angle_vel);

}

void MotionPlanner::update_speed()
{
    // 速度更新逻辑
    // 基于曲率和角速度估算加速度
    const float k = max_speed * 0.8f;
    speed = min(static_cast<float>(max_speed), k / (min(abs(angle), 2.0f) + 0.01f));

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

// 返回tuple<主前瞻target点, 主前瞻曲率, 二倍前瞻曲率, 实际主前瞻距离, 实际二倍前瞻距离>
std::tuple<Point2f, float, float> MotionPlanner::pure_pursuit(const TrackPath& path, float lookahead)
{
    if (path.size() == 0 || path.length() < 1e-6f)
        return std::tuple<Point2f, float, float>(Point2f(), 0.0f, 0.0f);
    Point2f pos(calibrated_width / 2, calibrated_height);

    // 主前瞻点
    float actual_lookahead = 0.0f;
    Point2f target = interpolate_path_point(path, lookahead, actual_lookahead);
    if (actual_lookahead < 0.5f * lookahead)
        actual_lookahead = 0.5f * lookahead;
    float x_r = target.x() - pos.x();
    float y_r = pos.y() - target.y();
    float curvature = actual_lookahead > 1e-6f ? x_r / (actual_lookahead) : 0;
    // y_r=lookahead时curvature*=1，y_r=0时curvature*=1.2，线性插值
    float k = 1.2f - 0.2f * (y_r / lookahead);
    if (k < 1.0f) k = 1.0f;
    if (k > 1.2f) k = 1.2f;
    curvature *= k;

    // // 二倍前瞻点，使用主前瞻点实际距离的2倍
    // float actual_lookahead2 = 0.0f;
    // Point2f target2 = interpolate_path_point(path, 2.0f * actual_lookahead, actual_lookahead2);
    // float x_r2 = target2.x() - pos.x();
    // float y_r2 = pos.y() - target2.y();
    // float curvature2 = actual_lookahead2 > 1e-6f ? x_r2 / (actual_lookahead2) : 0;

    return std::tuple<Point2f, float, float>(target, curvature, actual_lookahead);
}

void MotionPlanner::setup_debug_vars()
{
    add_debug_var("speed_base", make_debug_var("speed_base", &max_speed));
    add_debug_var("LRoundDir", make_debug_var("LRoundDir", &left_round_dir));
    add_debug_var("RRoundDir", make_debug_var("RRoundDir", &right_round_dir));
    add_debug_var("RoundTime", make_debug_var("RoundTime", &max_roundabout_remain_time));
    add_debug_var("anglek1", make_debug_var("anglek1", &anglek1));
    add_debug_var("anglek2", make_debug_var("anglek2", &anglek2));
}
