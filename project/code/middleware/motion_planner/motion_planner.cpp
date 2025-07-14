#include "motion_planner.h"
#include "multicore/core_shared.h"
#include <algorithm>

MotionPlanner::MotionPlanner(const Config& config)
{
}

void MotionPlanner::init()
{
}

void MotionPlanner::reset()
{
}

void MotionPlanner::update()
{
    if (!input_path_ || input_path_->size() < 2)
    {
        // 如果没有输入路径或路径节点数小于2，直接返回
        *output_target_speed_ = 0.0f;
        *output_target_angle_ = 0.0f;
        return;
    }

    update_angle();
    update_speed();
}

void MotionPlanner::update_angle()
{
    static TrackPath planner_path;
    memcpy(&planner_path, &vision_outputs_shared.track_path, sizeof(vision_outputs_shared.track_path));

    if (planner_path.size() < 2)
    {
        // 如果路径节点数小于2，无法进行规划
        *output_target_speed_ = 0.0f;
        *output_target_angle_ = 0.0f;
        return;
    }

    // 1. 计算主前瞻曲率和3/4前瞻曲率和实际主前瞻距离
    float lookahead = 20.0f;
    Point2f target_point;
    float angle, angle2, actual_lookahead;
    float angle_vel = 0.0f; // 曲率变化率，暂时设为0
    std::tie(target_point, angle, angle2, actual_lookahead) = pure_pursuit(planner_path, lookahead);

    // 2. 计算曲率变化率（曲率对时间的导数，简单差分）
    // static float last_curvature = 0.0f;
    // float dt = 0.01f; // 控制周期，单位：秒（请根据实际情况调整）
    // float curvature_rate = (curvature - last_curvature) / dt;
    // last_curvature = curvature;

    vision_debug_shared.pure_pursuit_target = target_point;

    *output_target_angle_ = angle_to_servo(angle);
    *output_target_angle_vel_ = angle_to_servo(angle_vel);
}

void MotionPlanner::update_speed()
{
    // 速度更新逻辑可以在这里实现
    // 目前只是简单地将目标速度设置为0.0f
    float speed = 30;
    float accel = 0.0f; // 假设加速度为0

    *output_target_speed_ = speed;
    *output_target_speed_accel_ = accel;
}

void MotionPlanner::connect_inputs(const TrackPath* path)
{
    input_path_ = path;
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

constexpr float U_TURN_THRESHOLD = calibrated_height * 0.2f; // U型转弯的y阈值
void MotionPlanner::detect_u_turn(const TrackPath& path)
{
    if (path.size() < 2)
    {
        return;
    }
    Point2f start = path.start();
    Point2f end = path.end();

    // 如果end的y过小，认为发生了U型转弯
    if (end.y() < U_TURN_THRESHOLD)
    {
        is_u_turn = true;
        u_turn_direction_ = (end.x() > start.x()); // 根据x方向判断U型转弯方向
    }
}
