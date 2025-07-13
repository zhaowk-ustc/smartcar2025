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
    static TrackPath planner_path;
    memcpy(&planner_path, &vision_outputs_shared.track_path, sizeof(vision_outputs_shared.track_path));

    // 1. 计算主前瞻曲率和3/4前瞻曲率和实际主前瞻距离
    float lookahead = 20.0f;
    Point2f target_point;
    float angle, curvature2, actual_lookahead;
    std::tie(target_point, angle, curvature2, actual_lookahead) = pure_pursuit(planner_path, lookahead);

    // 2. 计算曲率变化率（曲率对时间的导数，简单差分）
    // static float last_curvature = 0.0f;
    // float dt = 0.01f; // 控制周期，单位：秒（请根据实际情况调整）
    // float curvature_rate = (curvature - last_curvature) / dt;
    // last_curvature = curvature;

    // 3. 速度设定为 max(某个常数, 某个常数/|曲率|)
    // constexpr float kMinSpeed = 0.5f; // 最低速度（单位：m/s，可调整）
    // constexpr float kCurvatureSpeedFactor = 10.0f; // 曲率速度因子（可调整）
    // float speed = std::max(kMinSpeed, kCurvatureSpeedFactor / (std::abs(curvature) + 1e-3f));
    float speed = 30;
    // 4. 加速度（速度对时间的导数，简单差分）
    // static float last_speed = 0.0f;
    // float accel = (speed - last_speed) / dt;
    // last_speed = speed;

    // 5. 保存到成员变量
    *output_target_speed_ = speed;
    // target_speed_accel_ = accel;
    vision_debug_shared.pure_pursuit_target = target_point;
    *output_target_angle_ = angle_to_servo(angle);
    // target_curvature_rate_ = curvature_rate;
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
