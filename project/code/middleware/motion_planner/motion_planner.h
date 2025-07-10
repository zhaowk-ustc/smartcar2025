#ifndef MOTION_PLANNER_H
#define MOTION_PLANNER_H

#include "element/track_path.h"
#include "../module_base.h"
#include "../debug/debuggable.h"

class MotionPlanner :public Module, public IDebuggable
{
public:
    struct Config
    {
        float max_speed;
        float max_accel;
        float max_angle;
        // 可扩展更多参数
    };

    MotionPlanner(const Config& config);

    void init();
    void reset();
    void update();
    void connect_inputs(const TrackPath* path);

private:
    const TrackPath* input_path_ = nullptr;

    static float encoder_count_to_distance(float count)
    {
        return count * 0.01f; // 例如每个计数0.01米
    }

    static float angle_to_servo(float angle)
    {
        // 把角度映射到(-1,1)范围
        return angle / 90.0f; // 假设最大角度为90
    }

    float current_speed_ = 0.0f; // 当前速度
    float current_speed_accel_ = 0.0f; // 当前速度加速度
    float target_speed_ = 0.0f; // 目标速度
    float target_speed_accel_ = 0.0f; // 目标速度加速度
    float current_angle_ = 0.0f; // 当前角度
    float current_angle_vel_ = 0.0f; // 当前角速度
    float current_angle_accel_ = 0.0f; // 当前角加速度
    float target_angle_ = 0.0f; // 目标角度
    float target_angle_vel_ = 0.0f; // 目标角速度
    float target_angle_accel_ = 0.0f; // 目标角加速度

};

#endif // MOTION_PLANNER_H
