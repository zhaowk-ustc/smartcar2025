#ifndef MOTION_PLANNER_H
#define MOTION_PLANNER_H

#include "middleware/vision/element/track_path.h"
#include "../module_base.h"
#include "../debug/debuggable.h"
#include "middleware/vision/element/element.h"

class MotionPlanner :public Module, public IDebuggable
{
public:
    MotionPlanner();

    void init();
    void reset();
    void update();
    void connect_inputs(const TrackPath* path, const float* current_x, const float* current_y, const float* current_yaw);
    void connect_outputs(float* target_speed, float* target_speed_accel,
        float* target_angle, float* target_angle_vel);

    int update_count;

private:
    const TrackPath* input_path_ = nullptr;
    const float* input_current_x_ = nullptr;
    const float* input_current_y_ = nullptr;
    const float* input_current_yaw_ = nullptr;
    TrackPath planner_local_path;
    // TrackPath planner_global_path_tmp;

    void fix_path();
    void update_element();

    bool miss_line = false; // 是否偏离路径

    ElementType current_element_type = ElementType::NORMAL; // 当前元素类型
    Point2f current_element_point; // 当前元素的关键点
    Point2f current_element_target_dir; // 当前元素的目标方向

    bool in_roundabout_ = false; // 是否在环岛
    bool roundabout_direction_ = false; // 环岛行驶方向，false表示左转，true表示右转
    int max_roundabout_remain_time = 30;
    int roundabout_remain_time = 0;
    int roundabout_protect_time = 0; // 环岛保护时间，单位为10ms
    float left_round_dir = -6.5f;
    float right_round_dir = 5.0f;


    void update_angle();
    float base_lookahead = 27.0f; // 前瞻距离
    float angle = 0.0f; // 主前瞻曲率
    float angle2 = 0.0f; // 3/4前瞻
    float actual_lookahead, actual_lookahead2;
    float angle_vel = 0.0f;


    int max_speed = 150;
    float speed = 0.0f; // 目标速度
    float speed_accel = 0.0f; // 目标速度加速度


    // Pure Pursuit 跟踪算法，输入路径、前瞻距离，输出主前瞻target点、主前瞻曲率、3/4前瞻曲率、实际主前瞻距离
    static std::tuple<Point2f, float, float> pure_pursuit(const TrackPath& path, float lookahead);

    float angle_to_servo(float angle)
    {
        // 左负右正
        // 把角度映射到(-1, 1)范围内
        return angle * anglek1 + angle * abs(angle) * anglek2;
    }

    float anglek1 = 1.10f;
    float anglek2 = 0.4f;


    void update_speed();

    float* output_target_speed_; // 目标速度
    float* output_target_speed_accel_; // 目标速度加速度
    float* output_target_angle_; // 目标曲率
    float* output_target_angle_vel_;

    void setup_debug_vars() override;
};

#endif // MOTION_PLANNER_H
