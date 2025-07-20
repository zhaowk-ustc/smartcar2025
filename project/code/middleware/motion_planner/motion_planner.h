#ifndef MOTION_PLANNER_H
#define MOTION_PLANNER_H

#include "middleware/vision/element/track_path.h"
#include "../module_base.h"
#include "../debug/debuggable.h"
#include "middleware/vision/element/element.h"

class MotionPlanner :public Module, public IDebuggable
{
public:
    MotionPlanner() = default;

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
    TrackPath planner_global_path_tmp;

    void update_element();

    bool miss_line = false; // 是否偏离路径

    static std::tuple<bool, bool, Point2f> detect_u_turn(const TrackPath& path);
    ElementType current_element_type = ElementType::NORMAL; // 当前元素类型
    Point2f current_element_point; // 当前元素的关键点
    Point2f current_element_target_dir; // 当前元素的目标方向

    bool u_turn_direction_ = false; // U型转弯方向，false表示左侧，true表示右侧

    bool in_roundabout_ = false; // 是否在环岛
    bool roundabout_direction_ = false; // 环岛行驶方向，false表示左转，true表示右转




    void update_angle();
    float lookahead_distance = 20.0f; // 前瞻距离
    float angle = 0.0f; // 主前瞻曲率
    float angle2 = 0.0f; // 3/4前瞻


    // Pure Pursuit 跟踪算法，输入路径、前瞻距离，输出主前瞻target点、主前瞻曲率、3/4前瞻曲率、实际主前瞻距离
    static std::tuple<Point2f, float, float, float> pure_pursuit(const TrackPath& path, float lookahead);

    static float angle_to_servo(float angle)
    {
        // 左负右正
        // 把角度映射到(-1, 1)范围内
        return angle;
    }


    void update_speed();



    // float get_lookahead_distance(float speed) const
    // {
    //     // 根据目标速度计算前瞻距离
    //     // 这里可以根据实际需求调整前瞻距离的计算方式
    //     return 0.5f + speed * 0.01f;
    // }

    float* output_target_speed_; // 目标速度
    float* output_target_speed_accel_; // 目标速度加速度
    float* output_target_angle_; // 目标曲率
    float* output_target_angle_vel_;

    void path_local_to_global(TrackPath& global_path, const TrackPath& local_path);
    void path_global_to_local(TrackPath& local_path, const TrackPath& global_path);

};

#endif // MOTION_PLANNER_H
