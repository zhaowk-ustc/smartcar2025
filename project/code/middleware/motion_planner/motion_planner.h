#ifndef MOTION_PLANNER_H
#define MOTION_PLANNER_H

#include "middleware/vision/element/track_path.h"
#include "../module_base.h"
#include "../debug/debuggable.h"

class MotionPlanner :public Module, public IDebuggable
{
public:
    struct Config
    {
        float max_speed;
        float max_speed_accel;
        float max_curvature;
        float max_curvature_rate;
        // 可扩展更多参数
    };

    MotionPlanner(const Config& config);

    void init();
    void reset();
    void update();
    void connect_inputs(const TrackPath* path);
    void connect_outputs(float* target_speed, float* target_speed_accel,
        float* target_angle, float* target_angle_vel);

private:
    const TrackPath* input_path_ = nullptr;

    // static float encoder_count_to_distance(float count)
    // {
    //     return count * 0.00030679616f;
    // }
    void update_angle();
    void update_speed();

    static float angle_to_servo(float angle)
    {
        // 左负右正
        // 把角度映射到(-1, 1)范围内
        return angle;
    }

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

    // Pure Pursuit 跟踪算法，输入路径、前瞻距离，输出主前瞻target点、主前瞻曲率、3/4前瞻曲率、实际主前瞻距离
    static std::tuple<Point2f, float, float, float> pure_pursuit(const TrackPath& path, float lookahead);

    void detect_u_turn(const TrackPath& path);
    bool is_u_turn = false;
    bool u_turn_direction_ = false; // 回弯的方向，false表示左侧回弯，true表示右侧回弯

    bool in_roundabout_ = false; // 是否在环岛
    bool roundabout_direction_ = false; // 环岛行驶方向，false表示左转，true表示右转
    Point2f roundabout_point_; // 环岛入口点
    bool in_cross_ = false; // 是否在十字路口
    Point2f cross_point_; // 十字路口入口点

    


};

#endif // MOTION_PLANNER_H
