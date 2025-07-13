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

    static float angle_to_servo(float angle)
    {
        // 把角度映射到(-1, 1)范围内
        return -angle / 3;
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

    // // 屏幕坐标系转换为现实坐标系
    // static Point2f screen_to_world(const Point2f& pt_screen, float cx, float cy, float scale = 1.0f)
    // {
    //     // fx, fy: 像素到实际距离的缩放系数（米/像素）
    //     // cx, cy: 屏幕中心在像素坐标系下的位置
    //     // scale: 额外缩放因子（如有需要）
    //     float x_world = (pt_screen.x() - cx) * scale;
    //     float y_world = (pt_screen.y() - cy) * scale;
    //     return Point2f(x_world, y_world);
    // }

    // 批量将TrackPath从屏幕坐标转换为现实坐标
    // static void path_screen_to_world(TrackPath& path_world, const TrackPath& path_screen, float cx, float cy, float scale = 1.0f)
    // {
    //     path_world = path_screen;
    //     for (auto i = 0; i < path_world.size(); ++i)
    //     {
    //         path_world[i].pos = screen_to_world(path_world[i].pos, cx, cy, scale);
    //     }
    // }


};

#endif // MOTION_PLANNER_H
