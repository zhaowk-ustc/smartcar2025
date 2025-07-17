#pragma once

#include "zf_common_headfile.h"
#include "../module_base.h"
#include "../debug/debuggable.h"

class Position : public Module, public DebuggableModule
{
public:
    Position() = default;

    void reset() override
    {
        x_ = 0.0f;
        y_ = 0.0f;
        yaw_ = 0.0f;
    }

    void connect_inputs(float* speed_count, float* angle_velocity)
    {
        input_encoder_count_ = speed_count;
        input_angle_velocity_ = angle_velocity;
    }

    void connect_outputs(float* x, float* y, float* yaw)
    {
        // assert(x != nullptr && y != nullptr && yaw != nullptr);
        output_pixel_x_ = x;
        output_pixel_y_ = y;
        output_yaw_ = yaw;
    }

    void update() override
    {
        // 1. 更新航向角
        yaw_ += (*input_angle_velocity_) * DT;

        float speed_pixel = count_to_pixel(*input_encoder_count_);

        // 2. 更新位置
        float yaw_rad = yaw_ * 3.14159265358979323846f / 180.0f;
        x_ += speed_pixel * cosf(yaw_rad) * DT;
        y_ += speed_pixel * sinf(yaw_rad) * DT;

        // 3. 输出
        *output_pixel_x_ = x_;
        *output_pixel_y_ = y_;
        *output_yaw_ = yaw_;
    }

    static float pixel_to_count(float pixel)
    {
        // 假设每个像素对应的计数值为1
        return pixel; // 这里可以根据实际情况调整转换比例
    }

    static float count_to_pixel(float count)
    {
        // 假设每个计数值对应的像素为1
        return count; // 这里可以根据实际情况调整转换比例
    }

private:

    static constexpr float DT = 0.01f; // 控制周期，单位：秒
    float x_;
    float y_;
    float yaw_;

    float* input_encoder_count_ = nullptr; // 输入速度
    float* input_angle_velocity_ = nullptr; // 输入角速度

    float* output_pixel_x_ = nullptr; // 输出位置x
    float* output_pixel_y_ = nullptr; // 输出位置y
    float* output_yaw_ = nullptr; // 输出朝向


};
