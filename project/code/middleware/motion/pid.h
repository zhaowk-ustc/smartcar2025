#ifndef PID_H
#define PID_H

#include "zf_common_headfile.h"
#include "../debug/debuggable.h"
#include <algorithm>
#include <string>
#include <iostream>
#include <cassert>

using namespace std;

// PID控制器
class PID : public DebuggableModule
{
public:
    // PID参数
    struct Params
    {
        float kp, ki, kd, kd2;
    };

    PID(const Params& params):
        // PID参数
        kp_(params.kp), ki_(params.ki), kd_(params.kd), kd2_(params.kd2)
    {
        setup_debug_vars();
    }

    void connect_inputs(
        const float* input_setpoint,
        const float* input_measured_value,
        const float* input_feedforward = nullptr)
    {
        input_setpoint_ = input_setpoint;
        input_measured_value_ = input_measured_value;
        input_feedforward_ = input_feedforward;
    }

    void connect_outputs(float* output)
    {
        output_ = output;
    }

    void update() override
    {
        // 获取输入值
        const float setpoint = *input_setpoint_;
        const float measured_value = *input_measured_value_;
        const float feedforward = input_feedforward_ ? *input_feedforward_ : 0.0f;

        // PID算法计算
        error_ = setpoint - measured_value;
        integral_ += error_;

        // 导数计算
        derivative_ = error_ - previous_error_;

        // PID输出计算
        *output_ = kp_ * error_ +
            ki_ * integral_ +
            kd_ * derivative_ +
            kd2_ * feedforward;

        // 更新状态
        previous_error_ = error_;
    }

    void reset()
    {
        previous_error_ = 0.0f;
        integral_ = 0.0f;
        *output_ = 0.0f;
    }

    // 参数设置
    void setParams(const Params& params)
    {
        kp_ = params.kp;
        ki_ = params.ki;
        kd_ = params.kd;
        kd2_ = params.kd2;
    }

    Params getParams() const
    {
        return {kp_, ki_, kd_, kd2_};
    }

private:
    string name_;

    // PID参数
    float kp_, ki_, kd_, kd2_;
    float max_integral_, output_limit_;

    // 内部状态变量
    float error_;
    float previous_error_;
    float derivative_;
    float integral_;
    float previous_setpoint_;

    // 输入指针
    const float* input_setpoint_;
    const float* input_measured_value_;
    const float* input_feedforward_;

    // 输出变量
    float* output_;

    void setup_debug_vars() override
    {
        // 添加PID参数到调试变量
        add_debug_var("kp", make_debug_var("kp", &kp_));
        add_debug_var("ki", make_debug_var("ki", &ki_));
        add_debug_var("kd", make_debug_var("kd", &kd_));
        add_debug_var("kd2", make_debug_var("kd2", &kd2_));

        // 添加状态变量
        add_debug_var("error", make_readonly_var("error", &error_));
        add_debug_var("integral", make_readonly_var("integral", &integral_));
        add_debug_var("derivative", make_readonly_var("derivative", &derivative_));
    }
};

#endif // PID_H