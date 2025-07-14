#ifndef SERVO_H
#define SERVO_H


#include "zf_common_headfile.h"
#include "middleware/module_base.h"

class Servo : public Module
{
public:
    struct config
    {
        pwm_channel_enum pwm_channel;
    };

    Servo() = delete;

    Servo(const config& config)
        : pwm_channel_(config.pwm_channel)
    {
    }

    // 初始化舵机硬件
    void init() override
    {
        pwm_init(pwm_channel_, SERVO_FREQ, 0);
    }

    void reset() override
    {
        // 停止电机
        pwm_set_duty(pwm_channel_, (SERVO_MIN_DUTY + SERVO_MAX_DUTY) / 2);
    }

    void connect_inputs(const float* input_dir)
    {
        input_dir_ = input_dir;
    }

    void update() override
    {
        auto dir = *input_dir_;
        // 限制速度在允许范围内
        auto duty = duty_mapping(dir);
        pwm_set_duty(pwm_channel_, duty);

    }

private:
    const pwm_channel_enum pwm_channel_;


    // 运行参数
    const float* input_dir_;

    // 常量定义
    static constexpr uint16 SERVO_FREQ = 200;
    static constexpr uint16 SERVO_MAX_DUTY = 1800 * SERVO_FREQ / 100;
    static constexpr uint16 SERVO_MIN_DUTY = 1400 * SERVO_FREQ / 100;

    int duty_mapping(float dir)
    {
        // 将方向映射到舵机的占空比范围
        dir = -dir;
        if (dir < -1.0f) dir = -1.0f;
        if (dir > 1.0f) dir = 1.0f;
        return static_cast<int>(SERVO_MIN_DUTY + (SERVO_MAX_DUTY - SERVO_MIN_DUTY) * (dir + 1.0f) / 2.0f);
    }
};

#endif // SERVO_H
