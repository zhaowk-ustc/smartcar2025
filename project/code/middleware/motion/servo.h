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

    void connect_inputs(const uint16* input_duty)
    {
        input_duty_ = input_duty;
    }

    void update() override
    {
        auto duty = *input_duty_;
        // 限制速度在允许范围内
        if (duty > SERVO_MAX_DUTY) duty = SERVO_MAX_DUTY;
        if (duty < SERVO_MIN_DUTY) duty = SERVO_MIN_DUTY;

        pwm_set_duty(pwm_channel_, duty);

    }

private:
    const pwm_channel_enum pwm_channel_;


    // 运行参数
    const uint16* input_duty_;

    // 常量定义
    static constexpr uint16 SERVO_FREQ = 200;
    static constexpr uint16 SERVO_MAX_DUTY = 1800 * SERVO_FREQ / 100;
    static constexpr uint16 SERVO_MIN_DUTY = 1400 * SERVO_FREQ / 100;
};

#endif // SERVO_H
