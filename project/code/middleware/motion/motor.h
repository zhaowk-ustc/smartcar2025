#ifndef MOTOR_H
#define MOTOR_H


#include "zf_common_headfile.h"

class Motor : public Module
{
public:
    enum class Direction
    {
        CCW,
        CW
    };

    struct config
    {
        pwm_channel_enum pwm_channel;
        gpio_pin_enum dir_pin;
        Direction direction;
        config() = delete;
    };

    Motor() = delete;

    Motor(const config& config)
        : pwm_channel_(config.pwm_channel),
        dir_pin_(config.dir_pin),
        direction_(config.direction),
        input_duty_(nullptr)
    {
    }

    // 初始化电机硬件
    void init() override
    {
        pwm_init(pwm_channel_, PWM_FREQUENCY, 0);
        gpio_init(dir_pin_, GPO, GPIO_LOW, GPO_PUSH_PULL);
    }

    void reset() override
    {
        // 停止电机
        pwm_set_duty(pwm_channel_, 0);
        gpio_set_level(dir_pin_, GPIO_LOW);
    }

    void connect_inputs(const int16* input_duty)
    {
        input_duty_ = input_duty;
    }

    void update() override
    {
        auto duty = *input_duty_;
        // 限制速度在允许范围内
        if (duty > static_cast<int16>(MAX_DUTY)) duty = static_cast<int16>(MAX_DUTY);
        if (duty < -static_cast<int16>(MAX_DUTY)) duty = -static_cast<int16>(MAX_DUTY);

        if (duty >= 0)
        {
            gpio_set_level(dir_pin_, direction_ == Direction::CW ? GPIO_LOW : GPIO_HIGH);
            pwm_set_duty(pwm_channel_, duty);
        }
        else
        {
            gpio_set_level(dir_pin_, direction_ == Direction::CW ? GPIO_HIGH : GPIO_LOW);
            pwm_set_duty(pwm_channel_, -duty);
        }
    }

private:
    const pwm_channel_enum pwm_channel_;
    const gpio_pin_enum dir_pin_;
    const Direction direction_;

    // 运行参数
    const int16* input_duty_;

    // 常量定义
    static constexpr uint16 PWM_FREQUENCY = 17000;
    static constexpr uint16 MAX_DUTY = 9000;
    static constexpr uint16 MIN_DUTY = 0;
};

#endif // MOTOR_H
