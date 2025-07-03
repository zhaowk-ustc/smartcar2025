#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include "pid.h"
#include "motor.h"
#include "encoder.h"
#include "servo.h"
#include "../module_base.h"

class MotionController : public Module, public IDebuggable
{
public:
    struct Config
    {
        // 硬件配置
        Motor::config left_motor_config;
        Motor::config right_motor_config;
        Encoder::encoder_config left_encoder_config;
        Encoder::encoder_config right_encoder_config;
        Servo::config servo_config;

        // 控制参数
        PID::Params speed_pid_params;
        PID::Params direction_pid_params;
    };

    // 初始化构造函数
    MotionController(const Config& config);

    void init() override;
    void update() override;
    void reset() override;

    void connect_inputs(const float* input_speed,
                        const float* input_speed_accel,
                        const float* input_direction,
                        const float* input_direction_accel);

private:
    // 硬件组件
    Motor left_motor_;
    Motor right_motor_;
    Encoder left_encoder_;
    Encoder right_encoder_;
    Servo servo_;

    // PID控制器
    PID speed_pid_;
    PID direction_pid_;

    // 输入
    const float* input_speed_;
    const float* input_speed_accel_; 
    const float* input_direction_;
    const float* input_direction_accel_;
    float target_speed;
    float target_speed_accel;
    float target_direction;
    float target_direction_accel;


    // 内部状态
    int16 left_encoder_count_;
    int16 right_encoder_count_;

    float current_speed_;

    float speed_pid_output_;
    float direction_pid_output_;

    int16 left_motor_duty_;
    int16 right_motor_duty_;
    uint16 servo_duty_;

    void setup_debug_vars() override;

};

#endif // MOTION_CONTROLLER_H
