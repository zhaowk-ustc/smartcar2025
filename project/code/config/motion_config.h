#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

#include "middleware/motion/motor.h"
#include "middleware/motion/encoder.h"
#include "middleware/motion/servo.h"
#include "middleware/motion/pid.h"
#include "middleware/motion/motion_controller.h"
#include "middleware/motion_planner/motion_planner.h"

constexpr PID::Params speed_pid_params = {
    .kp = 4.0f,
    .ki = 0.3f,
    .kd = 0.05f,
    .kd2 = 0.0f
};


constexpr Motor::config left_motor_config = {
    .pwm_channel = TCPWM_CH30_P10_2,
    .dir_pin = P10_3,
    .direction = Motor::Direction::CCW
};

constexpr Motor::config right_motor_config = {
    .pwm_channel = TCPWM_CH24_P09_0,
    .dir_pin = P09_1,
    .direction = Motor::Direction::CW
};

constexpr Encoder::encoder_config left_encoder_config = {
    .encoder = TC_CH07_ENCODER,
    .count_pin = TC_CH07_ENCODER_CH1_P07_6,
    .dir_pin = TC_CH07_ENCODER_CH2_P07_7,
    .direction = Encoder::Direction::CCW
};

constexpr Encoder::encoder_config right_encoder_config = {
    .encoder = TC_CH20_ENCODER,
    .count_pin = TC_CH20_ENCODER_CH1_P08_1,
    .dir_pin = TC_CH20_ENCODER_CH2_P08_2,
    .direction = Encoder::Direction::CW
};

constexpr Servo::config servo_config = {
    .pwm_channel = TCPWM_CH13_P00_3
};


constexpr MotionController::Config motion_controller_config = {
    // 硬件配置
    .left_motor_config = left_motor_config,
    .right_motor_config = right_motor_config,
    .left_encoder_config = left_encoder_config,
    .right_encoder_config = right_encoder_config,
    .servo_config = servo_config,

    // 控制参数
    .speed_pid_params = speed_pid_params
};




#endif // HARDWARE_CONFIG_H