#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

#include "middleware/motion/motor.h"
#include "middleware/motion/encoder.h"
#include "middleware/motion/motion_controller.h"


constexpr Motor::config left_motor_config={
    .pwm_channel = TCPWM_CH11_P05_2, 
    .dir_pin = P05_3, 
    .direction = Motor::Direction::CW
};

constexpr Motor::config right_motor_config={
    .pwm_channel = TCPWM_CH09_P05_0,
    .dir_pin = P05_1,
    .direction = Motor::Direction::CCW
};

constexpr Encoder::encoder_config left_encoder_config={
    .encoder = TC_CH07_ENCODER,
    .count_pin = TC_CH07_ENCODER_CH1_P07_6,
    .dir_pin = TC_CH07_ENCODER_CH2_P07_7,
    .direction = Encoder::Direction::CW
};

constexpr Encoder::encoder_config right_encoder_config={
    .encoder = TC_CH20_ENCODER,
    .count_pin = TC_CH20_ENCODER_CH1_P08_1,
    .dir_pin = TC_CH20_ENCODER_CH2_P08_2,
    .direction = Encoder::Direction::CCW
};

constexpr PID::Params speed_pid_params = {
    .kp = 0.5f,
    .ki = 0.1f,
    .kd = 0.05f,
    .kd2 = 0.0f
};

constexpr PID::Params direction_pid_params = {
    .kp = 1.0f,
    .ki = 0.1f,
    .kd = 0.01f,
    .kd2 = 0.0f
};

constexpr MotionController::Config motion_config = {
   // 硬件配置
   .left_motor_config = left_motor_config,
   .right_motor_config = right_motor_config,
   .left_encoder_config = left_encoder_config,
   .right_encoder_config = right_encoder_config,

   // 控制参数
   .speed_pid_params = speed_pid_params,
   .direction_pid_params = direction_pid_params
};




#endif // HARDWARE_CONFIG_H