#include "motion_controller.h"
#include "config/motion_config.h"

MotionController::MotionController(const Config& config) :
    left_motor_(config.left_motor_config),
    right_motor_(config.right_motor_config),
    left_encoder_(config.left_encoder_config),
    right_encoder_(config.right_encoder_config),
    speed_pid_(config.speed_pid_params),
    direction_pid_(config.direction_pid_params)
{
    // 连接输入输出接口
    left_encoder_.connect_outputs(&left_encoder_count_);
    right_encoder_.connect_outputs(&right_encoder_count_);

    speed_pid_.connect_inputs(
        target_speed_,
        &current_speed_,
        nullptr // 无前馈输入
    );

    speed_pid_.connect_outputs(&speed_pid_output_);

    left_motor_.connect_inputs(&left_motor_duty_);
    right_motor_.connect_inputs(&right_motor_duty_);

    setup_debug_vars();

}

void MotionController::init()
{
    // 初始化电机和编码器
    left_motor_.init();
    right_motor_.init();
    left_encoder_.init();
    right_encoder_.init();

}

void MotionController::reset()
{
    // 重置电机和编码器状态
    left_motor_.reset();
    right_motor_.reset();
    left_encoder_.reset();
    right_encoder_.reset();

    // 重置PID控制器状态
    speed_pid_.reset();
    direction_pid_.reset();

    // 清除内部状态
    left_encoder_count_ = 0;
    right_encoder_count_ = 0;
    current_speed_ = 0.0f;
    speed_pid_output_ = 0.0f;
    direction_pid_output_ = 0.0f;
    left_motor_duty_ = 0;
    right_motor_duty_ = 0;
}

void MotionController::update()
{
    left_encoder_.update();
    right_encoder_.update();

    current_speed_ = (left_encoder_count_ + right_encoder_count_) / 2.0f;

    speed_pid_.update();

    left_motor_duty_ = static_cast<int16>(speed_pid_output_);
    right_motor_duty_ = static_cast<int16>(speed_pid_output_);

    left_motor_.update();
    right_motor_.update();
}

void MotionController::connect_inputs(const float* target_speed)
{
    target_speed_ = target_speed;
}

void MotionController::setup_debug_vars()
{
    speed_pid_.export_debug_vars(this, "speed_pid.");
    direction_pid_.export_debug_vars(this, "dir_pid.");
    add_debug_var("left_encoder_count", make_readonly_var("left_encoder_count", &left_encoder_count_));
    add_debug_var("right_encoder_count", make_readonly_var("right_encoder_count", &right_encoder_count_));
    add_debug_var("current_speed", make_readonly_var("current_speed", &current_speed_));
    add_debug_var("left_motor_duty", make_readonly_var("left_motor_duty", &left_motor_duty_));
    add_debug_var("right_motor_duty", make_readonly_var("right_motor_duty", &right_motor_duty_));
}