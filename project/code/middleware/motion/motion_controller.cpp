#include "motion_controller.h"
#include "config/motion_config.h"

MotionController::MotionController(const Config& config) :
    left_motor_(config.left_motor_config),
    right_motor_(config.right_motor_config),
    left_encoder_(config.left_encoder_config),
    right_encoder_(config.right_encoder_config),
    servo_(config.servo_config),
    speed_pid_(config.speed_pid_params),
    direction_pid_(config.direction_pid_params)
{
    // 连接输入输出接口
    left_encoder_.connect_outputs(&left_encoder_count_);
    right_encoder_.connect_outputs(&right_encoder_count_);

    speed_pid_.connect_inputs(&target_speed, &current_speed_, &target_speed_accel);
    speed_pid_.connect_outputs(&speed_pid_output_);
    left_motor_.connect_inputs(&left_motor_duty_);
    right_motor_.connect_inputs(&right_motor_duty_);
    servo_.connect_inputs(&servo_duty_);
    setup_debug_vars();
}

void MotionController::init()
{
    // 初始化电机和编码器
    left_motor_.init();
    right_motor_.init();
    left_encoder_.init();
    right_encoder_.init();
    servo_.init();

    icm20602_init();
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
    servo_duty_ = 0;
}

void MotionController::update()
{
    target_speed = *input_speed_;
    target_speed_accel = input_speed_accel_ ? *input_speed_accel_ : 0.0f;
    target_direction = *input_direction_;
    target_direction_accel = input_direction_accel_ ? *input_direction_accel_ : 0.0f;

    left_encoder_.update();
    right_encoder_.update();

    current_speed_ = (left_encoder_count_ + right_encoder_count_) / 2.0f;

    speed_pid_.update();

    left_motor_duty_ = static_cast<int16>(speed_pid_output_);
    right_motor_duty_ = static_cast<int16>(speed_pid_output_);

    left_motor_.update();
    right_motor_.update();
    servo_.update();
}

void MotionController::connect_inputs(
    const float* input_speed,
    const float* input_speed_accel,
    const float* input_direction,
    const float* input_direction_accel)
{
    input_speed_ = input_speed;
    input_speed_accel_ = input_speed_accel;
    input_direction_ = input_direction;
    input_direction_accel_ = input_direction_accel;
}

void MotionController::setup_debug_vars()
{
    speed_pid_.export_debug_vars(this, "pid_s.");
    direction_pid_.export_debug_vars(this, "pid_d.");
    add_debug_var("lenc", make_readonly_var("lenc", &left_encoder_count_));
    add_debug_var("renc", make_readonly_var("renc", &right_encoder_count_));
    add_debug_var("spd", make_readonly_var("spd", &current_speed_));
    add_debug_var("lpwm", make_readonly_var("lpwm", &left_motor_duty_));
    add_debug_var("rpwm", make_readonly_var("rpwm", &right_motor_duty_));
    add_debug_var("servopwm", make_debug_var("servopwm", &servo_duty_));
}