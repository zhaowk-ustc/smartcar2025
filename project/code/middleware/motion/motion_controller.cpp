#include "motion_controller.h"
#include "config/motion_config.h"

MotionController::MotionController(const Config& config) : left_motor_(config.left_motor_config),
right_motor_(config.right_motor_config),
left_encoder_(config.left_encoder_config),
right_encoder_(config.right_encoder_config),
servo_(config.servo_config),
speed_pid_(config.speed_pid_params)
{
    // 连接输入输出接口
    left_encoder_.connect_outputs(&left_encoder_count_);
    right_encoder_.connect_outputs(&right_encoder_count_);
    speed_pid_.connect_inputs(&target_speed, &average_encoder_count, &target_speed_accel);
    speed_pid_.connect_outputs(&speed_pid_output_);
    left_motor_.connect_inputs(&left_motor_duty_);
    right_motor_.connect_inputs(&right_motor_duty_);
    servo_.connect_inputs(&servo_dir_);

    position_.connect_inputs(&average_encoder_count, &angle_vel_);
    position_.connect_outputs(&global_x, &global_y, &global_yaw_);
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

    // 清除内部状态
    left_encoder_count_ = 0;
    right_encoder_count_ = 0;
    average_encoder_count = 0.0f;
    speed_pid_output_ = 0.0f;
    left_motor_duty_ = 0;
    right_motor_duty_ = 0;
    servo_dir_ = 0;
}

void MotionController::update()
{
    if (*input_last_updated_ == true)
    {
        target_speed = *input_speed_;
        target_speed_accel = input_speed_accel_ ? *input_speed_accel_ : 0.0f;
        target_direction = *input_direction_;
        target_direction_accel = input_direction_accel_ ? *input_direction_accel_ : 0.0f;
    }
    else
    {
        target_direction += target_direction_accel * 0.3f; // 10ms更新一次
    }

    left_encoder_.update();
    right_encoder_.update();
    average_encoder_count = (left_encoder_count_ + right_encoder_count_) / 2.0f;

    icm20602_get_gyro();
    icm20602_gyro_z -= 11;
    angle_vel_ = icm20602_gyro_transition(icm20602_gyro_z);
    position_.update();

    speed_pid_.update();

    left_motor_duty_ = static_cast<int16>(speed_pid_output_);
    right_motor_duty_ = static_cast<int16>(speed_pid_output_);

    servo_dir_ = target_direction;

    left_motor_.update();
    right_motor_.update();
    servo_.update();

    *output_global_x_ = global_x;
    *output_global_y_ = global_y;
    *output_global_yaw_ = global_yaw_;
}

void MotionController::connect_inputs(const float* input_speed,
    const float* input_speed_accel,
    const float* input_direction,
    const float* input_direction_accel,
    bool* updated)
{
    input_speed_ = input_speed;
    input_speed_accel_ = input_speed_accel;
    input_direction_ = input_direction;
    input_direction_accel_ = input_direction_accel;
    input_last_updated_ = updated;
}

void MotionController::connect_outputs(
    float* global_x,
    float* global_y,
    float* global_yaw)
{
    output_global_x_ = global_x;
    output_global_y_ = global_y;
    output_global_yaw_ = global_yaw;
}

void MotionController::setup_debug_vars()
{
    speed_pid_.export_debug_vars(this, "pid_s.");
    add_debug_var("lenc", make_readonly_var("lenc", &left_encoder_count_));
    // add_debug_var("renc", make_readonly_var("renc", &right_encoder_count_));
    // add_debug_var("spd", make_readonly_var("spd", &average_encoder_count));
    // add_debug_var("lpwm", make_readonly_var("lpwm", &left_motor_duty_));
    // add_debug_var("rpwm", make_readonly_var("rpwm", &right_motor_duty_));
    // add_debug_var("servodir", make_debug_var("servodir", &servo_dir_));
    // add_debug_var("global_x", make_readonly_var("global_x", &global_x));
    // add_debug_var("global_y", make_readonly_var("global_y", &global_y));
    // add_debug_var("global_yaw", make_readonly_var("global_yaw", &global_yaw_));
}