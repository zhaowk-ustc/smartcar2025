#include "guandao_module.h"
#include "Guandao_Plus.h"

// 构造函数
Guandao_module::Guandao_module(const Config& config)
{
    // 可初始化参数
}

void Guandao_module::init()
{
    // 初始化相关变量
    encoder_left = 0;
    encoder_right = 0;
    X = 0;
    Y = 0;
}

void Guandao_module::reset()
{
    // 重置相关变量
    encoder_left = 0;
    encoder_right = 0;
    X = 0;
    Y = 0;
}

void Guandao_module::set_encoder_values(int16_t left, int16_t right)
{
    encoder_left = left;
    encoder_right = right;
}

void Guandao_module::update()
{
    // 手动调用编码器处理
    Update_Wheel_Pulses(&wheel_left, encoder_left);
    Update_Wheel_Pulses(&wheel_right, encoder_right);

    // 更新位置（假设每次调用代表小车移动了distance）
    Distance_Get_Plus();
}

void Guandao_module::connect_inputs()
{
    // 可扩展输入连接
}

void Guandao_module::setup_debug_vars()
{
    // 可扩展调试变量
}