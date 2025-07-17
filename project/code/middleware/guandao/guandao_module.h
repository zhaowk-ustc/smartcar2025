#ifndef GUANDAO_MODULE_H
#define GUANDAO_MODULE_H

#include "Guandao_Plus.h"
#include "../module_base.h"
#include "../debug/debuggable.h"

class Guandao_module : public Module, public IDebuggable
{
public:
    struct Config
    {
        // 可扩展配置参数
    };

    // 初始化构造函数
    Guandao_module(const Config& config);

    void init() override;
    void update() override;
    void reset() override;

    void connect_inputs();

    // 新增：手动设置编码器数值
    void set_encoder_values(int16_t left, int16_t right);

    // 新增：获取当前位置
    float get_x() const { return X; }
    float get_y() const { return Y; }

private:
    int16_t encoder_left = 0;
    int16_t encoder_right = 0;

    void setup_debug_vars() override;

};

#endif // GUANDAO_MODULE_H
