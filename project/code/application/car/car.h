#ifndef CAR_H
#define CAR_H

#include "zf_common_headfile.h"
#include "middleware/motion/motion_controller.h"
// #include "middleware/vision/vision_system.h"
#include "application/debugger/debugger.h"
#include "application/ui/ui.h"
#include "middleware/debug/debuggable.h"

class Car : public IDebuggable
{

public:
    struct Config
    {
        MotionController::Config motion_config;
    };

    Car(const Config& config);

    // 控制循环
    void init();
    void update_mainloop();
    void update_pit5ms();
    void update_pit10ms();
    void update_pit20ms();

private:

    // 核心组件
    // VisionSystem vision_system;  // 视觉系统
    MotionController motion_controller;  // 运动控制
    Debugger debugger;
    UI ui;

    // 状态变量
    float target_speed;


    // 标志位
    bool debug_flag_ = false;
    bool ui_flag_ = false;


    void setup_debug_vars() override;

};

#endif // CAR_H
