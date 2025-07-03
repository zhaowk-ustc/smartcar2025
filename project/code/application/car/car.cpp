#include "car.h"
#include <cmath>

Car::Car(const Car::Config& config)
    : motion_controller(config.motion_config)
{
    motion_controller.connect_inputs(
        &target_speed,
        &target_speed_accel,
        &target_direction,
        &target_direction_accel);

    setup_debug_vars();
}

void Car::init()
{
    // 初始化子模块
    motion_controller.init();

    ui.init();
    debugger.init();

}

void Car::update_mainloop()
{
    if (debug_flag_)
    {
        debugger.update();
        debug_flag_ = false;
    }
    if (ui_flag_)
    {
        ui.update_mainloop();
        ui_flag_ = false;
    }
    system_delay_ms(10);
}

void Car::update_pit5ms()
{
}

void Car::update_pit10ms()
{
    motion_controller.update();
}

void Car::update_pit20ms()
{
    debug_flag_ = true;
    ui_flag_ = true;
    ui.update_pit();
}

void Car::setup_debug_vars()
{
    motion_controller.export_debug_vars(&debugger, "");
    ui.export_debug_vars(&debugger, "");

    // 添加调试变量
    add_debug_var("target_speed", make_debug_var("target_speed", &target_speed));

    // 导出到 Debugger
    export_debug_vars(&debugger, "");

    // 复制到ui
    ui.import_vars(debugger.list_var_ptrs());
}
