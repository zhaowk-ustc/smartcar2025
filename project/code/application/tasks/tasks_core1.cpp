#include "zf_common_headfile.h"
#include "middleware/vision/vision_system.h"
#include "tasks_core1.h"

VisionSystem vision_system({});

void app_init_core1()
{
    vision_system.init();
}

void mainloop_tasks_core1()
{
    vision_system.update();

}

void pit5ms_tasks_core1()
{

}

void pit10ms_tasks_core1()
{

}

void pit20ms_tasks_core1()
{

}