#include "zf_common_headfile.h"
#include "tasks.h"
#include "application/car/car.h"
#include "config/car_config.h"

Car car(car_config);

void app_init()
{
    car.init();
}

void mainloop_tasks()
{
    car.update_mainloop();
}

void pit5ms_tasks()
{
    car.update_pit5ms();
}

void pit10ms_tasks()
{
    car.update_pit10ms();
}

void pit20ms_tasks()
{
    car.update_pit20ms();
}