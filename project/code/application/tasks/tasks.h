#ifndef TASKS_H
#define TASKS_H

#ifdef __cplusplus
extern "C" {
#endif

void app_init();
void mainloop_tasks();
void pit5ms_tasks();
void pit10ms_tasks();
void pit20ms_tasks();

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // TASKS_H