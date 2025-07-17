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

    };

    // 初始化构造函数
    Guandao_module(const Config& config);

    void init() override;
    void update() override;
    void reset() override;

    void connect_inputs();

private:

    void setup_debug_vars() override;

};

#endif // GUANDAO_MODULE_H
