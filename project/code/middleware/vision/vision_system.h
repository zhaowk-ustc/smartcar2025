#ifndef VISION_SYSTEM_H
#define VISION_SYSTEM_H

#include "zf_common_headfile.h"
#include "middleware/module_base.h"
#include "middleware/debug/debuggable.h"

class VisionSystem : public Module, public IDebuggable
{
public:
    struct Config
    {
        // 这里可以添加相机配置参数
        // 例如：分辨率、帧率等
    };
    VisionSystem(const Config& config);

    void init() override;
    void reset() override;
    void update() override;

};

#endif // VISION_SYSTEM_H