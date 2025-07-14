#ifndef VISION_SYSTEM_H
#define VISION_SYSTEM_H

#include "zf_common_headfile.h"
#include "middleware/module_base.h"
#include "middleware/debug/debuggable.h"
#include "middleware/vision/common/point.h"

class VisionSystem : public Module, public IDebuggable
{
public:
    struct Config
    {
    };
    VisionSystem(const Config& config);

    void init() override;
    void reset() override;
    void update() override;

    Point last_start_point; // 上一次的起点
    Point last_end_point; // 上一次的终点

};

#endif // VISION_SYSTEM_H