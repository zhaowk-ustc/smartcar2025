#pragma once

#include "line_tracking_graph.h"
#include <vector>
#include <string>

/**
 * 赛道元素识别结果结构体
 */
struct TrackElement {
    enum Type {
        NONE,
        CROSS,           // 十字路口
        ROUNDABOUT,      // 六边形环岛
    };
    
    Type type;
    Point center;                    // 元素中心点
    double confidence;               // 识别置信度
};

pair<vector<Point>, vector<TrackElement>> getLineAndTrackElements(const LineTrackingGraph& graph);
