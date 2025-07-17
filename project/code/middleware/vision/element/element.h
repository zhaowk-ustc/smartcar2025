#pragma once


#include "../track/line_tracking_graph.h"
#include <utility>
#include <complex>

using namespace std;

// 元素类型
enum class ElementType : int8_t
{
    NORMAL,
    CROSS,      // 十字路口
    LEFT_ROUNDABOUT, // 左侧环岛
    RIGHT_ROUNDABOUT, // 右侧环岛
    U_TURN,     // U型转弯
    UNCERTAIN,  // 不确定
};

pair<ElementType, int> detect_branch_element(const Point2f& in_vec, const vector<Point2f>& out_vecs);