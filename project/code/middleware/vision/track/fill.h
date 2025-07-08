#pragma once

#include "zf_common_headfile.h"
#include "../common/point.h"

// 通用的连通域填充函数
std::vector<Point> flood_fill(
    const uint8_t* image,
    uint8_t* visited,
    int image_w,
    int image_h,
    Point start,
    uint8_t fill_value
);

// 从种子点开始对连通域做层次遍历并染色，染色值为层次+1
std::vector<Point> flood_fill_with_depth(
    const uint8_t* image,
    uint8_t* depth_map,
    int image_w,
    int image_h,
    Point start
);
