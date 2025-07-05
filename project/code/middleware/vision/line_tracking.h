#pragma once

#include "zf_common_headfile.h"
#include "point.h"
#include "road_element.h"
#include "vision_utils.h"
#include <vector>
using namespace std;

// 寻找所有行的左边界，返回左边界数组
vector<int16> find_left_bounds(const uint8* image, uint16 width, uint16 height,
    const vector<int16>& extend_search_offsets);

// 寻找所有行的右边界，返回右边界数组
vector<int16> find_right_bounds(const uint8* image, uint16 width, uint16 height,
    const vector<int16>& extend_search_offsets);

// 智能偏差计算函数，结合道路元素检测结果
// 参数：road_line - 整理后的道路中心线
//      road_element - 道路元素检测结果
//      width - 图像宽度，用于计算中心点
// 返回：计算得到的偏差值（负值表示左偏，正值表示右偏）
float get_bias(const vector<int16>& road_line, 
               const RoadElementResult& road_element, uint16 width = 80);


// 道路补线函数，专注于处理虚线和边界缺失
// 参数：left - 左边界数组
//      right - 右边界数组
//      road_element - 道路元素检测结果
// 返回：补线后的道路中心线（第一个无效点前总是有效，后面都是无效点）
vector<int16> process_road_line(const vector<int16>& left, const vector<int16>& right);

