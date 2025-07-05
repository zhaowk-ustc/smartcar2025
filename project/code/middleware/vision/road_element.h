#pragma once

#include "zf_common_headfile.h"
#include <vector>
using namespace std;

// 道路元素检测相关常量
const int16 BOUNDARY_MUTATION_THRESHOLD = 15;              // 边界突变检测阈值（像素）
const int CONSECUTIVE_POSITION_MUTATION_THRESHOLD = 2;    // 连续位置突变判定阈值
const int CONSECUTIVE_DISAPPEAR_MUTATION_THRESHOLD = 5;   // 连续消失突变判定阈值
const int ROUNDABOUT_DETECTION_SAMPLES = 38;              // 环岛检测的采样点数
const float ROUNDABOUT_NONLINEARITY_THRESHOLD = 8.0f;     // 环岛侧边界非线性度阈值
const int ROUNDABOUT_WIDTH_DIFF_THRESHOLD = 30;           // 左右边界差距阈值（像素）

// 道路元素类型枚举（统一所有类型）
enum RoadElementType {
    NO_ELEMENT = 0,               // 无特殊元素
    LEFT_TURN = 1,               // 左直角
    RIGHT_TURN = 2,              // 右直角
    CROSS_ROAD = 3,              // 十字路口
    LEFT_ROUNDABOUT = 4,         // 左侧环岛
    RIGHT_ROUNDABOUT = 5         // 右侧环岛
};

// 边界异常类型枚举（用于内部检测）
enum BoundaryAnomalyType {
    NO_ANOMALY = 0,               // 无异常
    BOUNDARY_MUTATION = 1,        // 边界突变
    BOUNDARY_DISAPPEAR = 2        // 边界消失
};

// 道路元素检测结果结构体
struct RoadElementResult {
    RoadElementType type;        // 元素类型
    int position;               // 元素位置（行号，左右平均值或唯一值）
    float confidence;           // 置信度（0-1）
    
    RoadElementResult() : type(NO_ELEMENT), position(-1), confidence(0.0f) {}
    RoadElementResult(RoadElementType t, int pos, float conf = 1.0f) 
        : type(t), position(pos), confidence(conf) {}
};

// 边界异常检测结果结构体
struct BoundaryAnomalyResult {
    BoundaryAnomalyType type;    // 异常类型
    int position;               // 异常位置（行号）
    float confidence;           // 置信度（0-1）
    
    BoundaryAnomalyResult() : type(NO_ANOMALY), position(-1), confidence(0.0f) {}
    BoundaryAnomalyResult(BoundaryAnomalyType t, int pos, float conf = 1.0f)
        : type(t), position(pos), confidence(conf) {}
};

// 检测单侧边界异常函数
// 参数：bounds - 边界数组（左边界或右边界）
// 返回：边界异常检测结果
BoundaryAnomalyResult detect_boundary_anomaly(const vector<int16>& bounds);

// 统一的道路元素检测函数（推荐使用）
// 参数：left_bounds - 左边界数组
//      right_bounds - 右边界数组
// 返回：道路元素检测结果
RoadElementResult detect_road_element(const vector<int16>& left_bounds, 
                                     const vector<int16>& right_bounds);

// 内部环岛检测函数
// 参数：left_bounds - 左边界数组
//      right_bounds - 右边界数组
// 返回：环岛检测结果
RoadElementResult detect_roundabout_internal(const vector<int16>& left_bounds, 
                                            const vector<int16>& right_bounds);

// 道路元素类型转换为字符串的辅助函数
const char* road_element_type_to_string(RoadElementType type);
