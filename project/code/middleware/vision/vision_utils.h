#pragma once

#include "zf_common_headfile.h"
#include <vector>
#include <utility>
using namespace std;
#include "Point.h"

// 二次拟合系数结构体
struct QuadraticCoefficients {
    float a, b, c;
    bool valid;
    
    QuadraticCoefficients();
    QuadraticCoefficients(float _a, float _b, float _c);
};

// 数学计算工具函数

// 计算二次拟合系数
QuadraticCoefficients calculate_quadratic_coefficients(const vector<pair<float, float>>& points);

// 统一的二次函数最小二乘拟合曲率计算
float calculate_quadratic_curvature(const vector<pair<float, float>>& points);

// 线性插值工具函数
float linear_interpolation(float x1, float y1, float x2, float y2, float x);

// 计算边界非线性度的辅助函数
// 参数：bounds - 边界数组
//      start_row - 起始行
//      end_row - 结束行
// 返回：边界非线性度指标
float calculate_boundary_nonlinearity(const vector<int16>& bounds, int start_row, int end_row);
