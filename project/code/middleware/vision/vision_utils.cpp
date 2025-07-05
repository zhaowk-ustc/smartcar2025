#include "vision_utils.h"
#include "road_element.h"
#include "point.h"
#include <cmath>
#include <cstdlib>
#include <algorithm>

// 二次拟合系数结构体构造函数实现
QuadraticCoefficients::QuadraticCoefficients() : a(0), b(0), c(0), valid(false) {}
QuadraticCoefficients::QuadraticCoefficients(float _a, float _b, float _c) : a(_a), b(_b), c(_c), valid(true) {}

// 计算二次拟合系数
QuadraticCoefficients calculate_quadratic_coefficients(const vector<pair<float, float>>& points)
{
    if (points.size() < 3) return QuadraticCoefficients();
    
    // 使用最小二乘法拟合二次多项式 y = ax² + bx + c
    float sum_x = 0, sum_x2 = 0, sum_x3 = 0, sum_x4 = 0;
    float sum_y = 0, sum_xy = 0, sum_x2y = 0;
    int n = points.size();
    
    for (const auto& point : points)
    {
        float x = point.first;
        float y = point.second;
        float x2 = x * x;
        float x3 = x2 * x;
        float x4 = x3 * x;
        
        sum_x += x;
        sum_x2 += x2;
        sum_x3 += x3;
        sum_x4 += x4;
        sum_y += y;
        sum_xy += x * y;
        sum_x2y += x2 * y;
    }
    
    // 构建法线方程组矩阵
    // [n    sum_x   sum_x2 ] [c]   [sum_y  ]
    // [sum_x sum_x2 sum_x3 ] [b] = [sum_xy ]
    // [sum_x2 sum_x3 sum_x4] [a]   [sum_x2y]
    
    float det = n * sum_x2 * sum_x4 + 2 * sum_x * sum_x2 * sum_x3 
              - sum_x2 * sum_x2 * sum_x2 - n * sum_x3 * sum_x3 - sum_x * sum_x * sum_x4;
    
    if (abs(det) < 1e-10) return QuadraticCoefficients();  // 矩阵奇异
    
    // 使用克拉默法则求解系数 a, b, c
    float det_a = n * sum_x2 * sum_x2y + sum_x * sum_xy * sum_x3 + sum_x2 * sum_y * sum_x3
                - sum_x2 * sum_x2 * sum_y - n * sum_x3 * sum_xy - sum_x * sum_x2y * sum_x2;
    float det_b = n * sum_x2 * sum_xy + sum_x * sum_y * sum_x4 + sum_x2 * sum_x2y * sum_x
                - sum_x2 * sum_xy * sum_x2 - n * sum_x4 * sum_y - sum_x * sum_x2y * sum_x2;
    float det_c = sum_x2 * sum_xy * sum_x3 + sum_x * sum_x2y * sum_x2 + sum_x2 * sum_y * sum_x3
                - sum_x2 * sum_x2y * sum_x2 - sum_x2 * sum_xy * sum_x2 - sum_x * sum_y * sum_x4;
    
    float a = det_a / det;
    float b = det_b / det;
    float c = det_c / det;
    
    return QuadraticCoefficients(a, b, c);
}

// 二次函数最小二乘拟合曲率计算
float calculate_quadratic_curvature(const vector<pair<float, float>>& points)
{
    QuadraticCoefficients coeffs = calculate_quadratic_coefficients(points);
    if (!coeffs.valid) return 0.0f;
    
    // 根据数据范围调整曲率系数
    return coeffs.a * 0.1f;
}

// 线性插值工具函数
float linear_interpolation(float x1, float y1, float x2, float y2, float x)
{
    if (x2 == x1) return y1;  // 避免除零错误
    
    return y1 + (y2 - y1) * (x - x1) / (x2 - x1);
}

// 计算边界非线性度的辅助函数
float calculate_boundary_nonlinearity(const vector<int16>& bounds, int start_row, int end_row)
{
    if (start_row < 0 || end_row >= bounds.size() || start_row >= end_row)
    {
        return 0.0f;  // 无效参数
    }

    // 收集有效的边界点
    vector<Point> valid_points;
    for (int i = start_row; i <= end_row; i++)
    {
        if (bounds[i] != -1)
        {
            valid_points.push_back(Point(bounds[i], i));
        }
    }

    if (valid_points.size() < 3)
    {
        return 0.0f;  // 有效点太少，无法计算非线性度
    }

    // 方法1：计算相对于直线拟合的平均偏差
    // 使用最小二乘法拟合直线
    float sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
    int n = valid_points.size();

    for (const Point& pt : valid_points)
    {
        sum_x += pt.x;
        sum_y += pt.y;
        sum_xy += pt.x * pt.y;
        sum_x2 += pt.x * pt.x;
    }

    float denominator = n * sum_x2 - sum_x * sum_x;
    if (abs(denominator) < 0.001f)
    {
        // 直线拟合失败
        return 0.0f;  // 无法计算非线性度
    }

    // 计算直线参数 y = ax + b
    float a = (n * sum_xy - sum_x * sum_y) / denominator;
    float b = (sum_y - a * sum_x) / n;

    // 计算每个点到拟合直线的偏差
    float total_deviation = 0;
    for (const Point& pt : valid_points)
    {
        float expected_y = a * pt.x + b;
        float deviation = abs(pt.y - expected_y);
        total_deviation += deviation;
    }

    return total_deviation / n;
}
