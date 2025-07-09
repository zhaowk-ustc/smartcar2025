#pragma once

#include <vector>
#include <utility>
#include <cmath>
#include <cstdint>
#include <vector>

using namespace std;
#include "../common/point.h"

// 搜索方向枚举
enum class SearchDirection
{
    UP,       // 向上搜索
    DOWN,     // 向下搜索
    LEFT,     // 向左搜索
    RIGHT     // 向右搜索
};

// 线性插值工具函数
float linear_interpolation(float x1, float y1, float x2, float y2, float x);

// 从点列向指定方向搜索白点的工具函数
// 参数：image - 图像数据
//      width - 图像宽度
//      height - 图像高度
//      start_points - 起始点列表
//      direction - 搜索方向
//      max_distance - 最大搜索距离，默认为0表示搜索到边界
// 返回：找到的第一个白点坐标，如果没有找到则返回(-1, -1)
// 使用示例：
//   vector<Point> boundary_points = {Point(20, 30), Point(25, 35), Point(30, 40)};
//   Point white_point = find_white_point(image, 80, 60, boundary_points, SearchDirection::UP, 10);
Point find_white_point(const uint8_t* image, uint16_t width, uint16_t height,
    const vector<Point>& start_points, SearchDirection direction,
    int max_distance = 0);

// 判断点是否在图像范围内
inline bool inBounds(int x, int y, int w, int h)
{
    return x >= 0 && x < w && y >= 0 && y < h;
}

// 计算两点欧氏距离
inline float euclideanDist(const Point& a, const Point& b)
{
    return std::sqrtf(static_cast<float>(a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

// 计算两点曼哈顿距离（直角距离）
inline int manhattanDist(const Point& a, const Point& b)
{
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

// 计算点到直线的垂直距离（欧氏距离）
inline float pointToLineDistance(const Point& point, const Point& lineStart, const Point& lineEnd)
{
    // 如果直线起点和终点相同，返回点到点的距离
    if (lineStart.x == lineEnd.x && lineStart.y == lineEnd.y)
    {
        return euclideanDist(point, lineStart);
    }
    
    // 使用点到直线距离公式：|ax + by + c| / sqrt(a² + b²)
    // 直线方程：(y2-y1)x - (x2-x1)y + (x2*y1 - x1*y2) = 0
    float a = lineEnd.y - lineStart.y;
    float b = lineStart.x - lineEnd.x;
    float c = lineEnd.x * lineStart.y - lineStart.x * lineEnd.y;
    
    float distance = std::abs(a * point.x + b * point.y + c) / std::sqrtf(a * a + b * b);
    return distance;
}
