#include "utils.h"
#include "point.h"
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <queue>

// 线性插值工具函数
float linear_interpolation(float x1, float y1, float x2, float y2, float x)
{
    if (x2 == x1) return y1;  // 避免除零错误

    return y1 + (y2 - y1) * (x - x1) / (x2 - x1);
}

// 从指定点开始向指定方向搜索白点的工具函数
Point find_white_point(const uint8_t* image, uint16_t width, uint16_t height,
    const vector<Point>& start_points, SearchDirection direction,
    int max_distance)
{
    if (start_points.empty())
    {
        return NULL_POINT;  // 空点列
    }

    // 设置搜索方向的偏移量
    int dx = 0, dy = 0;
    switch (direction)
    {
        case SearchDirection::UP:
            dy = -1; break;
        case SearchDirection::DOWN:
            dy = 1; break;
        case SearchDirection::LEFT:
            dx = -1; break;
        case SearchDirection::RIGHT:
            dx = 1; break;
    }

    // 设置最大搜索距离
    int max_search_distance = max_distance;
    if (max_search_distance <= 0)
    {
        // 设置默认最大搜索距离为整个图像的对角线长度
        max_search_distance = width + height;
    }

    // 初始化每个点的当前搜索位置
    vector<Point> current_positions = start_points;
    vector<bool> valid_positions(start_points.size(), true);

    // 逐步搜索：每次对所有有效点都搜索一步
    for (int distance = 0; distance < max_search_distance; distance++)
    {
        // 遍历所有点，每个点搜索一步
        for (size_t i = 0; i < current_positions.size(); i++)
        {
            if (!valid_positions[i])
            {
                continue;  // 跳过已经无效的点
            }

            Point& current_pos = current_positions[i];

            // 检查当前点是否在图像范围内
            if (current_pos.x < 0 || current_pos.x >= width ||
                current_pos.y < 0 || current_pos.y >= height)
            {
                valid_positions[i] = false;  // 标记为无效
                continue;
            }

            // 检查当前点是否为白点（二值化图像：255为白点，0为黑点）
            if (image[current_pos.y * width + current_pos.x] == 255)
            {
                return Point(current_pos.x, current_pos.y);  // 找到白点，立即返回
            }

            // 移动到下一个位置
            current_pos.x += dx;
            current_pos.y += dy;
        }

        // 检查是否还有有效的搜索点
        bool has_valid_point = false;
        for (bool valid : valid_positions)
        {
            if (valid)
            {
                has_valid_point = true;
                break;
            }
        }

        if (!has_valid_point)
        {
            break;  // 所有点都已经无效，提前退出
        }
    }

    return NULL_POINT;  // 遍历完所有点都没有找到白点
}

