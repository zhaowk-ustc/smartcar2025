#include <vector>
#include <cstdint>
#include <queue>
#include <algorithm>
#include <cmath>
#include "fill.h"
#include "../common/utils.h"



// 通用的连通域填充函数
std::vector<Point> flood_fill(
    const uint8_t* image,
    uint8_t* visited,
    int image_w,
    int image_h,
    Point start,
    uint8_t fill_value
)
{
    vector<Point> region;
    if (!inBounds(start.x(), start.y(), image_w, image_h) ||
        image[start.y() * image_w + start.x()] == 0 ||
        visited[start.y() * image_w + start.x()])
    {
        return region;
    }

    queue<Point> flood_queue;
    flood_queue.push(start);
    visited[start.y() * image_w + start.x()] = fill_value;
    while (!flood_queue.empty())
    {
        Point current = flood_queue.front();
        flood_queue.pop();
        region.push_back(current);

        // 8邻域扩展
        for (int d = 0; d < 8; ++d)
        {
            int nx = current.x() + dx[d];
            int ny = current.y() + dy[d];

            if (inBounds(nx, ny, image_w, image_h))
            {
                int nidx = ny * image_w + nx;
                if (image[nidx] > 0 && !visited[nidx])
                {
                    visited[nidx] = fill_value;
                    flood_queue.push(Point(nx, ny));
                }
            }
        }
    }
    return region;
}


// 从种子点开始对连通域做层次遍历并染色，染色值为层次+1
std::vector<Point> flood_fill_with_depth(
    const uint8_t* image,
    uint8_t* depth_map,
    int image_w,
    int image_h,
    Point start
)
{
    vector<Point> region;
    if (!inBounds(start.x(), start.y(), image_w, image_h) ||
        image[start.y() * image_w + start.x()] == 0 ||
        depth_map[start.y() * image_w + start.x()] > 0)
    {
        return region;
    }

    queue<Point> bfs_queue;
    bfs_queue.push(start);
    depth_map[start.y() * image_w + start.x()] = 1; // 起始点深度为1

    while (!bfs_queue.empty())
    {
        Point current = bfs_queue.front();
        bfs_queue.pop();
        region.push_back(current);

        int current_depth = depth_map[current.y() * image_w + current.x()];

        // 8邻域扩展
        for (int d = 0; d < 8; ++d)
        {
            int nx = current.x() + dx[d];
            int ny = current.y() + dy[d];

            if (inBounds(nx, ny, image_w, image_h))
            {
                int nidx = ny * image_w + nx;
                if (image[nidx] > 0 && depth_map[nidx] == 0)
                {
                    depth_map[nidx] = current_depth + 1; // 层次+1
                    bfs_queue.push(Point(nx, ny));
                }
            }
        }
    }
    return region;
}

