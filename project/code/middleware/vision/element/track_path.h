#pragma once
#include "../track/line_tracking_graph.h"
#include "element.h"
#include <vector>
#include <cmath>

// 单个轨迹节点
struct TrackPathNode
{
    Point2f pos;                   // 坐标
    ElementType element;       // 元素类型
    Point2f next_dir;              // 指向下一个节点的方向
    float next_length;             // 指向下一个节点的长度
};

// 轨迹序列（静态存储）
class TrackPath
{
public:

    TrackPath() : node_count(0) {}
    static constexpr size_t MAX_NODES = 16;
    TrackPathNode nodes[MAX_NODES];
    void clear() { node_count = 0; }
    size_t size() const { return node_count; }
    float length() const { return total_length; }
    const TrackPathNode& operator[](size_t i) const { return nodes[i]; }
    TrackPathNode& operator[](size_t i) { return nodes[i]; }
    size_t add_node(const TrackPathNode& node)
    {
        if (node_count >= MAX_NODES) return -1;
        nodes[node_count++] = node;
        return node_count - 1;
    }
    float total_length;

    const Point2f& start() const
    {
        if (node_count > 0)
            return nodes[0].pos;
        static Point2f empty_point(0,0);
        return empty_point; // 返回一个空点
    }
    const Point2f& end() const
    {
        if (node_count > 0)
            return nodes[node_count - 1].pos;
        static Point2f empty_point(0,0);
        return empty_point; // 返回一个空点
    }
private:

    size_t node_count;
};

// 从拓扑图提取主路径
void extract_path(const LineTrackingGraph& graph, TrackPath& path);
