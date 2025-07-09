#pragma once
#include "../track/line_tracking_graph.h"
#include "element.h"
#include <vector>

// 单个轨迹节点
struct TrackPathNode
{
    Point2f pos;                   // 坐标
    ElementType element;       // 元素类型
    Point2f next_vec;              // 指向下一个节点的向量
};

// 轨迹序列（静态存储）
class TrackPath
{
public:

    TrackPath() : node_count(0) {}
        static constexpr size_t MAX_NODES = 64;
    TrackPathNode nodes[MAX_NODES];
    void clear() { node_count = 0; }
    size_t size() const { return node_count; }
    const TrackPathNode& operator[](size_t i) const { return nodes[i]; }
    TrackPathNode& operator[](size_t i) { return nodes[i]; }
    size_t add_node(const TrackPathNode& node)
    {
        if (node_count >= MAX_NODES) return -1;
        nodes[node_count++] = node;
        return node_count - 1;
    }

private:

    size_t node_count;
};

// 从拓扑图提取主路径
void extract_path(const LineTrackingGraph& graph, TrackPath& path);
