#include "track_path.h"

void extract_path(const LineTrackingGraph& graph, TrackPath& path)
{
    path.clear();
    if (graph.empty()) return;
    int idx = graph.root();
    while (idx >= 0 && idx < graph.size())
    {
        const GraphNode& node = graph.getNode(idx);

        // 填充节点信息
        TrackPathNode path_node;
        path_node.pos = Point2f(node.data().x, node.data().y);

        // 计算所有后继的方向向量
        auto succ = node.successors();
        std::vector<Point2f> out_vecs;
        for (int sidx : succ) {
            const GraphNode& next_node = graph.getNode(sidx);
            out_vecs.emplace_back(next_node.data().x - node.data().x, next_node.data().y - node.data().y);
        }

        // 计算入射向量
        Point2f in_vec(0, 0);
        int pred_idx = node.predecessor();
        if (pred_idx >= 0 && pred_idx < graph.size()) {
            const GraphNode& pred_node = graph.getNode(pred_idx);
            in_vec = Point2f(path_node.pos.x - pred_node.data().x, path_node.pos.y - pred_node.data().y);
        } else if (!out_vecs.empty()) {
            // 起点时，取第一个出射方向
            in_vec = Point2f(0, 0);
        }

        // 调用分支元素检测
        auto result = detect_branch_element(in_vec, out_vecs);
        auto element = result.first;
        auto out_idx = result.second;
        path_node.element = element;
        if (!out_vecs.empty() && out_idx >= 0 && out_idx < out_vecs.size())
            path_node.next_vec = out_vecs[out_idx];
        else
            path_node.next_vec = Point2f(0, 0);

        path.add_node(path_node);

        // 只取第一个分支（按检测结果out_idx）
        if (succ.empty())
            break;
        else
        {
            idx = succ[out_idx];
        }
    }
}
