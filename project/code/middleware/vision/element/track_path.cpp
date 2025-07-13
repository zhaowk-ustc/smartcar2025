#include "track_path.h"

void extract_path(const LineTrackingGraph& graph, TrackPath& path)
{
    path.clear();
    path.total_length = 0.0f;
    if (graph.empty()) return;
    int idx = graph.root();
    float total_length = 0.0f;
    while (idx >= 0 && idx < graph.size())
    {
        const GraphNode& node = graph.getNode(idx);

        // 填充节点信息
        TrackPathNode path_node;
        path_node.pos = Point2f(node.data().x(), node.data().y());

        // 计算所有后继的方向向量
        auto succ = node.successors();
        std::vector<Point2f> out_dirs;
        std::vector<float> out_lengths;
        for (int sidx : succ)
        {
            const GraphNode& next_node = graph.getNode(sidx);
            float out_x = next_node.data().x() - node.data().x();
            float out_y = next_node.data().y() - node.data().y();
            // 归一化方向向量
            float length = std::sqrtf(out_x * out_x + out_y * out_y);
            if (length > 0)
            {
                out_x /= length;
                out_y /= length;
            }
            out_dirs.emplace_back(out_x, out_y);
            out_lengths.push_back(length);
        }

        // 计算入射向量
        Point2f in_dir(0, 0);
        int pred_idx = node.predecessor();
        if (pred_idx >= 0 && pred_idx < graph.size())
        {
            const GraphNode& pred_node = graph.getNode(pred_idx);
            float in_x = path_node.pos.x() - pred_node.data().x();
            float in_y = path_node.pos.y() - pred_node.data().y();
            // 归一化入射向量
            float in_length = std::sqrtf(in_x * in_x + in_y * in_y);
            if (in_length > 0)
            {
                in_x /= in_length;
                in_y /= in_length;
            }
            in_dir = Point2f(in_x, in_y);
        }
        else if (!out_dirs.empty())
        {
            // 起点时，取第一个出射方向
            in_dir = Point2f(0, 0);
        }

        // 调用分支元素检测
        auto result = detect_branch_element(in_dir, out_dirs);
        auto element = result.first;
        auto out_idx = result.second;
        path_node.element = element;
        if (!out_dirs.empty() && out_idx >= 0 && out_idx < out_dirs.size())
        {
            path_node.next_dir = out_dirs[out_idx];
            path_node.next_length = out_lengths[out_idx];
        }
        else
        {
            path_node.next_dir = Point2f(0, 0);
            path_node.next_length = 0;
        }


        auto newnode = path.add_node(path_node);
        if (newnode == -1)
        {
            // 添加节点失败，图已满
            path.total_length = 0;
            return;
        }

        // 累加路径长度
        total_length += path_node.next_length;

        // 只取第一个分支（按检测结果out_idx）
        if (succ.empty())
            break;
        else
        {
            idx = succ[out_idx];
        }
    }
    path.total_length = total_length;
}
