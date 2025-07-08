#include <vector>
#include <iostream>
#include <map>
#include <set>
#include <queue>
#include <algorithm>

#include "line_tracking_graph.h"


void LineTrackingGraph::simplify(float rdp_collinearity_threshold, int branch_merge_threshold, int min_branch_length)
{
    // 设置参数
    rdp_collinearity_threshold_ = rdp_collinearity_threshold;
    branch_merge_threshold_ = branch_merge_threshold;
    min_branch_length_ = min_branch_length;

    // 执行简化 - 优化的执行顺序：
    // 1. 先进行RDP简化，减少冗余节点，为后续操作提供更清晰的图结构
    // 2. 然后删除短分支，避免在分支合并后产生新的短分支
    // 3. 最后合并相近分支，因为前面的操作可能会产生新的相近分支点
    rdp_simplify();
    delete_short_branch();
    merge_nearby_branches();
}

void LineTrackingGraph::rdp_simplify()
{
    // RDP算法简化路径：按段简化，每个分支点之间的段独立处理

    // 首先找到所有分支点和端点
    std::vector<int> branch_and_end_points;
    for (int i = 0; i < node_count; ++i)
    {
        auto& node = nodes[i];
        if (node.type() == NodeType::DELETED) continue; // 跳过已删除的节点

        auto successors = node.successors();

        // 分支点（多个后继）、端点（无后继）、或根节点
        if (successors.size() != 1 || i == start_index)
        {
            branch_and_end_points.push_back(i);
        }
    }

    // 对每个分支点，收集并简化它的每个后继段
    for (int branch_idx : branch_and_end_points)
    {
        auto& branch_node = nodes[branch_idx];
        if (branch_node.type() == NodeType::DELETED) continue; // 跳过已删除的节点

        auto successors = branch_node.successors();

        // 对每个后继段进行简化
        for (int successor_idx : successors)
        {
            if (successor_idx >= 0 && successor_idx < node_count)
            {
                // 收集从当前分支点到下一个分支点/端点的完整段
                std::vector<int> segment = collectSegment(branch_idx, successor_idx);

                // if (segment.size() >= 3) // 至少需要3个点才能进行RDP简化
                // {
                simplifySegment(segment);
                // }
            }
        }
    }

    // printf("RDP simplification completed\n");
}

// 收集从起始点到下一个分支点/端点的完整段
std::vector<int> LineTrackingGraph::collectSegment(int start_idx, int first_idx)
{
    std::vector<int> segment;
    segment.push_back(start_idx); // 包含起始分支点

    int current_idx = first_idx;

    // 沿着单一路径收集节点，直到遇到分支点或端点
    while (current_idx >= 0 && current_idx < node_count)
    {
        auto& node = nodes[current_idx];
        if (node.type() == NodeType::DELETED) break; // 跳过已删除的节点

        segment.push_back(current_idx);
        auto successors = node.successors();

        if (successors.size() == 0)
        {
            // 到达端点，结束收集
            break;
        }
        else if (successors.size() == 1)
        {
            // 继续沿着单一路径
            current_idx = successors[0];
        }
        else
        {
            // 遇到分支点，结束收集（分支点已经包含在段中）
            break;
        }
    }

    return segment;
}

// 简化从start_idx到下一个分支点/端点之间的段
void LineTrackingGraph::simplifySegment(vector<int> segment)
{
    // 对这个段应用RDP算法
    Point start_point = nodes[segment.front()].data();
    Point end_point = nodes[segment.back()].data();

    float max_distance = -1.0f;
    int max_idx = -1;
    for (int i = 1; i < segment.size() - 1; ++i)
    {
        Point p = nodes[segment[i]].data();

        // 计算点到直线的垂直距离（欧氏距离）
        float distance = pointToLineDistance(p, start_point, end_point);
        if (distance > max_distance)
        {
            max_distance = distance;
            max_idx = i;
        }
    }
    if (max_idx == -1)
    {
        // 如果只有两个点
        return;
    }
    if (max_distance <= rdp_collinearity_threshold_)
    {
        // 如果最大偏差在阈值内，则直接连接起始点和结束点
        for (auto i = 1; i < segment.size() - 1; ++i)
        {
            nodes[segment[i]].set_predecessor(-2);
            nodes[segment[i]].set_type(NodeType::DELETED);
        }
        nodes[segment.front()].remove_successor(segment[1]);
        nodes[segment.front()].add_successor(segment.back());
        nodes[segment.back()].set_predecessor(segment.front());
        return;
    }
    else
    {
        vector<int> newseg1, newseg2;
        for (auto i = 0;i <= max_idx;++i)
        {
            newseg1.push_back(segment[i]);
        }
        for (auto i = max_idx; i < segment.size(); ++i)
        {
            newseg2.push_back(segment[i]);
        }
        // 递归简化新段
        simplifySegment(newseg1);
        simplifySegment(newseg2);
    }
}

void LineTrackingGraph::merge_nearby_branches()
{
    // 合并距离很近的分支节点 - 只合并同一段上的两个分支点
    // printf("Merging nearby branches with threshold %d...\n", branch_merge_threshold_);

    // 先统计当前的分支节点情况
    int branch_count = 0;
    // printf("Current branch nodes: ");
    for (int i = 0; i < node_count; ++i)
    {
        auto& node = nodes[i];
        if (node.predecessor() == -2) continue; // 跳过已删除的节点

        auto successors = node.successors();
        if (successors.size() > 1)
        {
            // printf("%d(succ:%d) ", i, (int)successors.size());
            branch_count++;
        }
    }

    // 找到所有分支节点
    std::vector<int> branch_nodes;
    for (int i = 0; i < node_count; ++i)
    {
        auto& node = nodes[i];
        if (node.predecessor() == -2) continue; // 跳过已删除的节点

        auto successors = node.successors();
        if (successors.size() > 1)
        {
            branch_nodes.push_back(i);
        }
    }

    // 对每个分支节点，收集从它开始的所有段
    for (int branch_idx : branch_nodes)
    {
        if (nodes[branch_idx].predecessor() == -2) continue; // 跳过已删除的节点

        auto successors = nodes[branch_idx].successors();

        for (int successor_idx : successors)
        {
            if (successor_idx < 0 || successor_idx >= node_count) continue;
            if (nodes[successor_idx].predecessor() == -2) continue; // 跳过已删除的节点

            // 收集从branch_idx到successor_idx的段
            std::vector<int> segment = collectSegment(branch_idx, successor_idx);

            if (segment.size() >= 2)
            {
                int start_idx = segment[0];        // 段的起始点（分支点）
                int end_idx = segment.back();      // 段的结束点

                // 检查段的两端是否都是分支点
                auto start_successors = nodes[start_idx].successors();
                auto end_successors = nodes[end_idx].successors();

                if (start_successors.size() > 1 && end_successors.size() > 1)
                {
                    // 两端都是分支点，检查距离
                    int distance = manhattanDist(nodes[start_idx].data(), nodes[end_idx].data());

                    if (distance <= branch_merge_threshold_)
                    {
                        // 首先从end_idx的前驱中移除start_idx（如果存在）
                        nodes[end_idx].set_predecessor(nodes[start_idx].predecessor());

                        // 将start_idx的所有后继转移到end_idx
                        auto start_successors = nodes[start_idx].successors();
                        for (int successor_idx : start_successors)
                        {
                            if (successor_idx != end_idx) // 避免自环
                            {
                                // 更新后继节点的前驱指向end_idx
                                nodes[successor_idx].set_predecessor(end_idx);
                                // 添加到end_idx的后继列表
                                nodes[end_idx].add_successor(successor_idx);
                            }
                        }

                        // 如果start_idx有前驱，更新前驱的后继指向end_idx
                        int start_pred = nodes[start_idx].predecessor();
                        if (start_pred >= 0 && start_pred < node_count)
                        {
                            nodes[start_pred].remove_successor(start_idx);
                            nodes[start_pred].add_successor(end_idx);
                        }

                        // 删除段中间的所有节点（包括start_idx）
                        for (int i = 0; i < segment.size() - 1; ++i)
                        {
                            // printf("Deleting intermediate node %d\n", segment[i]);
                            nodes[segment[i]].set_predecessor(-2);
                            nodes[segment[i]].set_type(NodeType::DELETED);
                        }
                    }
                }
            }
        }
    }
}

void LineTrackingGraph::delete_short_branch()
{
    // 删除过短的分支（从终点开始回溯检查）
    // printf("Deleting short branches with min length %d...\n", min_branch_length_);

    // 找到所有终点节点并删除短分支，只需要遍历一次
    for (int i = 0; i < node_count; ++i)
    {
        auto& node = nodes[i];
        if (node.predecessor() == -2) continue; // 跳过已删除的节点

        auto successors = node.successors();
        if (successors.size() == 0) // 终点节点（无后继）
        {
            // 从这个终点开始回溯计算分支长度
            int branch_length = calculateBranchLengthFromTerminal(i);

            // 如果分支太短，从终点开始删除路径
            if (branch_length < min_branch_length_)
            {
                // 直接从终点开始删除路径，直到遇到分支点
                deletePathFromTerminal(i);
            }
        }
    }
}

// 辅助函数：计算从给定节点开始的分支长度（曼哈顿距离之和）
int LineTrackingGraph::calculateBranchLength(int start_idx)
{
    if (start_idx < 0 || start_idx >= node_count) return 0;

    int total_manhattan_length = 0;
    int current_idx = start_idx;
    int prev_idx = -1;

    // 找到起始节点的前驱，作为第一个参考点
    auto& start_node = nodes[start_idx];
    if (start_node.predecessor() >= 0 && start_node.predecessor() < node_count)
    {
        prev_idx = start_node.predecessor();
    }

    while (current_idx >= 0 && current_idx < node_count)
    {
        auto& node = nodes[current_idx];
        if (node.predecessor() == -2) break; // 已删除的节点

        // 如果有前一个节点，计算当前段的曼哈顿距离
        if (prev_idx >= 0 && prev_idx < node_count && nodes[prev_idx].predecessor() != -2)
        {
            Point prev_point = nodes[prev_idx].data();
            Point current_point = nodes[current_idx].data();
            total_manhattan_length += manhattanDist(prev_point, current_point);
        }

        auto successors = node.successors();

        if (successors.size() == 0)
        {
            // 到达终点
            break;
        }
        else if (successors.size() == 1)
        {
            // 继续沿着单一路径
            prev_idx = current_idx;
            current_idx = successors[0];
        }
        else
        {
            // 遇到另一个分支点，停止计算
            break;
        }
    }

    return total_manhattan_length;
}

// 辅助函数：从终点开始回溯计算分支长度（曼哈顿距离之和）
int LineTrackingGraph::calculateBranchLengthFromTerminal(int terminal_idx)
{
    if (terminal_idx < 0 || terminal_idx >= node_count) return 0;

    int total_manhattan_length = 0;
    int current_idx = terminal_idx;

    while (current_idx >= 0 && current_idx < node_count)
    {
        auto& node = nodes[current_idx];
        if (node.predecessor() == -2) break; // 已删除的节点

        int pred_idx = node.predecessor();
        if (pred_idx >= 0 && pred_idx < node_count && nodes[pred_idx].predecessor() != -2)
        {
            // 计算当前节点到前驱节点的曼哈顿距离
            Point current_point = nodes[current_idx].data();
            Point pred_point = nodes[pred_idx].data();
            total_manhattan_length += manhattanDist(current_point, pred_point);

            // 检查前驱节点是否是分支点
            auto pred_successors = nodes[pred_idx].successors();
            if (pred_successors.size() > 1)
            {
                // 到达分支点，停止计算
                break;
            }

            // 继续向前回溯
            current_idx = pred_idx;
        }
        else
        {
            // 没有有效前驱，到达根节点或断链
            break;
        }
    }

    return total_manhattan_length;
}

// 辅助函数：直接从终点开始删除路径，直到遇到分支点
void LineTrackingGraph::deletePathFromTerminal(int terminal_idx)
{
    if (terminal_idx < 0 || terminal_idx >= node_count) return;

    int current_idx = terminal_idx;

    while (current_idx >= 0 && current_idx < node_count)
    {
        auto& node = nodes[current_idx];
        if (node.predecessor() == -2) break; // 已删除的节点

        int pred_idx = node.predecessor();

        // 标记当前节点为已删除
        node.set_predecessor(-2);
        node.set_type(NodeType::DELETED);

        // 如果有前驱节点
        if (pred_idx >= 0 && pred_idx < node_count && nodes[pred_idx].predecessor() != -2)
        {
            // 从前驱节点的后继列表中移除当前节点
            nodes[pred_idx].remove_successor(current_idx);

            // 检查前驱节点是否是分支点
            auto pred_successors = nodes[pred_idx].successors();
            if (pred_successors.size() > 1)
            {
                // 到达分支点，停止删除
                break;
            }

            // 继续向前删除
            current_idx = pred_idx;
        }
        else
        {
            // 没有有效前驱，停止删除
            break;
        }
    }
}