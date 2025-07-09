#pragma once

#include "../common/utils.h"

using namespace std;

// 节点类型枚举
enum class NodeType : int8_t
{
    DELETED = -1,
    NORMAL = 0,
    ENDPOINT = 1,
    BRANCH = 2,
    START = 3,
};

// 拓扑结构体
class GraphNode
{
public:
    GraphNode() : data_(0, 0), predecessor_(-1), type_(NodeType::NORMAL)
    {
        fill(begin(successors_), end(successors_), -1);
    }
    GraphNode(const Point& point, int16_t predecessor, NodeType node_type = NodeType::NORMAL)
        : data_(point), predecessor_(predecessor), type_(node_type)
    {
        fill(begin(successors_), end(successors_), -1);
    }

    void add_successor(int16_t successor)
    {
        // 先检查是否已经存在
        for (int i = 0; i < 3; ++i)
        {
            if (successors_[i] == successor)
            {
                return; // 已经存在，不需要重复添加
            }
        }
        
        // 添加到第一个空位
        for (int i = 0; i < 3; ++i)
        {
            if (successors_[i] == -1)
            {
                successors_[i] = successor;
                break;
            }
        }
    }

    void remove_successor(int16_t successor)
    {
        for (int i = 0; i < 3; ++i)
        {
            if (successors_[i] == successor)
            {
                // 向前移动后面的元素，保持数组紧凑
                for (int j = i; j < 2; ++j)
                {
                    successors_[j] = successors_[j + 1];
                }
                successors_[2] = -1;
                break;
            }
        }
    }

    vector<int16_t> successors() const
    {
        vector<int16_t> result;

        for (int i = 0; i < 3; ++i)
        {
            if (successors_[i] != -1)
            {
                result.push_back(successors_[i]);
            }
        }
        return result;
    }

    int16_t predecessor() const { return predecessor_; }

    void set_predecessor(int16_t pred) { predecessor_ = pred; }

    Point data() const { return data_; }

    NodeType type() const { return type_; }
    
    void set_type(NodeType new_type) { type_ = new_type; }

private:
    Point data_;
    int16_t successors_[3];   // 后继节点的索引
    int16_t predecessor_;      // 前继节点的索引
    NodeType type_;
};



class LineTrackingGraph
{
public:

    // 构造函数
    LineTrackingGraph() = default;

    // 添加节点
    int addNode(const Point& pt, int16_t predecessor = -1, NodeType type = NodeType::NORMAL)
    {
        if (node_count >= MAX_NODES)
        {
            return -1;  // 数组已满
        }

        nodes[node_count] = GraphNode(pt, predecessor, type);
        return node_count++;
    }

    // 获取节点
    const GraphNode& getNode(int index) const { return nodes[index]; }

    GraphNode& getNode(int index) { return nodes[index]; }

    // 判断图是否为空
    bool empty() const { return node_count == 0; }

    // 获取节点数量
    size_t size() const { return node_count; }

    // 清空图
    void clear()
    {
        node_count = 0;
        start_index = -1;
    }

    // 获取根节点（只有一个）
    int root() const { return start_index; }

    void set_root(int index) { start_index = index; }

    // void simplify();
    void simplify(float rdp_collinearity_threshold, int branch_merge_threshold, int min_branch_length);
    
    // 设置简化参数
    void setRdpCollinearityThreshold(int threshold) { rdp_collinearity_threshold_ = threshold; }
    void setBranchMergeThreshold(int threshold) { branch_merge_threshold_ = threshold; }
    void setMinBranchLength(int length) { min_branch_length_ = length; }

private:
    static constexpr size_t MAX_NODES = 512;
    GraphNode nodes[MAX_NODES];
    size_t node_count = 0;      // 当前节点数量
    int start_index = 0;    // 起始节点的索引

    // 简化参数
    float rdp_collinearity_threshold_ = 2;    // RDP共线性阈值
    int branch_merge_threshold_ = 3;        // 分支合并距离阈值
    int min_branch_length_ = 5;             // 最小分支长度

    void rdp_simplify();
    void merge_nearby_branches();
    void delete_short_branch();
    
    // 辅助函数
    int calculateBranchLength(int start_idx);
    int calculateBranchLengthFromTerminal(int terminal_idx);
    void deletePathFromTerminal(int terminal_idx);
    void simplifySegment(vector<int> segment);
    std::vector<int> collectSegment(int start_idx, int first_idx);
};