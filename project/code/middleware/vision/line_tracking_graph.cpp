#include "line_tracking_graph.h"
#include <cmath>
#include <iostream>
#include <algorithm>
#include <queue>

// GraphNode和LineTrackingGraph成员函数实现
int LineTrackingGraph::addFirstNode(const Point& pt) {
    GraphNode node;
    node.pt = pt;
    node.type = 0; // 初始为normal类型
    nodes.push_back(node);
    start_index = 0;
    return 0;
}

int LineTrackingGraph::addNode(const Point& pt) {
    GraphNode node;
    node.pt = pt;
    node.type = 0; // 初始为normal类型
    nodes.push_back(node);
    return (int)nodes.size() - 1;
}

void LineTrackingGraph::printNodes() const {
    printf("LineTrackingGraph: %zu nodes, start_index=%d\n", nodes.size(), start_index);
    for (size_t i = 0; i < nodes.size() && i < 10; ++i) {
        const auto& node = nodes[i];
        printf("  Node %zu: (%d,%d) type=%d successors=%zu predecessors=%zu\n",
               i, node.pt.x, node.pt.y, node.type, 
               node.successors.size(), node.predecessors.size());
    }
    if (nodes.size() > 10) {
        printf("  ... and %zu more nodes\n", nodes.size() - 10);
    }
}

// 拓扑图相关算法函数实现

// 从起始节点跟踪路径直到遇到分叉点或端点
vector<Point> tracePath(const LineTrackingGraph& graph, int start_idx, int current_idx, set<int>& visited) {
    vector<Point> path;
    if (start_idx >= 0 && start_idx < (int)graph.nodes.size()) {
        path.push_back(graph.nodes[start_idx].pt);
    }
    int curr = current_idx;
    while (curr >= 0 && curr < (int)graph.nodes.size() && visited.find(curr) == visited.end()) {
        const GraphNode& node = graph.nodes[curr];
        path.push_back(node.pt);
        visited.insert(curr);
        if (node.type == 2 || node.type == 1) {
            break;
        }
        if (node.successors.size() == 1) {
            curr = node.successors[0];
        } else {
            break;
        }
    }
    return path;
}

// 提取所有路径段（分叉点到分叉点/端点的路径）
vector<vector<Point>> extractPathSegments(const LineTrackingGraph& graph) {
    vector<vector<Point>> segments;
    set<int> visited_segments;
    
    for (size_t i = 0; i < graph.nodes.size(); ++i) {
        const GraphNode& node = graph.nodes[i];
        if (node.type == 2 || (int)i == graph.start_index) {
            for (int successor_idx : node.successors) {
                if (visited_segments.find(successor_idx) == visited_segments.end()) {
                    vector<Point> path = tracePath(graph, (int)i, successor_idx, visited_segments);
                    if (path.size() >= 2) {
                        segments.push_back(path);
                    }
                }
            }
        }
    }
    return segments;
}

// 合并临近的分叉点，返回新的拓扑图
LineTrackingGraph mergeBranchPoints(const LineTrackingGraph& input_graph, double merge_distance) {
    LineTrackingGraph result = input_graph; // 复制输入图
    
    // 收集所有分叉点的索引
    vector<int> branch_indices;
    for (int i = 0; i < (int)result.nodes.size(); ++i) {
        if (result.nodes[i].type == 2) {
            branch_indices.push_back(i);
        }
    }
    
    if (branch_indices.size() <= 1) {
        return result; // 少于2个分叉点，无需合并
    }
    
    // 标记需要被合并的节点
    vector<bool> to_merge(result.nodes.size(), false);
    vector<vector<int>> merge_groups; // 每个合并组包含多个节点索引
    vector<bool> processed(branch_indices.size(), false);
    
    // 分组临近的分叉点
    for (size_t i = 0; i < branch_indices.size(); ++i) {
        if (processed[i]) continue;
        
        vector<int> group = {branch_indices[i]};
        processed[i] = true;
        
        // 查找距离当前分叉点很近的其他分叉点
        for (size_t j = i + 1; j < branch_indices.size(); ++j) {
            if (processed[j]) continue;
            
            const Point& pt1 = result.nodes[branch_indices[i]].pt;
            const Point& pt2 = result.nodes[branch_indices[j]].pt;
            double dist = sqrt((pt1.x - pt2.x) * (pt1.x - pt2.x) + (pt1.y - pt2.y) * (pt1.y - pt2.y));
            
            if (dist < merge_distance) {
                group.push_back(branch_indices[j]);
                processed[j] = true;
                
                // 标记这些节点需要被合并
                for (int idx : group) {
                    to_merge[idx] = true;
                }
            }
        }
        
        if (group.size() > 1) {
            merge_groups.push_back(group);
        }
    }
    
    if (merge_groups.empty()) {
        return result; // 没有需要合并的分叉点
    }
    
    // 创建新的拓扑图
    LineTrackingGraph merged_graph;
    map<int, int> old_to_new_index; // 旧索引到新索引的映射
    
    // 首先添加不需要合并的节点
    for (int i = 0; i < (int)result.nodes.size(); ++i) {
        if (!to_merge[i]) {
            int new_idx = merged_graph.addNode(result.nodes[i].pt);
            merged_graph.nodes[new_idx].type = result.nodes[i].type;
            old_to_new_index[i] = new_idx;
        }
    }
    
    // 为每个合并组创建一个新的合并节点
    map<int, int> merged_node_map; // 原始节点索引 -> 合并后节点索引
    for (const auto& group : merge_groups) {
        // 计算合并组的中心点
        int sum_x = 0, sum_y = 0;
        for (int idx : group) {
            sum_x += result.nodes[idx].pt.x;
            sum_y += result.nodes[idx].pt.y;
        }
        Point center(sum_x / (int)group.size(), sum_y / (int)group.size());
        int merged_idx = merged_graph.addNode(center);
        merged_graph.nodes[merged_idx].type = 2; // 分叉点
        
        // 记录映射关系
        for (int idx : group) {
            merged_node_map[idx] = merged_idx;
            old_to_new_index[idx] = merged_idx;
        }
    }
    
    // 设置起始节点
    if (result.start_index >= 0 && old_to_new_index.find(result.start_index) != old_to_new_index.end()) {
        merged_graph.start_index = old_to_new_index[result.start_index];
    } else if (!merged_graph.nodes.empty()) {
        merged_graph.start_index = 0;
    }
    
    // 重建连接关系
    set<pair<int,int>> added_connections; // 避免重复连接
    
    for (int old_idx = 0; old_idx < (int)result.nodes.size(); ++old_idx) {
        if (old_to_new_index.find(old_idx) == old_to_new_index.end()) continue;
        
        int from_new = old_to_new_index[old_idx];
        
        // 处理后继关系
        for (int old_successor : result.nodes[old_idx].successors) {
            if (old_to_new_index.find(old_successor) == old_to_new_index.end()) continue;
            
            int to_new = old_to_new_index[old_successor];
            
            // 避免自连接和重复连接
            if (from_new != to_new && added_connections.find({from_new, to_new}) == added_connections.end()) {
                merged_graph.nodes[from_new].successors.push_back(to_new);
                merged_graph.nodes[to_new].predecessors.push_back(from_new);
                added_connections.insert({from_new, to_new});
            }
        }
    }
    
    // 重新计算节点类型
    for (int i = 0; i < (int)merged_graph.nodes.size(); ++i) {
        int successor_count = (int)merged_graph.nodes[i].successors.size();
        if (successor_count == 0) {
            merged_graph.nodes[i].type = 1; // endpoint
        } else if (successor_count == 1) {
            merged_graph.nodes[i].type = 0; // normal
        } else {
            merged_graph.nodes[i].type = 2; // branch
        }
    }
    
    return merged_graph;
}

// 路径简化功能：输入TopoGraph，输出新的简化TopoGraph
LineTrackingGraph simplifyGraph(const LineTrackingGraph& input_graph, double epsilon) {
    LineTrackingGraph result;
    if (input_graph.nodes.empty() || input_graph.start_index < 0) {
        return result;
    }
    
    // 步骤1: 合并临近分叉点
    LineTrackingGraph merged_graph = mergeBranchPoints(input_graph, 15.0);
    
    // 步骤2: 提取路径段
    vector<vector<Point>> segments = extractPathSegments(merged_graph);
    if (segments.empty()) {
        return result;
    }
    
    // 步骤3: 过滤短路径段
    vector<vector<Point>> filtered_segments;
    double min_segment_length = 20.0;
    for (const auto& segment : segments) {
        if (segment.size() < 2) continue;
        double total_length = 0.0;
        for (size_t i = 1; i < segment.size(); ++i) {
            double dx = segment[i].x - segment[i-1].x;
            double dy = segment[i].y - segment[i-1].y;
            total_length += sqrt(dx*dx + dy*dy);
        }
        if (total_length >= min_segment_length) {
            filtered_segments.push_back(segment);
        }
    }
    
    if (filtered_segments.empty()) {
        return result;
    }
    
    // 步骤4: 应用多边形简化
    vector<vector<Point>> simplified_segments;
    for (const auto& segment : filtered_segments) {
        if (segment.size() >= 2) {
            vector<Point> simplified = approx_polyline(segment, 10);
            if (simplified.size() >= 2) {
                simplified_segments.push_back(simplified);
            }
        }
    }
    
    if (simplified_segments.empty()) {
        return result;
    }
    
    // 重建拓扑图
    map<pair<int,int>, int> point_to_index;
    for (const auto& segment : simplified_segments) {
        for (const auto& pt : segment) {
            pair<int,int> key = {pt.x, pt.y};
            if (point_to_index.find(key) == point_to_index.end()) {
                int idx = result.addNode(Point(pt.x, pt.y));
                point_to_index[key] = idx;
            }
        }
    }
    
    for (const auto& segment : simplified_segments) {
        for (size_t i = 0; i < segment.size() - 1; ++i) {
            pair<int,int> from_key = {segment[i].x, segment[i].y};
            pair<int,int> to_key = {segment[i+1].x, segment[i+1].y};
            int from_idx = point_to_index[from_key];
            int to_idx = point_to_index[to_key];
            if (from_idx >= 0 && from_idx < (int)result.nodes.size() &&
                to_idx >= 0 && to_idx < (int)result.nodes.size()) {
                auto& successors = result.nodes[from_idx].successors;
                if (find(successors.begin(), successors.end(), to_idx) == successors.end()) {
                    successors.push_back(to_idx);
                    result.nodes[to_idx].predecessors.push_back(from_idx);
                }
            }
        }
    }
    
    // 设置起始节点
    if (merged_graph.start_index >= 0 && merged_graph.start_index < (int)merged_graph.nodes.size()) {
        const Point& orig_start = merged_graph.nodes[merged_graph.start_index].pt;
        pair<int,int> start_key = {orig_start.x, orig_start.y};
        if (point_to_index.find(start_key) != point_to_index.end()) {
            result.start_index = point_to_index[start_key];
        } else if (!result.nodes.empty()) {
            result.start_index = 0;
        }
    } else if (!result.nodes.empty()) {
        result.start_index = 0;
    }
    
    // 重新计算节点类型
    for (int i = 0; i < (int)result.nodes.size(); ++i) {
        int successor_count = (int)result.nodes[i].successors.size();
        if (successor_count == 0) {
            result.nodes[i].type = 1; // endpoint
        } else if (successor_count == 1) {
            result.nodes[i].type = 0; // normal
        } else {
            result.nodes[i].type = 2; // branch
        }
    }
    
    return result;
}

// 多边形近似算法实现
static double pointLineDistance(const Point& pt, const Point& start, const Point& end) {
    double dx = end.x - start.x;
    double dy = end.y - start.y;
    if (dx == 0 && dy == 0) {
        dx = pt.x - start.x;
        dy = pt.y - start.y;
        return sqrt(dx * dx + dy * dy);
    }
    double t = ((pt.x - start.x) * dx + (pt.y - start.y) * dy) / (dx * dx + dy * dy);
    double proj_x = start.x + t * dx;
    double proj_y = start.y + t * dy;
    return sqrt((pt.x - proj_x) * (pt.x - proj_x) + (pt.y - proj_y) * (pt.y - proj_y));
}

static void rdp_simplify(const vector<Point>& path, double epsilon, vector<Point>& out) {
    if (path.size() < 2) {
        out = path;
        return;
    }
    double max_dist = 0.0;
    size_t index = 0;
    for (size_t i = 1; i < path.size() - 1; ++i) {
        double dist = pointLineDistance(path[i], path[0], path.back());
        if (dist > max_dist) {
            max_dist = dist;
            index = i;
        }
    }
    if (max_dist > epsilon) {
        vector<Point> rec1, rec2;
        vector<Point> sub1(path.begin(), path.begin() + index + 1);
        vector<Point> sub2(path.begin() + index, path.end());
        rdp_simplify(sub1, epsilon, rec1);
        rdp_simplify(sub2, epsilon, rec2);
        out.assign(rec1.begin(), rec1.end() - 1);
        out.insert(out.end(), rec2.begin(), rec2.end());
    } else {
        out.clear();
        out.push_back(path.front());
        out.push_back(path.back());
    }
}

vector<Point> approx_polyline(const vector<Point>& path, double epsilon) {
    vector<Point> approx;
    if (path.size() < 2) return path;
    rdp_simplify(path, epsilon, approx);
    return approx;
}
