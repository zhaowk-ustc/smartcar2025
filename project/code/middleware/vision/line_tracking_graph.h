#ifndef LINE_TRACKING_GRAPH_H
#define LINE_TRACKING_GRAPH_H

#include "image.h"
#include <vector>
#include <set>
#include <map>
#include <cstdio>
using namespace std;

// 拓扑结构体
struct GraphNode
{
    Point pt;
    vector<int> successors; // 后继节点的索引，而不是指针
    vector<int> predecessors; // 前继节点的索引
    int type; // 0: normal, 1: endpoint, 2: branch, 3: ring_junction
};

class LineTrackingGraph
{
public:
    vector<GraphNode> nodes;
    int start_index = -1; // 起始节点的索引

    int addFirstNode(const Point& pt);
    int addNode(const Point& pt);
    void printNodes() const;
};

// 拓扑图相关算法函数声明
vector<Point> tracePath(const LineTrackingGraph& graph, int start_idx, int current_idx, set<int>& visited);
vector<vector<Point>> extractPathSegments(const LineTrackingGraph& graph);
LineTrackingGraph mergeBranchPoints(const LineTrackingGraph& input_graph, double merge_distance = 8.0);
LineTrackingGraph simplifyGraph(const LineTrackingGraph& input_graph, double epsilon = 2.0);
vector<Point> approx_polyline(const vector<Point>& path, double epsilon = 2.0);












#endif // LINE_TRACKING_GRAPH_H