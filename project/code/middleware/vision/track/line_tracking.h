#pragma once
#include "zf_common_headfile.h"
#include "line_tracking_graph.h"
#include "../common/point.h"
#include "../common/utils.h"
#include <vector>
using namespace std;

void create_line_tracking_graph(
    LineTrackingGraph& graph,
    const uint8_t* image,
    int image_w,
    int image_h,
    int search_seed_radius = 5,
    int dist_threshold = 5,
    float rdp_threshold = 5,
    int branch_merge_threshold = 8,
    int min_branch_length = 5
);

vector<pair<Point, Point>> search_seed_points(
    const uint8_t* image,
    int image_w,
    int image_h,
    Point seed_point,
    const uint8_t ttl,
    uint8_t* ttl_map
);


void get_depth_map(
    const uint8_t* image,
    uint8_t* depth_map,
    int image_w,
    int image_h,
    const std::vector<Point>& seed_points
);

// 从深度图和seed-link点对构建初始拓扑图
void build_graph(
    LineTrackingGraph& graph,
    const uint8_t* image,
    int image_w,
    int image_h,
    const std::vector<pair<Point, Point>>& seed_link_points,
    int16_t* point_to_node_map,
    uint8_t dist_thresh
);