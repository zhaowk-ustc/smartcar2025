#include <cmath>
#include <cstring>
#include <vector>
#include <queue>
#include <map>
using namespace std;

#include "line_tracking_graph.h"
#include "line_tracking.h"
#include "fill.h"


void create_line_tracking_graph(
    LineTrackingGraph& graph,
    const uint8_t* image,
    int image_w,
    int image_h,
    int search_seed_radius,
    int min_region_size,
    int dist_threshold,
    float rdp_threshold,
    int branch_merge_threshold,
    int min_branch_length,
    uint8_t* ttl_map,
    uint8_t* depth_map,
    uint8_t* visited,
    int16_t* point_to_node_map
)
{
    // 先清理图的状态
    graph.clear();

    vector<Point> start_points;
    for (auto x = image_w / 4; x < image_w * 3 / 4; x++)
    {
        start_points.push_back(Point(x, image_h - 1));
    }
    Point seed_point = find_white_point(image, image_w, image_h,
        start_points, SearchDirection::UP, image_h / 4);

    if (seed_point == NULL_POINT)
    {
        // 处理未找到白点的情况
        return;
    }

    // Step 1: 主连通域膨胀mask，标记邻域
    auto seed_link_points = search_seed_points(image, image_w, image_h,
        seed_point,
        search_seed_radius,
        min_region_size,
        ttl_map,
        visited);

    // 提取seed_points和link_points
    std::vector<Point> seed_points;
    std::vector<Point> link_points;
    for (const auto& pair : seed_link_points)
    {
        seed_points.push_back(pair.first);
        link_points.push_back(pair.second);
    }

    // Step 2: 对每个白点进行染色
    std::vector<Point> final_link_points;
    get_depth_map(image, depth_map, image_w, image_h, seed_points);

    // Step 3: 生成初始拓扑图
    build_graph(graph, depth_map, image_w, image_h,
        seed_link_points, point_to_node_map, dist_threshold);

    // Step 4: 图简化

    // 设置简化参数
    // int rdp_threshold = 30;           // RDP共线性阈值
    // int branch_merge_threshold = 40;  // 分支合并距离阈值
    // int min_branch_length = 5;       // 最小分支长度

    graph.simplify(rdp_threshold, branch_merge_threshold, min_branch_length);

    // 临时的遍历更新节点类型
    for (size_t i = 0; i < graph.size(); ++i)
    {
        GraphNode& node = graph.getNode(i);

        // 跳过已删除的节点和START节点
        if (node.type() == NodeType::DELETED || node.type() == NodeType::START)
        {
            continue;
        }

        if (node.predecessor() == -2)
        {
            node.set_type(NodeType::DELETED);
            continue;
        }

        // 统计后继节点数量（排除已删除的节点）
        vector<int16_t> successors = node.successors();
        int valid_successor_count = 0;
        for (int succ_idx : successors)
        {
            if (succ_idx >= 0 && succ_idx < graph.size() &&
                graph.getNode(succ_idx).type() != NodeType::DELETED)
            {
                valid_successor_count++;
            }
        }

        // 检查前驱节点是否有效
        bool has_valid_predecessor = false;
        if (node.predecessor() >= 0 && node.predecessor() < graph.size())
        {
            has_valid_predecessor = (graph.getNode(node.predecessor()).type() != NodeType::DELETED);
        }

        // 根据连接情况更新节点类型
        if (valid_successor_count == 0 && !has_valid_predecessor)
        {
            // 孤立节点，标记为端点
            node.set_type(NodeType::ENDPOINT);
        }
        else if (valid_successor_count == 0 || !has_valid_predecessor)
        {
            // 叶子节点（只有前驱或只有后继）
            node.set_type(NodeType::ENDPOINT);
        }
        else if (valid_successor_count >= 2)
        {
            // 分支节点（有多个后继）
            node.set_type(NodeType::BRANCH);
        }
        else
        {
            // 普通节点（有一个前驱和一个后继）
            node.set_type(NodeType::NORMAL);
        }
    }

    // 清理局部变量，确保内存得到释放
    // STL容器会自动管理内存，但我们可以明确清理
    seed_points.clear();
    link_points.clear();
    seed_link_points.clear();
}


// 返回seed和对应link点的pair数组
vector<pair<Point, Point>> search_seed_points(
    const uint8_t* image,
    int image_w,
    int image_h,
    Point seed_point,
    const uint8_t ttl,
    const uint16_t min_region_size,
    uint8_t* ttl_map,
    uint8_t* visited
)
{
    memset(ttl_map, 0, image_w * image_h * sizeof(uint8_t));
    vector<pair<Point, Point>> seed_link_points;

    // 第一个seed没有link点
    seed_link_points.push_back({ seed_point, NULL_POINT });

    // BFS队列，存储要扩展的点
    queue<Point> bfs_queue;

    // 1. 对初始seed所在连通域进行flood fill并标记ttl
    // 初始连通域flood fill
    auto initial_region = flood_fill(image, ttl_map, image_w, image_h, seed_point, ttl);

    // 将初始连通域的所有点加入BFS队列
    for (const auto& pt : initial_region)
    {
        bfs_queue.push(pt);
    }

    // 2. BFS扩展，寻找其他连通域
    while (!bfs_queue.empty())
    {
        Point current = bfs_queue.front();
        bfs_queue.pop();

        int current_ttl = ttl_map[current.y() * image_w + current.x()];
        if (current_ttl <= 0) continue;

        // 8邻域扩展
        for (int d = 0; d < 8; ++d)
        {
            int nx = current.x() + dx[d];
            int ny = current.y() + dy[d];

            if (!inBounds(nx, ny, image_w, image_h)) continue;

            int nidx = ny * image_w + nx;

            if (image[nidx] != 0 && ttl_map[nidx] == 0)
            {
                // 遇到未遍历过的白色像素
                Point new_seed(nx, ny);

                memset(visited, 0, image_w * image_h);

                auto new_region = flood_fill(image, visited, image_w, image_h, new_seed, 1);

                if (new_region.size() < min_region_size)
                {
                    // 如果新连通域小于最小区域大小，忽略这个区域的点
                    flood_fill(image, ttl_map, image_w, image_h, new_seed, 1);
                    continue;
                }

                vector<Point> current_layer;
                vector<Point> next_layer;
                current_layer.push_back(new_seed);
                bool found = false;
                vector<Point> manhattan_nearest;
                // 从当前点开始，沿着ttl梯度上升方向回溯
                memset(visited, 0, image_w * image_h);
                while (!found)
                {
                    // 检查当前点周围8邻域，找到ttl值更高的点
                    for (auto pt : current_layer)
                    {
                        for (int dd = 0; dd < 8; ++dd)
                        {
                            int tx = pt.x() + dx[dd];
                            int ty = pt.y() + dy[dd];

                            if (inBounds(tx, ty, image_w, image_h)
                                && visited[ty * image_w + tx] == 0
                                )
                            {
                                if (ttl_map[ty * image_w + tx] == ttl)
                                {
                                    found = true;
                                    manhattan_nearest.push_back(Point(tx, ty));
                                }
                                visited[ty * image_w + tx] = 1;
                                next_layer.push_back(Point(tx, ty));
                            }
                        }
                    }
                    current_layer = next_layer;
                    next_layer.clear();
                }
                // 提取出当前层欧氏距离最近的点
                Point euclidean_nearest;
                float nearest_dist = float(image_w + image_h); // 初始化为最大距离
                for (auto pt : manhattan_nearest)
                {
                    float dist = euclideanDist(new_seed, pt);
                    if (dist < nearest_dist)
                    {
                        nearest_dist = dist;
                        euclidean_nearest = pt;
                    }
                }

                // 将新的seed-link对加入结果
                seed_link_points.push_back({ new_seed, euclidean_nearest });

                // 对新连通域进行flood fill
                for (auto pt : new_region)
                {
                    ttl_map[pt.y() * image_w + pt.x()] = ttl; // 更新ttl值
                }

                // 将新连通域的点加入BFS队列
                for (const auto& pt : new_region)
                {
                    bfs_queue.push(pt);
                }

            }
            else
            {
                // 遇到黑色像素，更新ttl值并继续扩展
                int new_ttl = current_ttl - 1;
                if (new_ttl > 0 && new_ttl > ttl_map[nidx])
                {
                    ttl_map[nidx] = new_ttl;
                    bfs_queue.push(Point(nx, ny));
                }
            }
        }
    }

    return seed_link_points;
}

void get_depth_map(
    const uint8_t* image,
    uint8_t* depth_map,
    int image_w,
    int image_h,
    const std::vector<Point>& seed_points
)
{
    // 初始化深度图为0
    memset(depth_map, 0, image_w * image_h * sizeof(uint8_t));

    // 对每个种子点所在的连通域分别获取深度图
    for (const auto& seed : seed_points)
    {
        // 如果该点已经被染色，跳过
        if (depth_map[seed.y() * image_w + seed.x()] > 0) continue;

        // 调用 flood_fill_with_depth 对该连通域进行层次遍历染色
        flood_fill_with_depth(image, depth_map, image_w, image_h, seed);
    }
}

static void build_region_graph(
    LineTrackingGraph& graph,
    const uint8_t* image,
    int image_w,
    int image_h,
    const Point& seed,
    uint8_t dist_thresh,
    int16_t* point_to_node_map
)
{
    // 1.1 为主连通域进行层次遍历建图
    vector<Point> current_layer;
    vector<Point> prev_layer; // 用于存储上一层的像素点

    // 创建起始节点
    int start_node_idx = graph.addNode(seed);
    if (start_node_idx == -1)
    {
        // 图已满，无法添加更多节点
        return;
    }
    point_to_node_map[seed.y() * image_w + seed.x()] = start_node_idx;
    prev_layer.push_back(seed);

    while (!prev_layer.empty())
    {
        // 清空当前层
        current_layer.clear();
        int16_t current_node_idx = -1; // 当前处理的节点
        Point current_node_point = NULL_POINT; // 当前节点对应的代表点

        // 对上一层的每个像素点进行处理
        for (const auto& pt : prev_layer)
        {
            // 获取当前像素点对应的节点索引
            int node_idx = point_to_node_map[pt.y() * image_w + pt.x()];
            if (node_idx == -1) continue;

            // 获取当前节点的所有邻居
            const auto& neighbors = pt.neighbors();
            for (const auto& neighbor : neighbors)
            {
                // 如果是当前层未处理过的节点
                if (inBounds(neighbor.x(), neighbor.y(), image_w, image_h) &&
                    point_to_node_map[neighbor.y() * image_w + neighbor.x()] == -1 &&
                    image[neighbor.y() * image_w + neighbor.x()] != 0) // 确保是白色像素
                {
                    current_layer.push_back(neighbor);
                    // 如果当前节点索引未设置或像素超出聚类阈值
                    if (current_node_idx == -1 ||
                        manhattanDist(neighbor, current_node_point) > dist_thresh)
                    {
                        current_node_idx = graph.addNode(neighbor);
                        if (current_node_idx == -1)
                        {
                            // 图已满，无法添加更多节点
                            return;
                        }
                        current_node_point = neighbor;
                        point_to_node_map[neighbor.y() * image_w + neighbor.x()] = current_node_idx;
                        // 连接节点
                        graph.getNode(node_idx).add_successor(current_node_idx);
                        graph.getNode(current_node_idx).set_predecessor(node_idx);
                    }
                    else
                    {
                        // 如果当前节点已经存在，直接使用现有节点索引
                        point_to_node_map[neighbor.y() * image_w + neighbor.x()] = current_node_idx;
                    }
                }
            }
        }

        // 更新上一层为当前层
        prev_layer = current_layer;
    }
}


void build_graph(
    LineTrackingGraph& graph,
    const uint8_t* image,
    int image_w,
    int image_h,
    const std::vector<pair<Point, Point>>& seed_link_points,
    int16_t* point_to_node_map,
    uint8_t dist_thresh
)
{
    // 初始化节点映射数组，-1表示该位置没有节点
    for (int i = 0; i < image_w * image_h; ++i)
    {
        point_to_node_map[i] = -1;
    }

    // 1. 处理第一个seed连通域，建立主连通域的图
    Point main_seed = seed_link_points[0].first;
    build_region_graph(graph, image, image_w, image_h, main_seed, dist_thresh, point_to_node_map);

    // 设置根节点并标记为START类型
    int root_idx = point_to_node_map[main_seed.y() * image_w + main_seed.x()];
    graph.set_root(root_idx);
    if (root_idx >= 0 && root_idx < graph.size())
    {
        graph.getNode(root_idx).set_type(NodeType::START);
    }

    // 2. 处理其他seed连通域，并连接到对应的link节点
    for (size_t i = 1; i < seed_link_points.size(); ++i)
    {
        Point seed = seed_link_points[i].first;
        Point link = seed_link_points[i].second;
        build_region_graph(graph, image, image_w, image_h, seed, dist_thresh, point_to_node_map);

        // 获取当前seed对应的节点索引
        auto seed_node_idx = point_to_node_map[seed.y() * image_w + seed.x()];
        auto link_node_idx = point_to_node_map[link.y() * image_w + link.x()];
        if (seed_node_idx != -1)
        {
            // 连接方向：从link节点连接到seed节点（link是父节点，seed是子节点）
            graph.getNode(link_node_idx).add_successor(seed_node_idx);
            graph.getNode(seed_node_idx).set_predecessor(link_node_idx);
        }

    }

    // return graph;
}

// vector<Point> get_line_elements(const LineTrackingGraph& graph)
// {
//     vector<Point> line_points;

//     auto start = graph.root();
//     while()
//     // 从根节点开始，进行深度优先遍历
//     std::function<void(int)> dfs = [&](int node_idx)
//     {
//         const auto& node = graph.getNode(node_idx);
//         line_points.push_back(node.data());

//         for (int successor : node.successors())
//         {
//             dfs(successor);
//         }
//     };

//     dfs(graph.root());

//     return line_points;
// }