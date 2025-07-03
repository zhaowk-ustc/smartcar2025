#include "line_tracking.h"
#include "line_tracking_graph.h"
#include <iostream>
#include <queue>
#include <cmath>
#include <algorithm>
#include <cstdlib>
#include <cstring>

// 8邻域
const int dx[8] = { 1,1,0,-1,-1,-1,0,1 };
const int dy[8] = { 0,-1,-1,-1,0,1,1,1 };

// 虚线连接检测和修复
void connectDashedLines(vector<uint8_t>& img, int w, int h, int max_gap) {
    vector<uint8_t> result = img;
    
    // 对每个白色像素点，检查其周围是否有断裂的线段可以连接
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            if (img[y * w + x] == 0) continue; // 只处理白色像素
            
            // 检查8个方向，寻找可能的断裂连接
            for (int dir = 0; dir < 8; ++dir) {
                int dx_step = dx[dir];
                int dy_step = dy[dir];
                
                // 沿着当前方向搜索，寻找断裂后的线段
                bool found_gap = false;
                int gap_length = 0;
                int end_x = -1, end_y = -1;
                
                for (int step = 1; step <= max_gap; ++step) {
                    int nx = x + dx_step * step;
                    int ny = y + dy_step * step;
                    
                    if (nx < 0 || nx >= w || ny < 0 || ny >= h) break;
                    
                    if (img[ny * w + nx] == 0) {
                        // 遇到黑色像素，继续搜索
                        gap_length = step;
                        continue;
                    } else {
                        // 找到白色像素，检查是否为有效连接
                        if (gap_length > 0 && gap_length <= max_gap) {
                            // 验证这是一个有效的线段端点
                            int neighbor_count = 0;
                            for (int d = 0; d < 8; ++d) {
                                int nnx = nx + dx[d];
                                int nny = ny + dy[d];
                                if (nnx >= 0 && nnx < w && nny >= 0 && nny < h && 
                                    img[nny * w + nnx] > 0) {
                                    neighbor_count++;
                                }
                            }
                            
                            // 如果端点周围有足够的邻居，认为是有效连接
                            if (neighbor_count >= 1 && neighbor_count <= 3) {
                                end_x = nx;
                                end_y = ny;
                                found_gap = true;
                            }
                        }
                        break;
                    }
                }
                
                // 如果找到有效的断裂连接，填补间隙
                if (found_gap && end_x >= 0 && end_y >= 0) {
                    // 使用线性插值填补间隙
                    int steps = max(abs(end_x - x), abs(end_y - y));
                    if (steps > 0) {
                        for (int i = 1; i < steps; ++i) {
                            int fill_x = x + (end_x - x) * i / steps;
                            int fill_y = y + (end_y - y) * i / steps;
                            if (fill_x >= 0 && fill_x < w && fill_y >= 0 && fill_y < h) {
                                result[fill_y * w + fill_x] = 1;
                            }
                        }
                    }
                }
            }
        }
    }
    
    img = result;
}

// 端点检测：识别可能的线段端点
vector<pair<int, int>> detectEndpoints(const vector<uint8_t>& img, int w, int h) {
    vector<pair<int, int>> endpoints;
    
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            if (img[y * w + x] == 0) continue;
            
            // 计算8邻域中的白色像素数量
            int neighbor_count = 0;
            for (int d = 0; d < 8; ++d) {
                int nx = x + dx[d];
                int ny = y + dy[d];
                if (nx >= 0 && nx < w && ny >= 0 && ny < h && img[ny * w + nx] > 0) {
                    neighbor_count++;
                }
            }
            
            // 端点通常只有1个邻居
            if (neighbor_count == 1) {
                endpoints.push_back({x, y});
            }
        }
    }
    
    return endpoints;
}

// 改进的虚线修复：基于端点配对
void repairDashedLinesAdvanced(vector<uint8_t>& img, int w, int h, double max_gap) {
    // 1. 首先进行基础的虚线连接
    connectDashedLines(img, w, h, (int)max_gap);
    
    // 2. 检测端点
    vector<pair<int, int>> endpoints = detectEndpoints(img, w, h);
    
    // 3. 对端点进行配对连接
    vector<bool> used(endpoints.size(), false);
    
    for (size_t i = 0; i < endpoints.size(); ++i) {
        if (used[i]) continue;
        
        int x1 = endpoints[i].first;
        int y1 = endpoints[i].second;
        
        // 寻找最近的未配对端点
        double min_dist = max_gap + 1;
        int best_match = -1;
        
        for (size_t j = i + 1; j < endpoints.size(); ++j) {
            if (used[j]) continue;
            
            int x2 = endpoints[j].first;
            int y2 = endpoints[j].second;
            double dist = sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
            
            if (dist <= max_gap && dist < min_dist) {
                // 检查连接路径上是否有障碍
                bool path_clear = true;
                int steps = (int)dist;
                if (steps > 0) {
                    for (int step = 1; step < steps; ++step) {
                        int check_x = x1 + (x2 - x1) * step / steps;
                        int check_y = y1 + (y2 - y1) * step / steps;
                        if (check_x >= 0 && check_x < w && check_y >= 0 && check_y < h) {
                            // 检查周围是否有其他线段干扰
                            int interference_count = 0;
                            for (int d = 0; d < 8; ++d) {
                                int nx = check_x + dx[d];
                                int ny = check_y + dy[d];
                                if (nx >= 0 && nx < w && ny >= 0 && ny < h && img[ny * w + nx] > 0) {
                                    interference_count++;
                                }
                            }
                            if (interference_count > 2) {
                                path_clear = false;
                                break;
                            }
                        }
                    }
                }
                
                if (path_clear) {
                    min_dist = dist;
                    best_match = j;
                }
            }
        }
        
        // 如果找到合适的配对，连接两个端点
        if (best_match >= 0) {
            int x2 = endpoints[best_match].first;
            int y2 = endpoints[best_match].second;
            
            // 使用Bresenham算法绘制连接线
            int dx_line = abs(x2 - x1);
            int dy_line = abs(y2 - y1);
            int sx = (x1 < x2) ? 1 : -1;
            int sy = (y1 < y2) ? 1 : -1;
            int err = dx_line - dy_line;
            
            int curr_x = x1, curr_y = y1;
            while (true) {
                if (curr_x >= 0 && curr_x < w && curr_y >= 0 && curr_y < h) {
                    img[curr_y * w + curr_x] = 1;
                }
                
                if (curr_x == x2 && curr_y == y2) break;
                
                int e2 = 2 * err;
                if (e2 > -dy_line) { err -= dy_line; curr_x += sx; }
                if (e2 < dx_line) { err += dx_line; curr_y += sy; }
            }
            
            used[i] = used[best_match] = true;
        }
    }
}

// BFS层次遍历，构建拓扑图
LineTrackingGraph bfs_layer_midpoints(const uint8_t* img, int w, int h, double dist_thresh) {
    // ===== 1. 初始化阶段 =====
    // 1.1 寻找起点：遍历整个图像，找到最左侧的白色像素点作为BFS的起始点
    int min_x = w, min_y = 0;
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            if (img[y * w + x] && x < min_x) { 
                min_x = x; 
                min_y = y; 
            }
        }
    }
    if (min_x == w) return LineTrackingGraph(); // 没有找到白色像素

    // 1.2 创建数据结构
    LineTrackingGraph graph;                         // 存储拓扑图结构
    vector<bool> visited(w * h, false);              // 与图像同尺寸的bool数组，标记已访问的像素
    vector<int> prev_layer_indices;                  // 存储上一层节点的索引
    queue<pair<int, int>> bfs_queue;                 // BFS队列：用于层次遍历

    // ===== 2. 起始节点处理 =====
    // 2.1 将起始节点加入拓扑图，并设置起始索引
    graph.addFirstNode({static_cast<int16>(min_x), static_cast<int16>(min_y)});
    prev_layer_indices.push_back(0);
    
    // 2.2 标记起点为已访问，加入BFS队列
    visited[min_y * w + min_x] = true;
    bfs_queue.push({ min_x, min_y });

    // ===== 3. BFS层次遍历主循环 =====
    while (!bfs_queue.empty()) {
        int current_layer_size = (int)bfs_queue.size();
        vector<Point> current_layer; // 当前层的所有像素点
        vector<int> pixel_sources; // 每个像素点对应的源节点索引

        // 3.1 当前层像素收集 - 追踪像素来源
        // 从队列中取出当前层的所有像素点，并记录它们的来源节点
        for (int i = 0; i < current_layer_size; ++i) {
            auto pixel = bfs_queue.front(); 
            bfs_queue.pop();
            int x = pixel.first, y = pixel.second;
            
            // 边界检查
            if (x < 0 || x >= w || y < 0 || y >= h) continue;
            
            // 找到当前像素是从哪个前一层节点扩展而来的
            int source_node_idx = -1;
            double min_dist = 1e9;
            for (int prev_idx : prev_layer_indices) {
                const Point& prev_pt = graph.nodes[prev_idx].pt;
                double dist = sqrt((x - prev_pt.x) * (x - prev_pt.x) + (y - prev_pt.y) * (y - prev_pt.y));
                if (dist < min_dist) {
                    min_dist = dist;
                    source_node_idx = prev_idx;
                }
            }
            
            // 对每个像素点的8邻域进行扩展，将未访问的白色像素加入队列
            for (int d = 0; d < 8; ++d) {
                int nx = x + dx[d], ny = y + dy[d];
                if (nx >= 0 && nx < w && ny >= 0 && ny < h && img[ny * w + nx]) {
                    int idx = ny * w + nx;
                    if (!visited[idx]) {
                        visited[idx] = true;
                        current_layer.push_back({static_cast<int16>(nx), static_cast<int16>(ny)}); // 收集当前层的所有像素点
                        pixel_sources.push_back(source_node_idx); // 记录像素来源节点
                        bfs_queue.push({ nx, ny });
                    }
                }
            }
        }
        
        // 如果当前层没有新像素，结束遍历
        if (current_layer.empty()) break;
        
        // 3.2 像素点分组聚类 - 按来源节点和距离分组
        // 使用距离阈值对当前层像素进行分组，同时考虑来源节点
        vector<vector<Point>> pixel_groups;
        vector<int> group_sources; // 每个分组对应的源节点索引
        vector<bool> used(current_layer.size(), false);
        
        for (size_t i = 0; i < current_layer.size(); ++i) {
            if (used[i]) continue;
            
            // 创建新分组，以当前点为种子
            vector<Point> group = { current_layer[i] };
            int group_source = pixel_sources[i];
            used[i] = true;
            
            // 距离小于阈值且来源相同的像素点归为同一组
            for (size_t j = i + 1; j < current_layer.size(); ++j) {
                if (!used[j] && pixel_sources[j] == group_source) {
                    double distance = sqrt(
                        (current_layer[i].x - current_layer[j].x) * (current_layer[i].x - current_layer[j].x) +
                        (current_layer[i].y - current_layer[j].y) * (current_layer[i].y - current_layer[j].y)
                    );
                    if (distance < dist_thresh) {
                        group.push_back(current_layer[j]);
                        used[j] = true;
                    }
                }
            }
            pixel_groups.push_back(group);
            group_sources.push_back(group_source);
        }
        
        // 3.3 中点计算与节点创建 - 基于分裂关系
        vector<int> current_layer_indices;
        map<int, vector<int>> source_to_children; // 源节点到子节点的映射
        
        for (size_t g = 0; g < pixel_groups.size(); ++g) {
            const auto& group = pixel_groups[g];
            int source_idx = group_sources[g];
            
            if (group.empty()) continue;
            
            // 对每个分组计算几何中心点作为该组的代表点
            int sum_x = 0, sum_y = 0;
            for (const auto& pt : group) {
                sum_x += pt.x;
                sum_y += pt.y;
            }
            Point center = {static_cast<int16>(sum_x / (int)group.size()), static_cast<int16>(sum_y / (int)group.size())};
            
            // 使用addNode方法创建新节点
            int new_node_index = graph.addNode(center);
            current_layer_indices.push_back(new_node_index);
            
            // 记录分裂关系：源节点分裂出当前节点
            source_to_children[source_idx].push_back(new_node_index);
        }
        
        // 3.4 建立层间连接 - 基于分裂关系
        // 根据记录的分裂关系建立后继连接
        for (const auto& pair : source_to_children) {
            int source_idx = pair.first;
            const vector<int>& children = pair.second;
            
            // 源节点连接到所有分裂出的子节点
            for (int child_idx : children) {
                graph.nodes[source_idx].successors.push_back(child_idx);
                graph.nodes[child_idx].predecessors.push_back(source_idx); // 添加前继关系
            }
        }
        
        // 更新层索引
        prev_layer_indices = current_layer_indices;
    }
    
    // ===== 4. 节点类型标注 =====
    // 遍历完成后，根据每个节点的后继数量自动标注节点类型
    for (size_t i = 0; i < graph.nodes.size(); ++i) {
        auto& node = graph.nodes[i];
        int successor_count = (int)node.successors.size();
        
        if (successor_count == 0) {
            node.type = 1; // endpoint：没有后继的端点
        } else if (successor_count == 1) {
            node.type = 0; // normal：有1个后继的普通节点
        } else {
            node.type = 2; // branch：有2个或更多后继的分叉点
        }
    }
    
    return graph;
}

// 辅助函数：在图像上绘制圆点
void draw_circle_on_image(uint8_t* img_data, int width, int height, 
                         int center_x, int center_y, int radius, 
                         uint8_t r_val, uint8_t g_val, uint8_t b_val) {
    for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
            if (dx*dx + dy*dy <= radius*radius) {
                int x = center_x + dx;
                int y = center_y + dy;
                if (x >= 0 && x < width && y >= 0 && y < height) {
                    int idx = (y * width + x) * 3; // BGR格式
                    img_data[idx + 0] = b_val;     // B
                    img_data[idx + 1] = g_val;     // G
                    img_data[idx + 2] = r_val;     // R
                }
            }
        }
    }
}

// 输入二值图像，输出简化后的拓扑结构和可视化图像数据
uint8_t* extract_and_visualize_topology(const uint8_t* bin_img, int width, int height, 
                                       LineTrackingGraph& simplified_graph) {
    // 0. 虚线修复预处理
    vector<uint8_t> arr(height * width);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            arr[y * width + x] = (bin_img[y * width + x] > 0) ? 1 : 0;
        }
    }
    repairDashedLinesAdvanced(arr, width, height, 20.0);

    // 1. 提取主干拓扑图
    double dist_thresh = 10.0;
    LineTrackingGraph graph = bfs_layer_midpoints(arr.data(), width, height, dist_thresh);

    // 2. 路径段简化
    double simplify_epsilon = 3.0;
    simplified_graph = simplifyGraph(graph, simplify_epsilon);

    // 3. 创建可视化图像 (BGR格式)
    uint8_t* vis_img = (uint8_t*)malloc(width * height * 3 * sizeof(uint8_t));
    if (!vis_img) return nullptr;
    
    // 将二值图像转换为BGR格式作为背景
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int src_idx = y * width + x;
            int dst_idx = (y * width + x) * 3;
            uint8_t gray_val = bin_img[src_idx];
            vis_img[dst_idx + 0] = gray_val; // B
            vis_img[dst_idx + 1] = gray_val; // G
            vis_img[dst_idx + 2] = gray_val; // R
        }
    }

    // 绘制简化后的节点
    for (const auto& node : simplified_graph.nodes) {
        uint8_t r, g, b;
        int radius = 3;
        
        if (node.type == 1) {           // 端点
            r = 255; g = 100; b = 100;  // 浅红色
        } else if (node.type == 2) {    // 分叉点
            r = 100; g = 100; b = 255;  // 浅蓝色
        } else if (node.type == 3) {    // 环接点
            r = 100; g = 255; b = 255;  // 浅黄色
        } else {                        // 普通点
            r = 100; g = 255; b = 100;  // 浅绿色
        }
        
        draw_circle_on_image(vis_img, width, height, node.pt.x, node.pt.y, radius, r, g, b);
    }
    
    return vis_img;
}