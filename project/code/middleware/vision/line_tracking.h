#ifndef LINE_TRACKING_H
#define LINE_TRACKING_H

#include "image.h"
#include "line_tracking_graph.h"
#include <vector>
#include <cstdint>

using namespace std;

// 图像处理相关函数声明

// 虚线连接检测和修复
void connectDashedLines(vector<uint8_t>& img, int w, int h, int max_gap = 10);

// 端点检测：识别可能的线段端点
vector<pair<int, int>> detectEndpoints(const vector<uint8_t>& img, int w, int h);

// 改进的虚线修复：基于端点配对
void repairDashedLinesAdvanced(vector<uint8_t>& img, int w, int h, double max_gap = 15.0);

// BFS层次遍历，构建拓扑图
LineTrackingGraph bfs_layer_midpoints(const uint8_t* img, int w, int h, double dist_thresh = 5.0);

// 主流程：输入二值图像，输出简化后的拓扑结构和可视化图像数据
// 返回可视化图像的数据指针，调用者负责释放内存
uint8_t* extract_and_visualize_topology(const uint8_t* bin_img, int width, int height, 
                                       LineTrackingGraph& simplified_graph);

// 辅助函数：在图像上绘制圆点
void draw_circle_on_image(uint8_t* img_data, int width, int height, 
                         int center_x, int center_y, int radius, 
                         uint8_t r_val, uint8_t g_val, uint8_t b_val);

#endif // LINE_TRACKING_H