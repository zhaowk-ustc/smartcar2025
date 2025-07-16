#include "ui.h"
#include <algorithm>
#include <cmath>
#include "zf_device_ips114.h"
#include "middleware/vision/track/line_tracking.h"

#include "multicore/core_shared.h"


void UI::display_camera()
{
    // extern uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];

    const uint8* frame_buffer = nullptr;
    uint16 width = 0;
    uint16 height = 0;

    switch (camera_display_mode_)
    {
        case CameraDisplayMode::ORIGINAL:
            frame_buffer = &mt9v03x_image[0][0];
            width = MT9V03X_W;
            height = MT9V03X_H;
            // 刷新缓存，确保从内存重新读取图像数据
            SCB_InvalidateDCache_by_Addr((void*)frame_buffer, width * height);
            // 显示缩放2倍
            ips114_show_gray_image(camera_display_x_, 0, frame_buffer, width, height, width / 2, height / 2, 0);
            break;
        case CameraDisplayMode::CALIBRATED:
            frame_buffer = calibrated_image;
            width = calibrated_width;
            height = calibrated_height;
            SCB_InvalidateDCache_by_Addr((void*)frame_buffer, width * height);
            // 直接显示原始大小
            ips114_show_gray_image(camera_display_x_, 0, frame_buffer, width, height, width, height, 0);
            break;
        case CameraDisplayMode::CALIBRATED_BINARIZED:
            frame_buffer = calibrated_binary_image;
            width = calibrated_width;
            height = calibrated_height;
            SCB_InvalidateDCache_by_Addr((void*)frame_buffer, width * height);
            // 直接显示原始大小
            ips114_show_gray_image(camera_display_x_, 0, frame_buffer, width, height, width, height, 0);
            break;
        default:
            break;
    }

    if (camera_display_mode_ == CameraDisplayMode::CALIBRATED_BINARIZED)

        display_overlay(); // 显示拓扑图叠加层
}

void UI::select_camera_display_mode()
{
    if (camera_display_mode_ == CameraDisplayMode::ORIGINAL)
    {
        camera_display_mode_ = CameraDisplayMode::CALIBRATED;
    }
    else if (camera_display_mode_ == CameraDisplayMode::CALIBRATED)
    {
        camera_display_mode_ = CameraDisplayMode::CALIBRATED_BINARIZED;
    }
    else
    {
        camera_display_mode_ = CameraDisplayMode::ORIGINAL;
    }
    ips114_clear();
}


void UI::display_overlay()
{
    SCB_InvalidateDCache_by_Addr((void*)&vision_debug_shared, sizeof(vision_debug_shared));
    SCB_InvalidateDCache_by_Addr((void*)&vision_outputs_shared, sizeof(vision_outputs_shared));
    SCB_InvalidateDCache_by_Addr((void*)&vision_line_tracking_graph, sizeof(vision_line_tracking_graph));
    static LineTrackingGraph ui_display_graph;
    memcpy(&ui_display_graph, &vision_line_tracking_graph, sizeof(vision_line_tracking_graph));
    static TrackPath ui_display_path;
    memcpy(&ui_display_path, &vision_outputs_shared.track_path, sizeof(vision_outputs_shared.track_path));
    draw_path_overlay(ui_display_path);
    // draw_graph_overlay(ui_display_graph);
}

// 融合显示路径和图结构
void UI::draw_path_and_graph_overlay(const TrackPath& path, const LineTrackingGraph& graph)
{
    // 3. 画path节点（小圆点）
    for (size_t i = 0; i < path.size(); ++i)
    {
        const auto& node = path.nodes[i];
        uint16_t x = camera_display_x_ + static_cast<int>(node.pos.x());
        uint16_t y = static_cast<int>(node.pos.y());
        uint16_t color = RGB565_GRAY;
        if (i == 0) color = RGB565_BLUE; // 起点高亮
        draw_square(x, y, 4, color);
    }
    // // 4. 画graph节点（大圆点，按类型区分颜色）
    // for (size_t i = 0; i < graph.size(); ++i)
    // {
    //     const GraphNode& node = graph.getNode(i);
    //     if (node.type() == NodeType::DELETED) continue;
    //     Point pt = node.data();
    //     uint16_t x = camera_display_x_ + pt.x();
    //     uint16_t y = pt.y();
    //     uint16_t color = RGB565_BLUE;
    //     switch (node.type())
    //     {
    //         case NodeType::NORMAL: color = RGB565_BLUE; break;
    //         case NodeType::ENDPOINT: color = RGB565_RED; break;
    //         case NodeType::BRANCH: color = RGB565_GREEN; break;
    //         case NodeType::START: color = RGB565_CYAN; break;
    //         default: break;
    //     }
    //     draw_square(x, y, 4, color);
    // }

    char pos_buffer[32];
    snprintf(pos_buffer, sizeof(pos_buffer), "(%d, %d)",
        static_cast<int>(vision_debug_shared.pure_pursuit_target.x()),
        static_cast<int>(vision_debug_shared.pure_pursuit_target.y()));
    screen_show_string(camera_display_x_, calibrated_height + 16, pos_buffer, 10);
}

// 绘制拓扑图叠加层
void UI::draw_graph_overlay(const LineTrackingGraph& graph)
{
    // 第一遍遍历：绘制所有节点
    draw_graph_nodes(graph);

    // 第二遍遍历：绘制所有连线
    draw_graph_edges(graph);

    // 第三遍遍历：绘制根节点标记
    draw_graph_root(graph);

    // 第四遍遍历：显示统计信息
    // display_graph_stats(graph);
}

// 绘制图中的所有节点
void UI::draw_graph_nodes(const LineTrackingGraph& graph)
{
    for (size_t i = 0; i < graph.size(); ++i)
    {
        const GraphNode& node = graph.getNode(i);
        Point pt = node.data();

        // 图像坐标缩放
        uint16 screen_x = camera_display_x_ + pt.x();
        uint16 screen_y = pt.y();

        // 根据节点类型选择不同的绘制方式
        switch (node.type())
        {
            case NodeType::NORMAL:
                draw_square(screen_x, screen_y, 2, RGB565_BLUE);  // 白色小方块
                break;
            case NodeType::ENDPOINT:
                draw_square(screen_x, screen_y, 3, RGB565_RED);    // 红色方块
                break;
            case NodeType::BRANCH:
                draw_square(screen_x, screen_y, 4, RGB565_GREEN);  // 绿色方块
                break;
            case NodeType::START:
                draw_square(screen_x, screen_y, 5, RGB565_CYAN);   // 青色方块，表示起点
                break;
            default:
                break;
        }
    }
}

// 绘制图中的所有连线
void UI::draw_graph_edges(const LineTrackingGraph& graph)
{
    for (size_t i = 0; i < graph.size(); ++i)
    {
        const GraphNode& node = graph.getNode(i);
        Point pt = node.data();

        if (node.type() == NodeType::DELETED)
            continue; // 跳过已删除的节点

        // 图像坐标缩放
        uint16 screen_x = camera_display_x_ + pt.x();
        uint16 screen_y = pt.y();

        // 绘制到后继节点的连线
        vector<int16_t> successors = node.successors();
        for (int succ_idx : successors)
        {
            if (succ_idx >= 0 && succ_idx < graph.size())
            {
                const GraphNode& succ_node = graph.getNode(succ_idx);
                Point succ_pt = succ_node.data();

                uint16 succ_screen_x = camera_display_x_ + succ_pt.x();
                uint16 succ_screen_y = succ_pt.y();

                // 绘制连线
                ips114_draw_line(screen_x, screen_y, succ_screen_x, succ_screen_y, RGB565_YELLOW);
            }
        }
    }
}

// 绘制根节点标记
void UI::draw_graph_root(const LineTrackingGraph& graph)
{
    if (graph.root() >= 0 && graph.root() < graph.size())
    {
        const GraphNode& root_node = graph.getNode(graph.root());
        Point root_pt = root_node.data();
        uint16 root_screen_x = camera_display_x_ + root_pt.x();
        uint16 root_screen_y = root_pt.y();

        // 在根节点周围绘制一个大方框
        draw_square(root_screen_x, root_screen_y, 6, RGB565_BLUE);
    }
}

// 显示图的统计信息
void UI::display_graph_stats(const LineTrackingGraph& graph)
{
    // 统计节点类型数量
    int normal_count = 0;
    int endpoint_count = 0;
    int branch_count = 0;
    int start_count = 0;

    for (size_t i = 0; i < graph.size(); ++i)
    {
        const GraphNode& node = graph.getNode(i);
        if (node.type() == NodeType::DELETED)
            continue; // 跳过已删除的节点
        switch (node.type())
        {
            case NodeType::NORMAL:
                normal_count++;
                break;
            case NodeType::ENDPOINT:
                endpoint_count++;
                break;
            case NodeType::BRANCH:
                branch_count++;
                break;
            case NodeType::START:
                start_count++;
                break;
            default:
                break;
        }
    }

    // 显示统计信息
    char stats_buffer[32];

    // 显示总节点数
    snprintf(stats_buffer, sizeof(stats_buffer), "Nodes: %d", (int)graph.size());
    screen_show_string(camera_display_x_, calibrated_height, stats_buffer, 10);

    // 显示各类型节点数量
    snprintf(stats_buffer, sizeof(stats_buffer), "Normal: %d", normal_count);
    screen_show_string(camera_display_x_, calibrated_height + 16, stats_buffer, 10);

    snprintf(stats_buffer, sizeof(stats_buffer), "Branch: %d", branch_count);
    screen_show_string(camera_display_x_, calibrated_height + 32, stats_buffer, 10);

    snprintf(stats_buffer, sizeof(stats_buffer), "End: %d", endpoint_count);
    screen_show_string(camera_display_x_, calibrated_height + 48, stats_buffer, 10);

    snprintf(stats_buffer, sizeof(stats_buffer), "Start: %d", start_count);
    screen_show_string(camera_display_x_, calibrated_height + 64, stats_buffer, 10);

    // 显示根节点(START)坐标信息
    if (graph.root() >= 0 && graph.root() < graph.size())
    {
        const GraphNode& root_node = graph.getNode(graph.root());
        Point root_pt = root_node.data();

        // 直接显示坐标，根节点即START节点
        snprintf(stats_buffer, sizeof(stats_buffer), "(%d,%d)", root_pt.x(), root_pt.y());
        // screen_show_string(camera_display_x_, calibrated_height  + 80, stats_buffer, 10);
    }
}

// 绘制方块的辅助函数
void UI::draw_square(uint16 center_x, uint16 center_y, uint8 size, uint16 color)
{
    int half_size = size;

    // 绘制方块的四条边
    for (int i = -half_size; i <= half_size; ++i)
    {
        // 上边和下边
        if (center_y - half_size >= 0)
        {
            ips114_draw_point(center_x + i, center_y - half_size, color);
        }
        if (center_y + half_size >= 0)
        {
            ips114_draw_point(center_x + i, center_y + half_size, color);
        }

        // 左边和右边
        if (center_y + i >= 0)
        {
            ips114_draw_point(center_x - half_size, center_y + i, color);
            ips114_draw_point(center_x + half_size, center_y + i, color);
        }
    }
}

// 绘制轨迹路径（TrackPath）叠加层
void UI::draw_path_overlay(const TrackPath& path)
{
    // 绘制节点和连接线
    uint16_t color_normal = RGB565_BLUE;
    uint16_t color_roundabout = RGB565_RED;
    uint16_t color_cross = RGB565_CYAN;
    uint16_t color_uncertain = RGB565_BROWN;
    uint16_t color_start = RGB565_GREEN;

    for (auto i = 0;i < path.size();i++)
    {
        const auto& node = path.nodes[i];
        auto node_pt = node.pos;
        uint16_t screen_x = camera_display_x_ + static_cast<int>(node_pt.x());
        uint16_t screen_y = static_cast<int>(node_pt.y());
        uint16_t color = color_normal; // 默认颜色为白色
        uint16_t node_radius = 3; // 节点半径
        switch (node.element)
        {
            case ElementType::NORMAL:
                color = color_normal;
                node_radius = 3;
                break;
            case ElementType::ROUNDABOUT:
                color = color_roundabout;
                node_radius = 6;
                break;
            case ElementType::CROSS:
                color = color_cross;
                node_radius = 6;
                break;
            case ElementType::UNCERTAIN:
                color = color_uncertain;
                node_radius = 4;
                break;
            default:
                break;
        }
        if (i == 0) // 如果是起点
        {
            color = color_start; // 起点颜色为蓝色
            node_radius = 5; // 起点半径稍大
        }
        draw_square(screen_x, screen_y, node_radius, color);
        // 画出next_vec方向
        if (node.next_length != 0)
        {
            uint16_t next_x = screen_x + static_cast<int>(node.next_dir.x() * node.next_length);
            uint16_t next_y = screen_y + static_cast<int>(node.next_dir.y() * node.next_length);
            ips114_draw_line(screen_x, screen_y, next_x, next_y, color);
        }
    }

    // 显示统计信息
    auto first_element = ElementType::NORMAL;
    Point2f first_pos = { 0, 0 };
    for (size_t i = 0; i < path.size(); ++i)
    {
        const auto& node = path.nodes[i];
        if (node.element != ElementType::NORMAL)
        {
            first_element = node.element;
            first_pos = node.pos;
            break;
        }
    }
    // screen_show_string(camera_display_x_, calibrated_height, "Element:", 10);
    switch (first_element)
    {
        case ElementType::NORMAL:
            screen_show_string(camera_display_x_, calibrated_height, "Normal", 10);
            break;
        case ElementType::ROUNDABOUT:
            screen_show_string(camera_display_x_, calibrated_height, "Roundabout", 10);
            break;
        case ElementType::CROSS:
            screen_show_string(camera_display_x_, calibrated_height, "Cross", 10);
            break;
        case ElementType::UNCERTAIN:
            screen_show_string(camera_display_x_, calibrated_height, "Uncertain", 10);
            break;
        default:
            screen_show_string(camera_display_x_, calibrated_height, "Unknown", 10);
            break;
    }
    // screen_show_string(camera_display_x_, calibrated_height + 32, "Pos:", 10);
    char pos_buffer[32];
    snprintf(pos_buffer, sizeof(pos_buffer), "(%d, %d)", static_cast<int>(first_pos.x()), static_cast<int>(first_pos.y()));
    screen_show_string(camera_display_x_, calibrated_height + 16, pos_buffer, 10);
    snprintf(pos_buffer, sizeof(pos_buffer), "(%d, %d)", static_cast<int>(vision_debug_shared.pure_pursuit_target.x()), static_cast<int>(vision_debug_shared.pure_pursuit_target.y()));
    screen_show_string(camera_display_x_, calibrated_height + 32, pos_buffer, 10);
    if (vision_debug_shared.pure_pursuit_target.x() > 0.01f && vision_debug_shared.pure_pursuit_target.y() > 0.01f)
    {
        int screen_x = camera_display_x_ + static_cast<int>(vision_debug_shared.pure_pursuit_target.x());
        int screen_y = static_cast<int>(vision_debug_shared.pure_pursuit_target.y());
        draw_square(screen_x, screen_y, 3, RGB565_YELLOW); // 绘制目标点
    }
}