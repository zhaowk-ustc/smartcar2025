#ifndef UI_H
#define UI_H

#include "zf_common_headfile.h"
#include "../debugger/debugger.h"
#include "middleware/vision/track/line_tracking_graph.h"
#include "middleware/vision/element/track_path.h"
#include <vector>
#include <string>
#include <cmath>

class UI : public IDebuggable
{
public:
    UI();

    void init();
    void update_mainloop();
    void update_pit();

    void import_vars(const vector<const DebugVar*>& var_ptrs);

    void enable();
    void disable();

private:
    vector<const DebugVar*> var_ptrs_;

    int total_vars_ = 0; // 总变量数
    int total_pages_ = 0; // 总页数
    int current_var_index_ = 0; // 当前变量索引
    int current_page_ = 0; // 当前页码

    static constexpr int vars_per_page_ = 7; // 每页显示的变量数

    void setup_debug_vars() override;

    // ui_ops.cpp 中的函数
    void next_item();
    void prev_item();
    void next_page();
    void prev_page();

    // ui_keymap.cpp 中的函数
    enum class Key
    {
        NONE,
        UP,
        DOWN,
        LEFT,
        RIGHT,
        ENTER,
    };
    Key current_key_ = Key::NONE; // 当前按键状态
    static Key get_key_input();
    void update_key_state();
    void handle_current_key();
    void on_key_up();
    void on_key_down();
    void on_key_left();
    void on_key_right();
    void on_key_enter();

    // ui_menu.cpp 中的函数
    static constexpr uint16 menu_display_x_ = 0 * 8;
    bool need_clear_ = false; // 是否需要清屏
    static void screen_show_string(const uint16 x, const uint16 y, const string& str, const uint8 len = UINT8_MAX);
    void display_menu();
    void display_vars();
    void display_cursor();
    void display_page_info();
    enum class CameraDisplayMode
    {
        ORIGINAL,
        CALIBRATED,
        CALIBRATED_BINARIZED
    };
    CameraDisplayMode camera_display_mode_ = CameraDisplayMode::CALIBRATED;
    void select_camera_display_mode();
    static constexpr uint16 camera_display_x_ = 18 * 8; // 摄像头显示的起始X坐标
    void display_camera();
    void display_overlay(); // 显示拓扑图叠加层
    void draw_path_and_graph_overlay(const TrackPath& path, const LineTrackingGraph& graph);
    
    // 拓扑图可视化相关函数
    void draw_graph_overlay(const LineTrackingGraph& graph);
    void draw_graph_nodes(const LineTrackingGraph& graph);
    void draw_graph_edges(const LineTrackingGraph& graph);
    void draw_graph_root(const LineTrackingGraph& graph);
    void display_graph_stats(const LineTrackingGraph& graph);
    void draw_square(uint16 center_x, uint16 center_y, uint8 size, uint16 color);
    void draw_path_overlay(const TrackPath& path);

    bool enabled_ = true; // UI是否启用
};

#endif // UI_H