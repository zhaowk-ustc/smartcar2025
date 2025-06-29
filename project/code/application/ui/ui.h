#ifndef UI_H
#define UI_H

#include "zf_common_headfile.h"
#include "../debugger/debugger.h"
#include <vector>
#include <string>

class UI
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
    
    static constexpr int vars_per_page_ = 4; // 每页显示的变量数

    // ui_ops.cpp 中的函数
    void next_item();
    void prev_item();
    void next_page();
    void prev_page();

    // ui_keymap.cpp 中的函数
    enum class Key {
        NONE,
        UP,
        DOWN,
        LEFT,
        RIGHT,
        ENTER,
    };
    Key current_key_ = Key::NONE; // 当前按键状态
    void update_key_state();
    void handle_current_key();
    void on_key_up();
    void on_key_down();
    void on_key_left();
    void on_key_right();
    void on_key_enter();

    // ui_menu.cpp 中的函数
    bool need_clear_ = false; // 是否需要清屏
    static void screen_show_string(const uint16 x, const uint16 y, const string& str, const uint8 len = UINT8_MAX);
    void display_menu();
    void display_vars();
    void display_cursor();
    void display_page_info();
    void display_camera();


    bool enabled_ = true; // UI是否启用
};

#endif // UI_H