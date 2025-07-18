#include "ui.h"
#include <algorithm>
#include <cmath>
#include "zf_device_ips114.h"
#include "middleware/vision/track/line_tracking.h"

extern LineTrackingGraph graph;

using namespace std;

void UI::screen_show_string(const uint16 x, const uint16 y, const string &str, const uint8 len)
{
    uint8 fixed_len;
    string fixed_str;
    if (len == UINT8_MAX)
        fixed_len = str.length();
    else
        fixed_len = len;

    if (str.length() > fixed_len)
        fixed_str = str.substr(0, fixed_len);
    else
        fixed_str = str + string(fixed_len - str.length(), ' ');
    ips114_show_string(x, y, fixed_str.c_str());
}

void UI::display_menu()
{
    if (need_clear_)
    {
        ips114_clear();
        need_clear_ = false;
    }

    display_vars();
    display_cursor();
    display_page_info();
}

void UI::display_vars()
{
    int start_index = current_page_ * vars_per_page_;
    int end_index = min(start_index + vars_per_page_, total_vars_);

    for (int i = start_index; i < end_index; ++i)
    {
        const DebugVar *var = var_ptrs_[i];
        screen_show_string(menu_display_x_ + 1 * 8, (i - start_index) * 16, var->name, 9);
        screen_show_string(menu_display_x_ + 12 * 8, (i - start_index) * 16, var->get(), 5);
    }
}

/**
 * @brief 显示光标
 */
void UI::display_cursor()
{
    int cursor_y = (current_var_index_ % vars_per_page_) * 16;

    for (int i = 0; i < 7; i++)
    {
        if (i == (current_var_index_ % vars_per_page_))
        {
            screen_show_string(menu_display_x_, cursor_y, "*");
        }
        else
        {
            screen_show_string(menu_display_x_, i * 16, " ");
        }
    }
}

void UI::display_page_info()
{
    string page_info = "Page " + to_string(current_page_ + 1) + "/" + to_string(total_pages_) + (edit_mode_ ? " change " : " display");
    screen_show_string(menu_display_x_, vars_per_page_ * 16, page_info);
}
