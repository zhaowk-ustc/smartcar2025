#include "ui.h"
#include <algorithm>
using namespace std;

void UI::screen_show_string(const uint16 x, const uint16 y, const string& str, const uint8 len)
{
    uint8 fixed_len;
    string fixed_str;
    if (len == UINT8_MAX)
        fixed_len = str.length();
    else
        fixed_len = min(len, (uint8)str.length());

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
    display_camera();
}

void UI::display_vars()
{
    int start_index = current_page_ * vars_per_page_;
    int end_index = min(start_index + vars_per_page_, total_vars_);

    for (int i = start_index; i < end_index; ++i)
    {
        const DebugVar* var = var_ptrs_[i];
        screen_show_string(2 * 8, (i - start_index) * 16, var->name);
        screen_show_string(12 * 8, (i - start_index) * 16, var->get());
    }
}

void UI::display_cursor()
{
    int cursor_y = (current_var_index_ % vars_per_page_) * 16;
    screen_show_string(0, cursor_y, ">");
}

void UI::display_page_info()
{
    string page_info = "Page " + to_string(current_page_ + 1) + "/" + to_string(total_pages_);
    screen_show_string(0, 5 * 16, page_info);
}