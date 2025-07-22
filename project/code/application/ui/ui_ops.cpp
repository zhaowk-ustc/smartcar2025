#include "ui.h"

void UI::next_item()
{
    current_var_index_++;
    if (current_var_index_ >= total_vars_)
    {
        current_var_index_ = 0;
        need_clear_ = true;
    }
    current_page_ = current_var_index_ / vars_per_page_;
}

void UI::prev_item()
{
    current_var_index_--;
    if (current_var_index_ < 0)
    {
        current_var_index_ = total_vars_ - 1;
        need_clear_ = true;
    }
    current_page_ = current_var_index_ / vars_per_page_;
}

void UI::next_page()
{
    current_page_++;
    if (current_page_ >= total_pages_)
    {
        current_page_ = 0;
    }
    need_clear_ = true;
    current_var_index_ = current_page_ * vars_per_page_;
}

void UI::prev_page()
{
    current_page_--;
    if (current_page_ < 0)
    {
        current_page_ = total_pages_ - 1;
    }
    need_clear_ = true;
    current_var_index_ = current_page_ * vars_per_page_;
}

void UI::change_mod()
{
    edit_mode_ = !edit_mode_; // 切换模式

    if (edit_mode_)
    {
        // 进入编辑模式
        const DebugVar* current_var = var_ptrs_[current_var_index_];
        // screen_show_string(menu_display_x_ + 12 * 8,
        //                    (current_var_index_ % vars_per_page_) * 16,
        //                    "[" + current_var->get() + "]",
        //                    5);
    }
    else
    {
        // 返回显示模式
        // display_vars();   // 重新显示变量值
        // display_cursor(); // 重新显示光标
        need_clear_ = true; // 切换回显示模式时需要清屏
    }
}
