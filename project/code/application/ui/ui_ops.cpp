#include "ui.h"

void UI::next_item()
{
    current_var_index_++;
    if (current_var_index_ >= total_vars_)
    {
        current_var_index_ = 0;
    }
    current_page_ = current_var_index_ / vars_per_page_;
}

void UI::prev_item()
{
    current_var_index_--;
    if (current_var_index_ < 0)
    {
        current_var_index_ = total_vars_ - 1;
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
    current_var_index_ = current_page_ * vars_per_page_;
}

void UI::prev_page()
{
    current_page_--;
    if (current_page_ < 0)
    {
        current_page_ = total_pages_ - 1;
    }
    current_var_index_ = current_page_ * vars_per_page_;
}