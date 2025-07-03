#include "zf_common_headfile.h"
#include "ui.h"
#include <iostream>

UI::UI()
{
    setup_debug_vars();
}

void UI::init()
{
    ips114_init();

}

void UI::update_mainloop()
{
    if (!enabled_)
        return;

    handle_current_key();
    display_menu();
}

void UI::update_pit()
{
    if (!enabled_)
        return;

    update_key_state();
}

void UI::import_vars(const vector<const DebugVar*>& var_ptrs)
{
    var_ptrs_ = var_ptrs;
    total_vars_ = var_ptrs_.size();
    total_pages_ = (total_vars_ + vars_per_page_ - 1) / vars_per_page_;
}

void UI::enable()
{
    enabled_ = true;
}

void UI::disable()
{
    enabled_ = false;
}

string& ui_test_get()
{
    static string test_var = "UI Test Variable";
    return test_var;
}

void UI::setup_debug_vars()
{
    add_debug_var("switch_camera", make_function_var("switch_camera", [this] { select_camera_display_mode(); }));
}