#include "ui.h"

UI::Key UI::get_key_input()
{
    // 实现具体的按键读取逻辑

    return Key::NONE;
}

void UI::update_key_state()
{
    // 读取当前按键状态并更新 current_key_
    auto key = get_key_input();
    if (key != Key::NONE)
    {
        current_key_ = key;
    }
}

void UI::handle_current_key()
{
    // 根据 current_key_ 调用相应的处理函数
    switch (current_key_)
    {
        case Key::UP:
            on_key_up();
            break;
        case Key::DOWN:
            on_key_down();
            break;
        case Key::LEFT:
            on_key_left();
            break;
        case Key::RIGHT:
            on_key_right();
            break;
        case Key::ENTER:
            on_key_enter();
            break;
        default:
            break;
    }

    current_key_ = Key::NONE; // 处理完后重置按键状态
}

void UI::on_key_up()
{
    prev_item();
}

void UI::on_key_down()
{
    next_item();
}

void UI::on_key_left()
{
    prev_page();
}

void UI::on_key_right()
{
    next_page();
}

void UI::on_key_enter()
{
    // 处理确认操作
}
