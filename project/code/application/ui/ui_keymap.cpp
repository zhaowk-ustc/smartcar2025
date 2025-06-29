#include "ui.h"

void UI::update_key_state()
{
    // // 读取当前按键状态并更新 current_key_
    // char key = get_key_input();
    // switch (key) {
    //     case 'w':
    //         current_key_ = Key::UP;
    //         break;
    //     case 's':
    //         current_key_ = Key::DOWN;
    //         break;
    //     case 'a':
    //         current_key_ = Key::LEFT;
    //         break;
    //     case 'd':
    //         current_key_ = Key::RIGHT;
    //         break;
    //     case 'e':
    //         current_key_ = Key::ENTER;
    //         break;
    //     default:
    //         current_key_ = Key::NONE;
    //         break;
    // }
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
