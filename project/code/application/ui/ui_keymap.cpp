#include "ui.h"

UI::Key UI::get_key_input()
{
    // 实现具体的按键读取逻辑
    uint16 voltage;
    voltage = adc_convert(ADC0_CH00_P06_0);

    if (voltage <= 18)
    { // voltage == 0
        return Key::LEFT;
    }
    else if (19 <= voltage and voltage <= 60)
    { // voltage == 36
        return Key::UP;
    }
    else if (61 <= voltage and voltage <= 105)
    { // voltage == 83
        return Key::DOWN;
    }
    else if (106 <= voltage and voltage <= 156)
    { // voltage == 127
        return Key::RIGHT;
    }
    else if (157 <= voltage and voltage <= 220)
    { // voltage == 185
        return Key::ENTER;
    }
    // else if(221 <= voltage and voltage <= 255){ //voltage == 255
    //     return Key::NONE;
    // }
    else
    { // voltage == 255
        return Key::NONE;
    }

    // return Key::NONE;
}

void UI::update_key_state()
{
    // 读取当前按键状态并更新 current_key_
    auto key = get_key_input();
    
    if (key != Key::NONE and key != pre_key_)
    {
        current_key_ = key;
    }
    else
    {
        current_key_ = Key::NONE;
    }

    pre_key_ = key;
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
