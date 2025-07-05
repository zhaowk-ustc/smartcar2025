#include "ui.h"
#include <algorithm>
#include <cmath>
#include "middleware/vision/road_element.h"
using namespace std;

void UI::screen_show_string(const uint16 x, const uint16 y, const string& str, const uint8 len)
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
    display_camera();
    if (camera_display_mode_ == CameraDisplayMode::CALIBRATED_BINARIZED)
        display_overlay(); // 显示拓扑图叠加层
}

void UI::display_vars()
{
    int start_index = current_page_ * vars_per_page_;
    int end_index = min(start_index + vars_per_page_, total_vars_);

    for (int i = start_index; i < end_index; ++i)
    {
        const DebugVar* var = var_ptrs_[i];
        screen_show_string(menu_display_x_ + 1 * 8, (i - start_index) * 16, var->name, 9);
        screen_show_string(menu_display_x_ + 12 * 8, (i - start_index) * 16, var->get(), 5);
    }
}

void UI::display_cursor()
{
    int cursor_y = (current_var_index_ % vars_per_page_) * 16;
    screen_show_string(menu_display_x_, cursor_y, ">");
}

void UI::display_page_info()
{
    string page_info = "Page " + to_string(current_page_ + 1) + "/" + to_string(total_pages_);
    screen_show_string(menu_display_x_, vars_per_page_ * 16, page_info);
}

#include "multicore/core_shared.h"

void UI::select_camera_display_mode()
{
    if (camera_display_mode_ == CameraDisplayMode::ORIGINAL)
    {
        camera_display_mode_ = CameraDisplayMode::CALIBRATED;
    }
    else if (camera_display_mode_ == CameraDisplayMode::CALIBRATED)
    {
        camera_display_mode_ = CameraDisplayMode::CALIBRATED_BINARIZED;
    }
    else
    {
        camera_display_mode_ = CameraDisplayMode::ORIGINAL;
    }
    ips114_clear();
}

void UI::display_camera()
{
    extern uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];
    extern uint8 calibrated_image[49 * 80];
    extern uint8 calibrated_binary_image[49 * 80];

    const uint8* frame_buffer = nullptr;
    uint16 width = 0;
    uint16 height = 0;

    switch (camera_display_mode_)
    {
        case CameraDisplayMode::ORIGINAL:
            frame_buffer = &mt9v03x_image[0][0];
            width = MT9V03X_W;
            height = MT9V03X_H;
            // 刷新缓存，确保从内存重新读取图像数据
            SCB_InvalidateDCache_by_Addr((void*)frame_buffer, width * height);
            // 显示缩放2倍
            ips114_show_gray_image(camera_display_x_, 0, frame_buffer, width, height, width / 2, height / 2, 0);
            break;
        case CameraDisplayMode::CALIBRATED:
            frame_buffer = calibrated_image;
            width = 80;
            height = 49;
            SCB_InvalidateDCache_by_Addr((void*)frame_buffer, width * height);
            // 直接显示原始大小
            ips114_show_gray_image(camera_display_x_, 0, frame_buffer, width, height, width, height, 0);
            break;
        case CameraDisplayMode::CALIBRATED_BINARIZED:
            frame_buffer = calibrated_binary_image;
            width = 80;
            height = 49;
            SCB_InvalidateDCache_by_Addr((void*)frame_buffer, width * height);
            // 直接显示原始大小
            ips114_show_gray_image(camera_display_x_, 0, frame_buffer, width, height, width, height, 0);
            break;
        default:
            break;
    }
}

void UI::display_overlay()
{
    int line_count = calibrated_height;
    SCB_InvalidateDCache_by_Addr((void*)&vision_debug_shared, sizeof(vision_debug_shared));
    auto left_bound = vision_debug_shared.left_bounds;
    auto middle_bound = vision_debug_shared.line_points;
    auto right_bound = vision_debug_shared.right_bounds;

    for (int i = 0; i < line_count; ++i)
    {
        auto l = camera_display_x_ + left_bound[i];
        auto m = camera_display_x_ + middle_bound[i];
        auto r = camera_display_x_ + right_bound[i];
        ips114_draw_point(l, i, RGB565_RED);
        ips114_draw_point(l + 1, i, RGB565_RED);
        ips114_draw_point(l + 2, i, RGB565_RED);

        ips114_draw_point(r, i, RGB565_BLUE);
        ips114_draw_point(r - 1, i, RGB565_BLUE);
        ips114_draw_point(r - 2, i, RGB565_BLUE);

        ips114_draw_point(m, i, RGB565_GREEN);
    }
    ips114_show_string(camera_display_x_, 50, "Element:");
    screen_show_string(camera_display_x_, 50 + 16, road_element_type_to_string(vision_debug_shared.element_type), 8);
    ips114_show_string(camera_display_x_, 50 + 16 * 2, "pos:");
    ips114_show_int(camera_display_x_ + 8 * 5, 50 + 16 * 2, vision_debug_shared.element_position, 3);
    ips114_show_string(camera_display_x_, 50 + 16 * 3, "conf:");
    ips114_show_float(camera_display_x_ + 8 * 5, 50 + 16 * 3, vision_debug_shared.element_confidence, 3, 2);
}