#include "zf_common_headfile.h"

struct Point
{
    int16 x; // x坐标
    int16 y; // y坐标

    Point operator+(const Point& other) const
    {
        return { int16(x + other.x), int16(y + other.y) };
    }
    Point operator-(const Point& other) const
    {
        return { int16(x - other.x), int16(y - other.y) };
    }
};

class Image
{
public:
    // 构造函数
    Image(uint8* data, uint16 width, uint16 height)
        : data_(data), width_(width), height_(height)
    {
    }

    // 获取图像数据指针
    uint8* get_data() const { return data_; }

    // 获取图像宽度
    uint16 get_width() const { return width_; }

    // 获取图像高度
    uint16 get_height() const { return height_; }

private:
    uint8* data_; // 图像数据指针
    uint16 width_; // 图像宽度
    uint16 height_; // 图像高度
};

