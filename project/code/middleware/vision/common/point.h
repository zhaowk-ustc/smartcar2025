#ifndef POINT_H
#define POINT_H

// 8邻域方向
const int16_t dx[8] = { 1, 1, 0, -1, -1, -1, 0, 1 };
const int16_t dy[8] = { 0, 1, 1, 1, 0, -1, -1, -1 };

#include <vector>
using namespace std;

template<typename T>
struct Point2_
{
    T x, y;
    Point2_() : x(0), y(0) {}
    Point2_(T x_, T y_) : x(x_), y(y_) {}

};

using Point2s = Point2_<int16_t>;

struct Point : public Point2s
{
    Point() : Point2s() {}
    Point(int16_t x_, int16_t y_) : Point2s(x_, y_) {}

    vector<Point> neighbors() const
    {
        vector<Point> result;
        for (int d = 0; d < 8; ++d)
        {
            result.emplace_back(x + dx[d], y + dy[d]);
        }
        return result;
    }

    bool operator==(const Point& other) const
    {
        return x == other.x && y == other.y;
    }

    bool operator!=(const Point& other) const
    {
        return !(*this == other);
    }
};

const Point NULL_POINT = Point(INT16_MAX, INT16_MAX);

using Point2i = Point2_<int>;
using Point2f = Point2_<float>;



#endif // POINT_H