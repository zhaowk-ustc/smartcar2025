#ifndef POINT_H
#define POINT_H

// 8邻域方向
const int16_t dx[8] = { 1, 1, 0, -1, -1, -1, 0, 1 };
const int16_t dy[8] = { 0, 1, 1, 1, 0, -1, -1, -1 };

#include <vector>
#include <complex>
using namespace std;

template<typename T>
struct Point2_ : public complex<T>
{
    using complex<T>::complex;
    Point2_() : complex<T>(0, 0) {}
    T x() const { return this->real(); }
    T y() const { return this->imag(); }
};

struct Point : public Point2_<int16_t>
{
    Point() : Point2_<int16_t>() {}
    Point(int16_t x_, int16_t y_) : Point2_<int16_t>(x_, y_) {}

    vector<Point> neighbors() const
    {
        vector<Point> result;
        for (int d = 0; d < 8; ++d)
        {
            result.emplace_back(x() + dx[d], y() + dy[d]);
        }
        return result;
    }
};

const Point NULL_POINT = Point(INT16_MAX, INT16_MAX);

using Point2f = Point2_<float>;



#endif // POINT_H