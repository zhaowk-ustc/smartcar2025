#ifndef POINT_H
#define POINT_H

struct Point
{
    int x, y;
    Point() : x(0), y(0) {}
    Point(int x_, int y_) : x(x_), y(y_) {}
};


#endif // POINT_H