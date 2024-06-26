#pragma once
#include "polygon_base.h"
#include <cmath>

namespace polygon_plugins
{

    class Triangle : public polygon_base::RegularPolygon
    {
    public:
        Triangle() : side_length_() {}

        // 初始化边长
        void initialize(double side_length)
        {
            side_length_ = side_length;
        }

        double area()
        {
            return 0.5 * side_length_ * getHeight();
        }

        // Triangle类自己的接口
        double getHeight()
        {
            return sqrt((side_length_ * side_length_) - ((side_length_ / 2) * (side_length_ / 2)));
        }

    private:
        double side_length_;
    };
}