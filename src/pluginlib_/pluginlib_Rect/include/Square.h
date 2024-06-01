#pragma once
#include "polygon_base.h"
#include <cmath>

namespace polygon_plugins
{
    class Square : public polygon_base::RegularPolygon
    {
    public:
        Square() : side_length_() {}

        // 初始化边长
        void initialize(double side_length)
        {
            side_length_ = side_length;
        }

        double area()
        {
            return side_length_ * side_length_;
        }

    private:
        double side_length_;
    };

};