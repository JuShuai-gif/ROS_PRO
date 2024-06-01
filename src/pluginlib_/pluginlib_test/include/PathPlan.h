#pragma once
#include <string>

namespace plan_base{
    class PathPlan
    {
    public:
        virtual void initialize(std::string name) = 0;
        virtual std::string path_name() = 0;
        virtual ~PathPlan(){};

    public:
        PathPlan(){}
    };

}