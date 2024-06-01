#pragma once
#include "PathPlan.h"
#include <iostream>

namespace plan_plugins
{
    class Astr : public plan_base::PathPlan
    {
    public:
        Astr() : name_() {}

        void initialize(std::string name)
        {
            name_ = name;
        }

        void getPlan()
        {
            std::cout << "getPlan" << std::endl;
        }

        void setGoal()
        {
            std::cout << "setGoal" << std::endl;
        }

        std::string path_name()
        {
            return name_;
        }

    private:
        std::string name_{};
    };
}
