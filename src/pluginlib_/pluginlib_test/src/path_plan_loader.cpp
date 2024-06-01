#include <boost/shared_ptr.hpp>

#include <pluginlib/class_loader.h>
#include "PathPlan.h"

int main(int argc, char **argv)
{
    std::cout << "1" << std::endl;
    // 创建一个ClassLoader，用来加载plugin
    pluginlib::ClassLoader<plan_base::PathPlan> poly_loader("pluginlib_test", "plan_base::PathPlan");
    std::cout << "1" << std::endl;
    try
    {
        std::cout << "1" << std::endl;
        // 加载Triangle插件类，路径在polygon_plugins.xml中定义
        boost::shared_ptr<plan_base::PathPlan> astr = poly_loader.createInstance("pluginlib_test/plan_astr");
        std::cout << "1" << std::endl;
        // 初始化边长
        astr->initialize("plan_astr");
        
        std::cout << astr->path_name() << std::endl;
        
    }
    catch (pluginlib::PluginlibException &ex)
    {
        ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
    }

    try
    {
        boost::shared_ptr<plan_base::PathPlan> dijkstra = poly_loader.createInstance("pluginlib_test/plan_dijkstra");
        dijkstra->initialize("plan_dijkstra");

        std::cout << dijkstra->path_name() << std::endl;

        dijkstra->path_name();
    }
    catch(pluginlib::PluginlibException& ex)
    {
        ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
    }

    return 0;
}