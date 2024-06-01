#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>

using namespace grid_map;

int main(int argc, char** argv)
{
  // 初始化一个名称为grid_map_simple_demo的节点和发布者
  ros::init(argc, argv, "grid_map_simple_demo");
  ros::NodeHandle nh("~");
  ros::Publisher publisher = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

  // 创建一个grid_map 这个grid_map里含有一个名称为elevation的层
  // 可以一次性创建多个层，GridMap map({"elevation","normal_x","normal_y"});
  GridMap map({"elevation"});
  // 设置grid_map的坐标系为map，这里相当于在map坐标系下生成了一个grid_map
  map.setFrameId("map");
  // 设置grid_map的大小，这里是1.2m 2.0m 每个格子的大小为3cm
  // 如果出现2.0/0.03不为整数的情况下，那么grid_map会自动填充一个格子，保证被整除
  // 这里可以设置地图的中心，默认情况下地图的中心和上一步设置的坐标系重合
  map.setGeometry(Length(4.0, 4.0), 0.1);
  ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
    map.getLength().x(), map.getLength().y(),
    map.getSize()(0), map.getSize()(1));

  grid_map::Polygon polygon;
  polygon.addVertex(grid_map::Position(1.0,1.0));
  polygon.addVertex(grid_map::Position(1.0,1.0));
  polygon.addVertex(grid_map::Position(1.0,1.0));
  polygon.addVertex(grid_map::Position(1.0,1.0));
  
  
  // Work with grid map in a loop.
  ros::Rate rate(30.0);
  while (nh.ok()) {

    // Add data to grid map.
    ros::Time time = ros::Time::now();
    
    for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
      Position position;
      map.getPosition(*it, position);
      // 根据指针*it访问
      map.at("elevation", *it) = -0.04 + 2 * std::sin(3.0 * time.toSec() + 5.0 * position.y()) * position.x();
    }

    // Publish grid map.
    map.setTimestamp(time.toNSec());
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(map, message);
    publisher.publish(message);
    ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());

    // Wait for next cycle.
    rate.sleep();
  }

  return 0;
}
