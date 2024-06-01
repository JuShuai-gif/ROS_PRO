#include "waypoint_rviz_plugin/waypoint_tool.h"

namespace rviz
{
WaypointTool::WaypointTool()
{
  // 快捷键
  shortcut_key_ = 'w';
  // 话题属性
  topic_property_ = new StringProperty("Topic", "waypoint", "The topic on which to publish navigation waypionts.",
                                       getPropertyContainer(), SLOT(updateTopic()), this);
}
// 初始化
void WaypointTool::onInitialize()
{
  PoseTool::onInitialize();
  setName("Waypoint");
  updateTopic();
  vehicle_z = 0;
}

void WaypointTool::updateTopic()
{
  // 订阅估计位姿，为了下面设置位置点的时候和位姿平面保持同一高度
  sub_ = nh_.subscribe<nav_msgs::Odometry> ("/state_estimation", 5, &WaypointTool::odomHandler, this);
  pub_ = nh_.advertise<geometry_msgs::PointStamped>("/way_point", 5);
  pub_joy_ = nh_.advertise<sensor_msgs::Joy>("/joy", 5);
}

void WaypointTool::odomHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
  ROS_INFO("接收到Odom消息!!!");
  vehicle_z = odom->pose.pose.position.z;
}

void WaypointTool::onPoseSet(double x, double y, double theta)
{
  ROS_INFO("onPoseSet!!!");
  sensor_msgs::Joy joy;

  joy.axes.push_back(0);
  joy.axes.push_back(0);
  joy.axes.push_back(-1.0);
  joy.axes.push_back(0);
  joy.axes.push_back(1.0);
  joy.axes.push_back(1.0);
  joy.axes.push_back(0);
  joy.axes.push_back(0);

  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(1);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);

  joy.header.stamp = ros::Time::now();
  joy.header.frame_id = "waypoint_tool";
  pub_joy_.publish(joy);

  geometry_msgs::PointStamped waypoint;
  waypoint.header.frame_id = "map";
  waypoint.header.stamp = joy.header.stamp;
  waypoint.point.x = x;
  waypoint.point.y = y;
  waypoint.point.z = vehicle_z;

  pub_.publish(waypoint);
  usleep(10000);
  pub_.publish(waypoint);
}
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::WaypointTool, rviz::Tool)
