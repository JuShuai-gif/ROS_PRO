#ifndef WAYPOINT_RVIZ_PLUGIN_WAYPOINT_TOOL_H
#define WAYPOINT_RVIZ_PLUGIN_WAYPOINT_TOOL_H

#include <sstream>
#include <ros/ros.h>
#include <QObject>

#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>

#include "rviz/display_context.h"
#include "rviz/properties/string_property.h"
#include "rviz/default_plugin/tools/pose_tool.h"

namespace rviz
{
class StringProperty;

class WaypointTool : public PoseTool
{
  Q_OBJECT
public:
  WaypointTool();
  virtual ~WaypointTool()
  {
  }
  // 初始化
  virtual void onInitialize();

protected:
  // 里程计处理
  virtual void odomHandler(const nav_msgs::Odometry::ConstPtr& odom);
  // 位置设置
  virtual void onPoseSet(double x, double y, double theta);

private Q_SLOTS:
  // 更新话题 
  void updateTopic();

private:
  float vehicle_z;
  // 节点处理句柄
  ros::NodeHandle nh_;
  // 订阅
  ros::Subscriber sub_;
  // 发布
  ros::Publisher pub_;
  // 发布
  ros::Publisher pub_joy_;

  StringProperty* topic_property_;
};
}

#endif  // WAYPOINT_RVIZ_PLUGIN_WAYPOINT_TOOL_H
