#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include "Astar_searcher.h"
#include "OccMapTrans.h"

using namespace cv;
using namespace std;

// 地图订阅
ros::Subscriber map_sub;
// 开始节点订阅
ros::Subscriber startPoint_sub;
// 目标节点订阅
ros::Subscriber targetPoint_sub;
// 膨胀地图发布
ros::Publisher mask_pub;
// 路径发布
ros::Publisher path_pub;

// 膨胀地图
nav_msgs::OccupancyGrid OccGridMask;
// A*路径
nav_msgs::Path astr_path;

// A*
JPS::AstarPathFinder astar;
JPS::AstarConfig astarConfig;

// 占用栅格地图参数
OccupancyGridParam OccGridParam;
//      起始点       终止点      
Point startPoint, targetPoint;

/***** 参数 *****/
// 膨胀半径
double InflateRadius;
// 地图标签
bool map_flag;
// 起始点标签
bool startpoint_flag;
// 目标点标签
bool targetpoint_flag;
// 开始标签
bool start_flag;
// 时间
int rate=10;

//-------------------------------- 回调函数 ---------------------------------//
// 地图回调函数
void MapCallback(const nav_msgs::OccupancyGrid& msg)
{
    // 获取参数
    OccGridParam.GetOccupancyGridParam(msg);

    // 获取地图
    int height = OccGridParam.height;
    int width = OccGridParam.width;
    int OccProb;
    Mat Map(height, width, CV_8UC1);
    for(int i=0;i<height;i++)
    {
        for(int j=0;j<width;j++)
        {
            OccProb = msg.data[i * width + j];
            OccProb = (OccProb < 0) ? 100 : OccProb; // set Unknown to 0
            // 地图的原点通常在地图的左下角，而图像的原点通常在左上角，所以通过 height-i-1 进行转换。
            Map.at<uchar>(height-i-1, j) = 255 - round(OccProb * 255.0 / 100.0);
        }
    }

    // 初始化A* 寻路所需
    Mat Mask;// 这里定义一个Mask的原因是为了及那个下面的OccGridMask发布出来，也就是膨胀之后的地图
    astarConfig.InflateRadius = round(InflateRadius / OccGridParam.resolution);
    //Eigen::Vector2d global_xy_l{OccGridParam.x,OccGridParam.y};
    //Eigen::Vector2d global_xy_u{OccGridParam.}
    Eigen::Vector2i w_h{OccGridParam.x,OccGridParam.y};

    astar.initGridMap(OccGridParam.resolution,w_h,OccGridParam.width,OccGridParam.height,Map,Mask,astarConfig);
    //astar.InitAstar(Map, Mask, astrconfig);
    std::cout << "Mask height: " << Mask.cols << Mask.rows << std::endl;
    // Publish Mask
    OccGridMask.header.stamp = ros::Time::now();
    OccGridMask.header.frame_id = "map";
    OccGridMask.info = msg.info;
    OccGridMask.data.clear();
    for(int i=0;i<height;i++)
    {
        for(int j=0;j<width;j++)
        {
            OccProb = Mask.at<uchar>(height-i-1, j) * 255;
            OccGridMask.data.push_back(OccProb);
        }
    }

    // Set flag
    map_flag = true;
    startpoint_flag = false;
    targetpoint_flag = false;
}
// 起始点
void StartPointCallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    Point2d src_point = Point2d(msg.pose.pose.position.x, msg.pose.pose.position.y);
    OccGridParam.Map2ImageTransform(src_point, startPoint);

    // Set flag
    startpoint_flag = true;
    if(map_flag && startpoint_flag && targetpoint_flag)
    {
        start_flag = true;
    }

    ROS_INFO("startPoint: %f %f %d %d", msg.pose.pose.position.x, msg.pose.pose.position.y,
             startPoint.x, startPoint.y);
}
// 终点
void TargetPointtCallback(const geometry_msgs::PoseStamped& msg)
{
    Point2d src_point = Point2d(msg.pose.position.x, msg.pose.position.y);
    OccGridParam.Map2ImageTransform(src_point, targetPoint);

    // Set flag
    targetpoint_flag = true;
    if(map_flag && startpoint_flag && targetpoint_flag)
    {
        start_flag = true;
    }
    ROS_INFO("targetPoint: %f %f %d %d", msg.pose.position.x, msg.pose.position.y,
             targetPoint.x, targetPoint.y);
}

//-------------------------------- Main function ---------------------------------//
int main(int argc, char * argv[])
{
    //  Initial node
    ros::init(argc, argv, "astar");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    ROS_INFO("Start astar node!\n");

    // Initial variables
    map_flag = false;
    startpoint_flag = false;
    targetpoint_flag = false;
    start_flag = false;

    // Parameter
    //nh_priv.param<bool>("Euclidean", astrconfig.Euclidean, true);
    //nh_priv.param<int>("OccupyThresh", astrconfig.OccupyThresh, -1);
    //nh_priv.param<double>("InflateRadius", InflateRadius, -1);
    //nh_priv.param<int>("rate", rate, 10);

    // Subscribe topics
    map_sub = nh.subscribe("map", 10, MapCallback);
    startPoint_sub = nh.subscribe("initialpose", 10, StartPointCallback);
    targetPoint_sub = nh.subscribe("move_base_simple/goal", 10, TargetPointtCallback);

    // Advertise topics
    mask_pub = nh.advertise<nav_msgs::OccupancyGrid>("mask", 1);
    path_pub = nh.advertise<nav_msgs::Path>("jps_path", 10);

    // Loop and wait for callback
    ros::Rate loop_rate(rate);
    while(ros::ok())
    {
        
        if(start_flag)
        {
            // 记录A*寻路时间
            double start_time = ros::Time::now().toSec();
            Eigen::Vector2i startPos{startPoint.x,startPoint.y};
            Eigen::Vector2i targetPos{targetPoint.x,targetPoint.y};
            astar.AstarGraphSearch(startPos, targetPos);

            std::vector<Eigen::Vector2d> PathList=astar.getPath();
            double end_time = ros::Time::now().toSec();
            ROS_INFO("Astar Use %f s", end_time - start_time);

            // A* path
            if(!PathList.empty())
            {
                astr_path.header.stamp = ros::Time::now();
                astr_path.header.frame_id = "map";
                astr_path.poses.clear();
                for(int i=0;i<PathList.size();i++)
                {
                    // Point2d dst_point;
                    // OccGridParam.Image2MapTransform(PathList[i], dst_point);

                    geometry_msgs::PoseStamped pose_stamped;
                    pose_stamped.header.stamp = ros::Time::now();
                    pose_stamped.header.frame_id = "map";
                    pose_stamped.pose.position.x = PathList[i].x();
                    pose_stamped.pose.position.y = PathList[i].y();
                    pose_stamped.pose.position.z = 0;
                    astr_path.poses.push_back(pose_stamped);
                }
                path_pub.publish(astr_path);
            }
            else
            {
                ROS_ERROR("Can not find a valid path");
            }


            // Set flag
            start_flag = false;
        }
        
        if(map_flag)
        {
            mask_pub.publish(OccGridMask);
        }

        loop_rate.sleep();
        ros::spinOnce();
    }


    return 0;
}
