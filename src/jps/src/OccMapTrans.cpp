#include "OccMapTrans.h"

void OccupancyGridParam::GetOccupancyGridParam(nav_msgs::OccupancyGrid OccGrid)
{
    // 获取参数
    resolution = OccGrid.info.resolution;
    height = OccGrid.info.height;
    width = OccGrid.info.width;
    x = OccGrid.info.origin.position.x;
    y = OccGrid.info.origin.position.y;

    // 位姿
    double roll, pitch, yaw;
    geometry_msgs::Quaternion q = OccGrid.info.origin.orientation;
    tf::Quaternion quat(q.x, q.y, q.z, q.w); // x, y, z, w
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    theta = yaw;
    // Calculate R, t
    R = Mat::zeros(2,2, CV_64FC1);
    R.at<double>(0, 0) = resolution * cos(theta);
    R.at<double>(0, 1) = resolution * sin(-theta);
    R.at<double>(1, 0) = resolution * sin(theta);
    R.at<double>(1, 1) = resolution * cos(theta);
    t = Mat(Vec2d(x, y), CV_64FC1);

    Point2d dst_point{};
    Point src_point = Point(width, 0);
    Image2MapTransform(src_point,dst_point);
    coordHeight = dst_point.y - y;
    coordWidth = dst_point.x - x;

    //ROS_INFO("resolution: %f,height: %d,width: %d,x: %f,y: %f,coordHeight: %d,coordWidth:%d ",
    //                resolution,height,width,x,y,coordHeight,coordWidth);
    //ROS_INFO("height: %d,width: %d,x: %f,y: %f,coordHeight: %d,coordWidth:%d ",
    //                height,width,x,y,coordHeight,coordWidth);
}

// 图像到地图变换
void OccupancyGridParam::Image2MapTransform(Point& src_point, Point2d& dst_point)
{
    // Upside down
    Mat P_src = Mat(Vec2d(src_point.x, height - 1 - src_point.y), CV_64FC1);
    // Rotate and translate
    Mat P_dst = R * P_src + t;

    dst_point.x = P_dst.at<double>(0, 0);
    dst_point.y = P_dst.at<double>(1, 0);
}

// 地图到图像变换
void OccupancyGridParam::Map2ImageTransform(Point2d& src_point, Point& dst_point)
{
    Mat P_src = Mat(Vec2d(src_point.x, src_point.y), CV_64FC1);
    //std::cout<< "P_src: \n" << P_src << std::endl;
    //std::cout<< "R.inv(): \n" << R.inv() << std::endl;
    //std::cout<< "t: \n" << t << std::endl;
    Mat P_dst = R.inv() * (P_src - t);
    //std::cout<< "P_dst: \n" << P_dst << std::endl;
    // Upside down
    dst_point.x = round(P_dst.at<double>(0, 0));
    dst_point.y = height - 1 - round(P_dst.at<double>(1, 0));
}