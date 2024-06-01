#ifndef OCCMAPTRANSFORM_H
#define OCCMAPTRANSFORM_H

#include <iostream>
#include <cmath>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <opencv2/opencv.hpp>


using namespace std;
using namespace cv;


class OccupancyGridParam{

public: 
    // 获取占据网格参数
    void GetOccupancyGridParam(nav_msgs::OccupancyGrid OccGrid);
    // 图像转地图
    void Image2MapTransform(Point& src_point, Point2d& dst_point);
    // 地图转图像
    void Map2ImageTransform(Point2d& src_point, Point& dst_point);

private: 

public: 
    // 分辨率
    double resolution;
    // 高度
    int height;
    // 宽度
    int width;
    // 原始位置
    double x;
    double y;
    double theta;

private: 
    // 变换
    Mat R;// 旋转
    Mat t;// 位移

};

#endif //OCCMAPTRANSFORM_H
