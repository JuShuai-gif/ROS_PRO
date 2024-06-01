#pragma once
#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>

#define inf 1>>20
struct GridNode;
typedef GridNode* GridNodePtr;

struct GridNode
{
    int id;                 // 1 开放集合   -1 闭合集合
    Eigen::Vector2d coord;  // 坐标
    Eigen::Vector2i dir;    // 扩展的方向
    Eigen::Vector2i index;  // 索引

    double gScore,fScore;
    GridNodePtr parent;
    std::multimap<double,GridNodePtr>::iterator nodeMapIt;

    GridNode(Eigen::Vector2i _index,Eigen::Vector2d _coord){
        id = 0;
        index = _index;
        coord = _coord;
        dir = Eigen::Vector2i::Zero();

        gScore = inf;
        fScore = inf;
        parent = NULL;
    }

    GridNode(){};
    ~GridNode(){};
};

