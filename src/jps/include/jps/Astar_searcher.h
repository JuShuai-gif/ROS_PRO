#pragma once
#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include "math.h"
#include "node.h"
#include <opencv2/opencv.hpp>

using namespace cv;

namespace JPS{

// 配置
struct AstarConfig{
    bool Euclidean;         // true/false
    int OccupyThresh;       // 0~255
    int InflateRadius;      // integer

    AstarConfig(bool _Euclidean = true, int _OccupyThresh = -1, int _InflateRadius = -1):
        Euclidean(_Euclidean), OccupyThresh(_OccupyThresh), InflateRadius(_InflateRadius)
    {
    }
};


class AstarPathFinder
{
private:
    /* data */
protected:
    uint8_t* data;
    GridNodePtr** GridNodeMap;
    // 目标索引
    Eigen::Vector2i goalIdx;
    int GLX_SIZE,GLY_SIZE;
    int GLXY_SIZE;

    // 分辨率
    Eigen::Vector2i originPt;
    double resolution,inv_resolution;

    Mat Map;//原始地图数据
    Mat LabelMap;// 膨胀之后的数据
    AstarConfig config;

    GridNodePtr startPtr;
    GridNodePtr endPtr;
    GridNodePtr terminatePtr;
    std::multimap<double,GridNodePtr> openSet;
    
    double getDiagHeu(GridNodePtr node1,GridNodePtr node2);
    double getHeu(GridNodePtr node1, GridNodePtr node2);
    double getEuclHeu(GridNodePtr node1,GridNodePtr node2);

    double getManhHeu(GridNodePtr node1,GridNodePtr node2);

	void AstarGetSucc(GridNodePtr currentPtr, std::vector<GridNodePtr> & neighborPtrSets, std::vector<double> & edgeCostSets);		

	bool isOccupied(const Eigen::Vector2i & index) const;
	bool isFree(const Eigen::Vector2i & index) const;
		
        // 位置转索引
    inline int point2index(Point point) {
        return point.y * Map.cols + point.x;
    }
    // 索引转位置
    inline Point index2point(int index) {
        return Point(int(index / Map.cols), index % Map.cols);
    }

    Eigen::Vector2d gridIndex2coord(const Eigen::Vector2i & index) 
    {
        Eigen::Vector2d pt;

        pt(0) = index(0) * resolution + originPt.x();
        pt(1) = (GLY_SIZE - 1 - index(1)) * resolution + originPt.y();

        return pt;
    }

    Eigen::Vector2i coord2gridIndex(const Eigen::Vector2d & pt) 
    {
        Eigen::Vector2i idx;
        idx <<  round((pt(0) - originPt.x()) * inv_resolution),
                GLY_SIZE - 1 - round((pt(1) - originPt.y()) * inv_resolution);                  
    
        return idx;
    }

public:
    AstarPathFinder(){};
	~AstarPathFinder(){};
	void AstarGraphSearch(Eigen::Vector2i start_idx, Eigen::Vector2i end_idx);
	void resetGrid(GridNodePtr ptr);
	void resetUsedGrids();

	void initGridMap(double _resolution, Eigen::Vector2i pos,int max_x_id, int max_y_id,Mat& _Map, Mat& Mask, AstarConfig _config);

	Eigen::Vector2d coordRounding(const Eigen::Vector2d & coord);
	std::vector<Eigen::Vector2d> getPath();
	std::vector<Eigen::Vector2d> getVisitedNodes();
};


}




