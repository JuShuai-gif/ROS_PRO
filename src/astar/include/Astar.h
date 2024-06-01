#ifndef ASTAR_H
#define ASTAR_H

#include <iostream>
#include <queue>
#include <unordered_map>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

namespace pathplanning{

enum NodeType{
    obstacle = 0,// 是否占据
    free,
    inOpenList,
    inCloseList
};

struct Node{
    Point point;  // 节点坐标
    int F, G, H;  // F = G + H   G：起始格子到格子n的实际代价  H：格子n到终点格子的估计代价
    Node* parent; // 父节点

    Node(Point _point = Point(0, 0)):point(_point), F(0), G(0), H(0), parent(NULL)
    {
    }
};

// 这里可以进行优化，如果F相等的情况，可以通过比较H的值
struct cmp
{
    bool operator() (pair<int, Point> a, pair<int, Point> b) // Comparison function for priority queue
    {
        return a.first > b.first; // min heap
    }
};

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

class Astar{
public:
    // 初始化函数
    void InitAstar(Mat& _Map, AstarConfig _config = AstarConfig());
    void InitAstar(Mat& _Map, Mat& Mask, AstarConfig _config = AstarConfig());
    // 路径规划
    void PathPlanning(Point _startPoint, Point _targetPoint, vector<Point>& path);
    // 绘制路径
    void DrawPath(Mat& _Map, vector<Point>& path, InputArray Mask = noArray(), Scalar color = Scalar(0, 0, 255),
            int thickness = 1, Scalar maskcolor = Scalar(255, 255, 255));
    // 位置转索引
    inline int point2index(Point point) {
        return point.y * Map.cols + point.x;
    }
    // 索引转位置
    inline Point index2point(int index) {
        return Point(int(index / Map.cols), index % Map.cols);
    }

private:
    void MapProcess(Mat& Mask);
    Node* FindPath();
    void GetPath(Node* TailNode, vector<Point>& path);

private:
    //Object
    Mat Map;
    Point startPoint, targetPoint;
    Mat neighbor;

    Mat LabelMap;
    AstarConfig config;

    priority_queue<pair<int, Point>, vector<pair<int, Point>>, cmp> OpenList; // open list
    unordered_map<int, Node*> OpenDict; // open dict
    vector<Node*> PathList;  // path list
};

}




#endif //ASTAR_H
