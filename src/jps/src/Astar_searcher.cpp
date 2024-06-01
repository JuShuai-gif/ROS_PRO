#include "Astar_searcher.h"

using namespace std;
using namespace Eigen;
using namespace JPS;

void AstarPathFinder::initGridMap(double _resolution,Vector2i pos,int max_x_id, int max_y_id,Mat& Map_, Mat& Mask, AstarConfig _config)
{   
    /*
    // 地图下界
    gl_xl = global_xy_l(0);
    gl_yl = global_xy_l(1);
    // 地图上界
    gl_xu = global_xy_u(0);
    gl_yu = global_xy_u(1);
    */
    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    std::cout << "GLX_SIZE: " << GLX_SIZE << GLY_SIZE << std::endl;
    GLXY_SIZE  = GLX_SIZE * GLY_SIZE;
    originPt = pos;

    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;    
/*
    data = new uint8_t[GLXY_SIZE];
    // 内存分配
    memset(data, 0, GLXY_SIZE * sizeof(uint8_t));
    // 分配栅格地图
    GridNodeMap = new GridNodePtr * [GLX_SIZE];
    for(int i = 0; i < GLX_SIZE; i++){
        GridNodeMap[i] = new GridNodePtr[GLY_SIZE];
        for(int j = 0; j < GLY_SIZE; j++){
            Vector2i tmpIdx(i,j);
            Vector2d pos = gridIndex2coord(tmpIdx);
            GridNodeMap[i][j] = new GridNode(tmpIdx, pos);
        }
    }
*/
    Map = Map_;
    config = _config;

    int width = Map.cols;
    int height = Map.rows;
    Mat _Map = Map.clone();

    // 转为灰度图
    if(_Map.channels() == 3)
    {
        cvtColor(_Map.clone(), _Map, cv::COLOR_BGR2GRAY);
    }

    // 二值化 将灰度图转换为二值图像，其中像素值为 0 或 255
    if(config.OccupyThresh < 0)
    {
        threshold(_Map.clone(), _Map, 0, 255, cv::THRESH_OTSU);
    } else
    {
        threshold(_Map.clone(), _Map, config.OccupyThresh, 255, cv::THRESH_BINARY);
    }

    // 膨胀
    Mat src = _Map.clone();
    if(config.InflateRadius > 0)
    {
        Mat se = getStructuringElement(MORPH_ELLIPSE, Size(2 * config.InflateRadius, 2 * config.InflateRadius));
        erode(src, _Map, se);
    }

    // src是膨胀之后的地图 _Map是未膨胀地图  Mask就是膨胀的那个圈
    bitwise_xor(src, _Map, Mask);

    // Initial LabelMap（该地图就是膨胀之后的地图，也就是需要扩充的地图）
    LabelMap = Mat::zeros(height, width, CV_8UC1);
    for(int y=0;y<height;y++)
    {
        for(int x=0;x<width;x++)
        {
            if(_Map.at<uchar>(y, x) == 0)
            {   // 障碍物
                LabelMap.at<uchar>(y, x) = 0;
            }
            else
            {   // 空闲
                LabelMap.at<uchar>(y, x) = 1;
            }
        }
    }
}

// 重置网格
void AstarPathFinder::resetGrid(GridNodePtr ptr)
{
    ptr->id = 0;
    ptr->parent = NULL;
    ptr->gScore = inf;
    ptr->fScore = inf;
}

// 重置使用过的网格
void AstarPathFinder::resetUsedGrids()
{   
    for(int i=0; i < GLX_SIZE ; i++)
        for(int j=0; j < GLY_SIZE ; j++)
            resetGrid(GridNodeMap[i][j]);
}

// 获取可视化节点
vector<Vector2d> AstarPathFinder::getVisitedNodes()
{   
    vector<Vector2d> visited_nodes;
    for(int i = 0; i < GLX_SIZE; i++)
        for(int j = 0; j < GLY_SIZE; j++) 
                //if(GridNodeMap[i][j][k]->id != 0) // visualize all nodes in open and close list
                if(GridNodeMap[i][j]->id == -1)  // visualize nodes in close list only
                    visited_nodes.push_back(GridNodeMap[i][j]->coord);
    ROS_WARN("visited_nodes size : %d", visited_nodes.size());
    return visited_nodes;
}

inline bool AstarPathFinder::isOccupied(const Eigen::Vector2i & index) const
{
    if (LabelMap.at<uchar>(index.y(), index.x())==0)
        return true;
    else
        return false;
}

inline bool AstarPathFinder::isFree(const Eigen::Vector2i & index) const
{
    if (LabelMap.at<uchar>(index.y(), index.x())==1)
        return true;
    else
        return false;
}


// 获取当前节点周围的 nerigbor,并计算每个 nerignor 的 fscore
inline void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr, vector<GridNodePtr> & neighborPtrSets, vector<double> & edgeCostSets)
{   
    Vector2d coord_nerigbor;
    Vector2i idx_nerigbor;

    neighborPtrSets.clear();
    edgeCostSets.clear();
    
    static int count = 0;
    count++;
    // 计算总共寻找多少次邻居
    //ROS_INFO("Iteration Counts %d",count);

    for (size_t dx = -1; dx < 2; ++dx)
    {
        for (size_t dy = -1; dy < 2; ++dy)
        {
            if (dx == 0 && dy == 0)
                continue;
            idx_nerigbor(0) = (currentPtr->index)(0) + dx;
            idx_nerigbor(1) = (currentPtr->index)(1) + dy;

            coord_nerigbor = gridIndex2coord(idx_nerigbor);

            if (idx_nerigbor(0) < 0 || idx_nerigbor(0) >= GLX_SIZE || idx_nerigbor(1) < 0 || idx_nerigbor(1) >= GLY_SIZE)
                continue;
            idx_nerigbor = coord2gridIndex(coord_nerigbor);

            if (isFree(idx_nerigbor))
            {
                GridNodePtr neighborPtr = new GridNode(idx_nerigbor,coord_nerigbor);

                neighborPtr->gScore = currentPtr->gScore + getEuclHeu(currentPtr,neighborPtr);
                neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr,endPtr);
                neighborPtr->parent = currentPtr;

                edgeCostSets.push_back(neighborPtr->fScore);
                neighborPtrSets.push_back(neighborPtr);
            }
        }    
    }
}

double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2)
{
    double tie_breaker = 1 + 1/1000;
    return tie_breaker * getDiagHeu(node1,node2);
}

double AstarPathFinder::getEuclHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = node1->index(0) - node2->index(0);
    double dy = node1->index(1) - node2->index(1);
    double distance = sqrt(dx * dx + dy * dy);
    return distance;
}

double AstarPathFinder::getDiagHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = node1->index(0) - node2->index(0);
    double dy = node1->index(1) - node2->index(1);

    double h = 0.0;
    int diag = min(dx,dy);
    dx -= diag;
    dy -= diag;

    if (dx == 0)
        h = sqrt(2.0) * diag + abs(dy);
    if (dy == 0)
        h = sqrt(2.0) * diag + abs(dx);
    return h;
}

double AstarPathFinder::getManhHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    return dx + dy;
}

void AstarPathFinder::AstarGraphSearch(Vector2i start_idx, Vector2i end_idx)
{   
    // 这里传进来的start_pt和end_pt已经是从地图转到图像的坐标
    ros::Time time_start = ros::Time::now();    

    if (isOccupied(end_idx))
    {
        ROS_INFO("goal isOccupied!");
        return;
    }

    Vector2d start_coord = gridIndex2coord(start_idx);
    Vector2d end_coord = gridIndex2coord(end_idx);
    // 初始化节点信息
    startPtr = new GridNode(start_idx, start_coord);
    endPtr   = new GridNode(end_idx,   end_coord);
    
    ROS_INFO("Start Node position: (%f,%f)",startPtr->coord(0),startPtr->coord(1));
    ROS_INFO("End Node position: (%f,%f)",endPtr->coord(0),endPtr->coord(1));

    //openSet is the open_list implemented through multimap in STL library
    openSet.clear();
    // currentPtr 表示在open_list中f(n)最小的那个节点
    GridNodePtr currentPtr  = NULL;
    GridNodePtr neighborPtr = NULL;

    // 将开始节点放入open set中
    startPtr -> gScore = 0;
    // 计算startPtr到endPtr之间的代价
    startPtr -> fScore = getHeu(startPtr,endPtr);   
    startPtr -> id = 1; // id = 1表示在OpenSet中，id = -1表示在闭集中
    openSet.insert( make_pair(startPtr->fScore, startPtr) );

    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;

    // this is the main loop
    while ( !openSet.empty() ){

        std::multimap<double, GridNodePtr>::iterator itFoundmin;
        for (itFoundmin = openSet.begin(); itFoundmin != openSet.end(); itFoundmin++)
        {
            if (itFoundmin->second->id == 1)    //说明该节点没被访问过
            {
                currentPtr = itFoundmin->second;
                currentPtr->id = -1;    //标记当前节点为已访问状态
                GridNodeMap[currentPtr->index(0)][currentPtr->index(1)]->id = -1;
                break;
            }
        }

        // ROS_INFO("Current Node index : (%d,%d,%d)", currentPtr->index(0), currentPtr->index(1), currentPtr->index(2));
        // ROS_INFO("Current Node Coord : (%f,%f,%f)", currentPtr->coord(0), currentPtr->coord(1), currentPtr->coord(2));

        //找到Goal,返回
        if (currentPtr->index == end_idx)
        {
            ros::Time time_end = ros::Time::now();
            terminatePtr = currentPtr;
            ROS_WARN("[A*] succeed, Time consumed in Astar path finding is %f ms, path cost is %f m", (time_end - time_start).toSec() * 1000.0, currentPtr->gScore * resolution);
            return;
        }

        //得到当前节点的neighbor
        AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);

        //处理发现的neighbor
        for (int i = 0; i < (int)neighborPtrSets.size(); i++)
        {
            neighborPtr = neighborPtrSets[i];

            std::multimap<double, GridNodePtr>::iterator itFoundSame = openSet.begin();
            for (itFoundSame = openSet.begin(); itFoundSame != openSet.end(); itFoundSame++)
            {
                if (neighborPtr->index(0) == itFoundSame->second->index(0)  //openset中已经存在此neighbor
                && neighborPtr->index(1) == itFoundSame->second->index(1))
                    break;
            }

            if (itFoundSame == openSet.end())   //openSet中没有该neighbor
            {
                neighborPtr->id = 1;   //标记该节点为未访问状态
                // ROS_INFO("neighborPtr -> fScore = %f", neighborPtr -> fScore);
                openSet.insert(make_pair(neighborPtr->fScore, neighborPtr));
            }
            else    //该节点已经在openSet内
            {
                if (itFoundSame->second->gScore > neighborPtr->gScore)   //openSet中的g(n) > 当前路线的g(n), 需要更新
                {
                    itFoundSame->second->parent = currentPtr;
                    itFoundSame->second->gScore = neighborPtr->gScore;
                    itFoundSame->second->fScore = neighborPtr->fScore;
                    delete neighborPtr;    //若节点已经在openSet内,更新openSet中的节点内容后,删除内容相同的多余内存
                }
            }
        } 
    }
    //if search fails
    ros::Time time_end = ros::Time::now();
    if((time_end - time_start).toSec() > 0.1)
        ROS_WARN("Time consume in Astar path finding is %f", (time_end - time_start).toSec() );
}

// 获取最优路径
std::vector<Eigen::Vector2d> AstarPathFinder::getPath()
{
    vector<Vector2d> path;
    vector<GridNodePtr> gridPath;

    GridNodePtr currentPtr = terminatePtr;
    while (currentPtr != startPtr)
    {
        path.push_back(currentPtr->coord);
        currentPtr = currentPtr->parent;
    }
    reverse(path.begin(),path.end());
    return path;
}
