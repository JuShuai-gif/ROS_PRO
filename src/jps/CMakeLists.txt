########### 版本要求
cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_STANDARD 14)

########### 项目名称
project(jps)

########### 添加系统库
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        geometry_msgs
        nav_msgs
        tf
        )

catkin_package()

########### 添加头文件
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

########### 编译可执行文件
add_executable(${PROJECT_NAME} src/demo_node.cpp src/Astar_searcher.cpp src/OccMapTrans.cpp)

# 指定可执行文件或库的包含路径
target_include_directories(${PROJECT_NAME}
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include/jps  # 添加你的头文件路径
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS})

