cmake_minimum_required(VERSION 3.0.2)
project(waypoint_rviz_plugin)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rviz
)
set(CMAKE_AUTOMOC ON)
if(rviz_QT_VERSION VERSION_LESS "5")
 message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
 find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
 include(${QT_USE_FILE})
else()
 message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
 find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
 set(QT_LIBRARIES Qt5::Widgets)
endif()

add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
#  INCLUDE_DIRS include
LIBRARIES waypoint_rviz_plugin
  CATKIN_DEPENDS roscpp rospy rviz
#  DEPENDS system_lib
)


include_directories(
 include
  ${QT_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


set(HEADER_FILES
        include/${PROJECT_NAME}/waypoint_tool.h
)
set(SRC_FILES
    src/waypoint_tool.cpp
)

add_library(${PROJECT_NAME}
            ${SRC_FILES} ${HEADER_FILES}
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_LIBRARIES} ${Boost_LIBRARIES} ${OGRE_OV_LIBRARIES_ABS})
