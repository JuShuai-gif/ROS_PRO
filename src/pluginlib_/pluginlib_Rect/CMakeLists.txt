cmake_minimum_required(VERSION 2.8.3)
project(pluginlib_Rect)

## 找catkin 依赖
find_package(catkin REQUIRED COMPONENTS pluginlib roscpp)

## Find Boost (headers only)
find_package(Boost REQUIRED)

## Define catkin exports
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pluginlib_Rect
  CATKIN_DEPENDS pluginlib roscpp
  DEPENDS Boost
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

## pluginlib_Rect library
add_library(pluginlib_Rect src/Square.cpp src/Triangle.cpp)
target_link_libraries(pluginlib_Rect ${catkin_LIBRARIES})

## polygon_loader executable
add_executable(polygon_loader src/polygon_loader.cpp)
target_link_libraries(polygon_loader ${catkin_LIBRARIES})

## Mark executables and/or libraries for installation
install(TARGETS pluginlib_Rect polygon_loader
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES polygon_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})