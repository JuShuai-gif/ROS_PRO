cmake_minimum_required(VERSION 3.0.2)
project(pluginlib_test)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pluginlib_test
  CATKIN_DEPENDS pluginlib roscpp
  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

## 生成 library
add_library(pluginlib_test src/Astr.cpp src/Dijkstra.cpp)
target_link_libraries(pluginlib_test ${catkin_LIBRARIES})

## 生成调用的可执行文件
add_executable(path_plan_loader src/path_plan_loader.cpp)
target_link_libraries(path_plan_loader ${catkin_LIBRARIES})

## Mark executables and/or libraries for installation
install(TARGETS pluginlib_test path_plan_loader
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES plan_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})