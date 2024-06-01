cmake_minimum_required(VERSION 3.0.2)
project(person_gdb)

SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  PersonGDB.msg
#   Message2.msg
 )


 generate_messages(
     DEPENDENCIES
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES person_pkg
  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(lis_person_gdb src/lis_person.cpp)
add_executable(pub_person_gdb src/pub_person.cpp)


add_dependencies(lis_person_gdb ${PROJECT_NAME}_generate_messages_cpp)

add_dependencies(pub_person_gdb ${PROJECT_NAME}_generate_messages_cpp)


## Specify libraries to link a library or executable target against
 target_link_libraries(lis_person_gdb
   ${catkin_LIBRARIES}
 )


 target_link_libraries(pub_person_gdb
 ${catkin_LIBRARIES}
)

