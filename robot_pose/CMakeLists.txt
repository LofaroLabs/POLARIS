cmake_minimum_required(VERSION 2.8.3)
project(robot_pose)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -std=gnu++0x -Os -fPIC")
## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS geometry_msgs roscpp rospy std_msgs tf visualization_msgs)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs tf visualization_msgs)

##Build robot pose
include_directories(include ${catkin_INCLUDE_DIRS} )

add_executable(pose src/pose.cpp)
target_link_libraries(pose ${catkin_LIBRARIES})

#add_library (pose_listener src/pose_listener.cpp)
add_executable(pose_listener src/pose_listener.cpp)
target_link_libraries(pose_listener ach ${catkin_LIBRARIES})

if(catkin_EXPORTED_TARG)
  add_dependencies(pose pose_listener ${catkin_EXPORTED_TARGETS})
endif()
