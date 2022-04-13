cmake_minimum_required(VERSION 3.0.2)
project(project1)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(pub src/pub.cpp)
target_link_libraries(pub ${catkin_LIBRARIES})

add_executable(vel_calc src/vel_calc.cpp)
target_link_libraries(vel_calc ${catkin_LIBRARIES})