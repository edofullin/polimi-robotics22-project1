cmake_minimum_required(VERSION 3.0.2)
project(project1)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs tf)

## Declare a catkin package
catkin_package()

## Build talker and listener

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(odom src/odom.cpp)
target_link_libraries(odom ${catkin_LIBRARIES})

add_executable(vel_calc src/vel_calc.cpp)
target_link_libraries(vel_calc ${catkin_LIBRARIES})