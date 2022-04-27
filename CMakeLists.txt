cmake_minimum_required(VERSION 3.0.2)
project(project1)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs tf2 tf2_ros message_generation geometry_msgs dynamic_reconfigure)

add_service_files(
    FILES
    Reset.srv
)

add_message_files(
    FILES
    Wheel.msg
)

generate_dynamic_reconfigure_options(
    cfg/intparams.cfg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)



## Build talker and listener

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(odom src/odom.cpp)
add_dependencies(odom ${catkin_EXPORTED_TARGETS})
target_link_libraries(odom ${catkin_LIBRARIES})

add_executable(vel_calc src/vel_calc.cpp)
target_link_libraries(vel_calc ${catkin_LIBRARIES})

add_executable(tf2_broadcaster src/tf2_broadcaster.cpp)
add_dependencies(tf2_broadcaster ${catkin_EXPORTED_TARGETS})
target_link_libraries(tf2_broadcaster ${catkin_LIBRARIES})

add_executable(wheel_calc src/wheel_calc.cpp)
add_dependencies(wheel_calc ${catkin_EXPORTED_TARGETS})
target_link_libraries(wheel_calc ${catkin_LIBRARIES})