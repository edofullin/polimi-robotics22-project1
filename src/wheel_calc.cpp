#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include "project1/Reset.h"
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>
#include "project1/Wheel.h"
#include "const.h"

ros::Publisher pub;

void velocity_received_callback(const geometry_msgs::TwistStamped::ConstPtr& msg) {

    double vel_x = msg.get()->twist.linear.x;
    double vel_y = msg.get()->twist.linear.y;
    double vel_z = msg.get()->twist.angular.z;

    project1::Wheel rpms;
    ros::Time nowTime = ros::Time::now();

    double rpm_fl = (vel_x - vel_y - (ROBOT_LENGTH + ROBOT_HEIGHT) * vel_z) / WHEEL_RADIUS * GEAR_RATIO * 60.0;
    double rpm_fr = (vel_x + vel_y + (ROBOT_LENGTH + ROBOT_HEIGHT) * vel_z) / WHEEL_RADIUS * GEAR_RATIO * 60.0;
    double rpm_rr = (vel_x + vel_y - (ROBOT_LENGTH + ROBOT_HEIGHT) * vel_z) / WHEEL_RADIUS * GEAR_RATIO * 60.0;
    double rpm_rl = (vel_x - vel_y + (ROBOT_LENGTH + ROBOT_HEIGHT) * vel_z) / WHEEL_RADIUS * GEAR_RATIO * 60.0;

    rpms.header.stamp = nowTime;
    rpms.rpm_fl = rpm_fl;
    rpms.rpm_fr = rpm_fr;
    rpms.rpm_rr = rpm_rr;
    rpms.rpm_rl = rpm_rl;

    ROS_INFO("rpms: [%lf, %lf, %lf %lf]", rpm_fl, rpm_fr, rpm_rr, rpm_rl);

    pub.publish(rpms);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "wheel_calc");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/cmd_vel", 1000, velocity_received_callback);
    pub = n.advertise<project1::Wheel>("wheels_rpm", 1000);


    ros::spin();

    return 0;
}