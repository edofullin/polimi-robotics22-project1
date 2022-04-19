#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/TwistStamped.h"
#include <math.h>

ros::Publisher pub;

void newVelReceived(const sensor_msgs::JointState::ConstPtr& msg) {
    auto vel = msg.get()->velocity;
    double vel_x = (vel[0] + vel[1] + vel[2] + vel[3]) / 60.0 * 0.07 / 4.0; // velocita x robot
    double vel_y = (-vel[0] + vel[1] + vel[2] - vel[3]) / 60.0 * 0.07 / 4.0; // velocita y robot
    double vel_z = (-vel[0] + vel[1] - vel[2] + vel[3]) / 60.0 * 0.07 / 4.0 / (0.2 + 0.169); // velocita angolare

    geometry_msgs::TwistStamped out_robot; //messaggio da stampare

    out_robot.twist.linear.x = vel_x;
    out_robot.twist.linear.y = vel_y;
    out_robot.twist.angular.z = vel_z;

    out_robot.header.stamp = ros::Time::now();

    pub.publish(out_robot);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "vel_calc");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/wheel_states", 1000, newVelReceived);
    pub = n.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1000);


    ros::spin();

    return 0;
}
