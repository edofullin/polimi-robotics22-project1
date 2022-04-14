#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/TwistStamped.h"
#include <math.h>

ros::Publisher pub;

void newVelReceived(const sensor_msgs::JointState::ConstPtr& msg) {
    auto vel = msg.get()->velocity;
    double vel_x; // velocita dritta
    double vel_y; // velocita laterale
    double vel_z; // velocita angolare
    double velocity; // modulo velocita
    double ang; // angolo velocita

    geometry_msgs::TwistStamped out; //messaggio da stampare

    vel_x = (vel[0] + vel[1] + vel[2] + vel[3]) / 60 * 0.07 / 4.0;
    vel_y = (-vel[0] + vel[1] - vel[2] + vel[3]) / 60 * 0.07 / 4.0;
    vel_z = (-vel[0] + vel[1] + vel[2] - vel[3]) / 60 * 0.07 / 4.0 / (0.2 + 0.169);

    velocity = sqrt(pow(vel_x, 2) * pow(vel_y, 2));
    ang = atan(vel_y/vel_x) * 180 / M_PI;

    if(vel_x < 0) ang += 180.0;

    ROS_INFO("vel: [%lf %lf]", velocity, ang);

    out.twist.linear.x = vel_x;
    out.twist.linear.y = vel_y;
    out.twist.linear.z = 0.0;
    out.twist.angular.x = 0.0;
    out.twist.angular.y = 0.0;
    out.twist.angular.z = vel_z;

    pub.publish(out);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "vel_calc");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/wheel_states", 1000, newVelReceived);
    pub = n.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1000);

    ros::spin();

    return 0;
}
