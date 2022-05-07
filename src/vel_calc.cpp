#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/TwistStamped.h"
#include <math.h>

ros::Publisher pub;

std::vector<double> oldpos(4, 0.0);
ros::Time oldtime(0, 0);
bool init = false;

double robot_size_x;
double robot_size_y;
double robot_tpr;
double robot_wheel_radius;
double robot_gear_ratio;

void position_received_callback(const sensor_msgs::JointState::ConstPtr& msg) {
    std::vector<double> vel(4, 0.0);

    if(init)
        for(size_t i = 0; i < 4; ++i) {
            ros::Duration dt = msg.get()->header.stamp - oldtime;
            vel[i] = 2.0 * M_PI * (msg.get()->position[i] - oldpos[i]) / robot_tpr / robot_gear_ratio / dt.toSec();
        }

    oldpos =  msg.get()->position;
    oldtime = msg.get()->header.stamp;
    init = true;

    ROS_INFO("old_pose: [%lf %lf %lf %lf]", oldpos[0], oldpos[1], oldpos[2], oldpos[3]);
    ROS_INFO("wheel_speed: [%lf %lf %lf %lf]", vel[0], vel[1], vel[2], vel[3]);


    double vel_x = (vel[0] + vel[1] + vel[2] + vel[3]) * robot_wheel_radius / 4.0; // velocita x robot
    double vel_y = (-vel[0] + vel[1] + vel[2] - vel[3]) * robot_wheel_radius / 4.0; // velocita y robot
    double vel_z = (-vel[0] + vel[1] - vel[2] + vel[3]) * robot_wheel_radius / 4.0 / (robot_size_x + robot_size_y); // velocita angolare

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

    n.getParam("robot_wheel_radius", robot_wheel_radius);
    n.getParam("robot_size_x", robot_size_x);
    n.getParam("robot_size_y", robot_size_y);
    n.getParam("robot_tpr", robot_tpr);
    n.getParam("robot_gear_ratio", robot_gear_ratio);

    ros::Subscriber sub = n.subscribe("/wheel_states", 1000, position_received_callback);
    pub = n.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 1000);


    ros::spin();

    return 0;
}
