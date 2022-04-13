#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include <math.h>

double wheelAngle(int i) {
    double rad = 45.0 * (M_PI / 180.0);
    return i % 2 == 1 ? rad : -rad;
}

void newVelReceived(const sensor_msgs::JointState::ConstPtr& msg) {
    auto vel = msg.get()->velocity;
    std::vector<double> vel_s(4); // perpendicolare alla ruota
    std::vector<double> vel_e(4); // parallela alla ruota

    for (size_t i = 0; i < vel.size(); i++) {
        double vel_ir = (vel[i]/60) * 0.07 * cos(45.0 * (M_PI / 180.0));

        vel_s[i] = vel_ir * sin(wheelAngle(i));
        vel_e[i] = (vel[i]/60)*0.07 + vel_ir*cos(wheelAngle(i));
    }

    

    ROS_INFO("vel_s: [%lf %lf %lf %lf]", vel_s[0], vel_s[1], vel_s[2], vel_s[3]);
    ROS_INFO("vel_e: [%lf %lf %lf %lf]", vel_e[0], vel_e[1], vel_e[2], vel_e[3]);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "vel_calc");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/wheel_states", 1000, newVelReceived);

    ros::spin();

    return 0;
}
