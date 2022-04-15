#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include <math.h>

ros::Publisher pub;
ros::Publisher pub2;

double odom_vel_x = 0.0; // velocita x odometria
double odom_vel_y = 0.0; // velocita y odometria
double odom_x = 0.0; // posizione x odometria
double odom_y = 0.0; // posizione y odometria
double odom_ang = 0.0; // angolo odometria

ros::Time current_time;
ros::Time last_time;

void newVelReceived(const sensor_msgs::JointState::ConstPtr& msg) {
    auto vel = msg.get()->velocity;
    double vel_x; // velocita x robot
    double vel_y; // velocita y robot
    double vel_z; // velocita angolare
    double velocity; // modulo velocita robot
    double ang; // angolo velocita robot

    current_time = ros::Time::now();

    double dt = (current_time - last_time).toSec();

    geometry_msgs::TwistStamped out_robot; //messaggio da stampare
    nav_msgs::Odometry out_odom; //messaggio da stampare
    tf::TransformBroadcaster odom_broad;

    vel_x = (vel[0] + vel[1] + vel[2] + vel[3]) / 60 * 0.07 / 4.0;
    vel_y = (-vel[0] + vel[1] + vel[2] - vel[3]) / 60 * 0.07 / 4.0;
    vel_z = (-vel[0] + vel[1] - vel[2] + vel[3]) / 60 * 0.07 / 4.0 / (0.2 + 0.169);

    velocity = sqrt(pow(vel_x, 2) * pow(vel_y, 2));
    ang = atan(vel_y/vel_x) * 180 / M_PI;

    if(vel_x < 0) ang += 180.0;

    ROS_INFO("vel: [%lf %lf]", velocity, ang);

    out_robot.header.stamp = current_time;

    out_robot.twist.linear.x = vel_x;
    out_robot.twist.linear.y = vel_y;
    out_robot.twist.linear.z = 0.0;
    out_robot.twist.angular.x = 0.0;
    out_robot.twist.angular.y = 0.0;
    out_robot.twist.angular.z = vel_z;

    pub.publish(out_robot);

    odom_vel_x = vel_x * cos(odom_ang * M_PI / 180) - vel_y * sin(odom_ang * M_PI / 180);
    odom_vel_y = vel_x * sin(odom_ang * M_PI / 180) + vel_y * cos(odom_ang * M_PI / 180);

    odom_x = odom_x + odom_vel_x * dt;
    odom_y = odom_y + odom_vel_y * dt;
    odom_ang = odom_ang + vel_z * dt * 180 / M_PI;

    while (odom_ang > 360.0) odom_ang -= 360.0;
    while (odom_ang < 0) odom_ang += 360.0;

    ROS_INFO("odom: [%lf %lf %lf]", odom_x, odom_y, odom_ang);

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom_ang);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = odom_x;
    odom_trans.transform.translation.y = odom_y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_broad.sendTransform(odom_trans);

    out_odom.header.stamp = current_time;
    out_odom.header.frame_id = "odom";

    out_odom.pose.pose.position.x = odom_x;
    out_odom.pose.pose.position.y = odom_y;
    out_odom.pose.pose.position.z = 0.0;
    out_odom.pose.pose.orientation = odom_quat;

    out_odom.child_frame_id = "base_link";

    out_odom.twist.twist.linear.x = odom_vel_x;
    out_odom.twist.twist.linear.y = odom_vel_y;
    out_odom.twist.twist.angular.z = vel_z;

    pub2.publish(out_odom);

    last_time = current_time;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "vel_calc");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/wheel_states", 1000, newVelReceived);
    pub = n.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1000);
    pub2 = n.advertise<nav_msgs::Odometry>("odom", 1000);


    ros::spin();

    return 0;
}
