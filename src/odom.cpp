#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "project1/Reset.h"
#include <math.h>

class Odometry {

private:
    ros::Publisher pub2;

    double odom_vel_x = 0.0; // velocita x odometria
    double odom_vel_y = 0.0; // velocita y odometria
    double odom_x = 0.0; // posizione x odometria
    double odom_y = 0.0; // posizione y odometria
    double odom_ang = 0.0; // angolo odometria

    ros::Time last_time;

public:

    bool reset_pose_callback(project1::Reset::Request &req, project1::Reset::Response &res) {
        this->odom_x = req.newX;
        this->odom_y = req.newY;
        this->odom_ang = req.newTheta;

        ROS_INFO("Called reset with args [%lf %lf %lf]", req.newX, req.newY, req.newTheta);

        return true;
    }


    void newVelReceived(const geometry_msgs::TwistStamped::ConstPtr& msg) {

        double vel_x = msg.get()->twist.linear.x;
        double vel_y = msg.get()->twist.linear.y;
        double vel_z = msg.get()->twist.angular.z;
        double velocity = sqrt(pow(vel_x, 2) + pow(vel_y, 2));
        double ang = atan2(vel_y, vel_x) * 180.0 / M_PI; // angolo velocita robot

        ros::Time nowTime = ros::Time::now();
        double dt = (nowTime - last_time).toSec();

        nav_msgs::Odometry out_odom; //messaggio da stampare
        tf::TransformBroadcaster odom_broad;

        ROS_INFO("vel: [%lf %lf]", velocity, ang);


        odom_vel_x = vel_x * cos(odom_ang) - vel_y * sin(odom_ang);
        odom_vel_y = vel_x * sin(odom_ang) + vel_y * cos(odom_ang);

        odom_x = odom_x + odom_vel_x * dt;
        odom_y = odom_y + odom_vel_y * dt;
        odom_ang = odom_ang + vel_z * dt;

        ROS_INFO("odom: [%lf %lf %lf]", odom_x, odom_y, odom_ang * 180.0 / M_PI);

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom_ang);

        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = nowTime;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = odom_x;
        odom_trans.transform.translation.y = odom_y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        ROS_INFO("odom_quat: [%lf %lf]", odom_quat.z, odom_quat.w);

        odom_broad.sendTransform(odom_trans);

        out_odom.header.stamp = nowTime;
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

        last_time = ros::Time::now();
    }

    void main_loop() {
        ros::NodeHandle n;

        ros::Subscriber sub = n.subscribe("/cmd_vel", 1000, &Odometry::newVelReceived, this);
        pub2 = n.advertise<nav_msgs::Odometry>("/odom", 1000);

        ros::ServiceServer resetService = n.advertiseService("/odom/reset", &Odometry::reset_pose_callback, this);


        ros::spin();
    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom");
    Odometry odom;

    odom.main_loop();

    return 0;
}