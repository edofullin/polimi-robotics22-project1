#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include "project1/Reset.h"
#include "project1/intparamsConfig.h"
#include <dynamic_reconfigure/server.h>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>

enum int_method
{
    EULER = 0,
    RK = 1
};

double odom_x_init;
double odom_y_init;
double odom_ang_init;

class Odometry {

private:
    ros::Publisher odom_publisher;

    double odom_vel_x = 0.0; // velocita x odometria
    double odom_vel_y = 0.0; // velocita y odometria
    double odom_x = 0.0; // posizione x odometria
    double odom_y = 0.0; // posizione y odometria
    double odom_ang = 0.0; // angolo odometria
    int_method method = EULER;

    ros::Time last_time;

public:

    bool reset_pose_callback(project1::Reset::Request &req, project1::Reset::Response &res) {
        this->odom_x = req.newX;
        this->odom_y = req.newY;
        this->odom_ang = req.newTheta;

        ROS_INFO("Called reset with args [%lf %lf %lf]", req.newX, req.newY, req.newTheta);

        return true;
    }


    void velocity_received_callback(const geometry_msgs::TwistStamped::ConstPtr& msg) {

        double vel_x = msg.get()->twist.linear.x;
        double vel_y = msg.get()->twist.linear.y;
        double vel_z = msg.get()->twist.angular.z;
        double velocity = sqrt(pow(vel_x, 2) + pow(vel_y, 2));
        double ang = atan2(vel_y, vel_x); // angolo velocita robot

        ros::Time nowTime = ros::Time::now();
        double dt = (nowTime - last_time).toSec();

        nav_msgs::Odometry out_odom; //messaggio da stampare

        ROS_INFO("vel: [%lf %lf]", velocity, ang * 180.0 / M_PI);

        if(method == EULER){
            odom_vel_x = velocity * cos(odom_ang + ang);
            odom_vel_y = velocity * sin(odom_ang + ang);
        }
        else{
            odom_vel_x = velocity * cos(odom_ang + ang + (vel_z * dt) / 2);
            odom_vel_y = velocity * sin(odom_ang + ang + (vel_z * dt) / 2);
        }

        odom_x = odom_x + odom_vel_x * dt;
        odom_y = odom_y + odom_vel_y * dt;
        odom_ang = odom_ang + vel_z * dt;

        ROS_INFO("odom: [%lf %lf %lf] using %s", odom_x, odom_y, odom_ang * 180.0 / M_PI, method ? "runge_kutta" : "euler");

        tf2::Quaternion tf_quat;
        tf_quat.setRPY(0, 0, odom_ang);

        out_odom.header.stamp = nowTime;

        out_odom.pose.pose.position.x = odom_x;
        out_odom.pose.pose.position.y = odom_y;
        out_odom.pose.pose.position.z = 0.0;
        out_odom.pose.pose.orientation.x = tf_quat.x();
        out_odom.pose.pose.orientation.y = tf_quat.y();
        out_odom.pose.pose.orientation.z = tf_quat.z();
        out_odom.pose.pose.orientation.w = tf_quat.w();

        odom_publisher.publish(out_odom);

        last_time = ros::Time::now();
    }

    void dyn_rec_callback(project1::intparamsConfig &config, uint32_t level) {
        ROS_INFO("dynamic reconfigure: [%d]", config.odom_int);

        this->method = (int_method)config.odom_int;
    }

    void main_loop() {

        ros::NodeHandle n;

        n.getParam("odom_x_init", odom_x_init);
        n.getParam("odom_y_init", odom_y_init);
        n.getParam("odom_ang_init", odom_ang_init);

        odom_x = odom_x_init;
        odom_y = odom_y_init;
        odom_ang = odom_ang_init;

        ros::Subscriber sub = n.subscribe("/cmd_vel", 1000, &Odometry::velocity_received_callback, this);
        odom_publisher = n.advertise<nav_msgs::Odometry>("/odom", 1000);

        ros::ServiceServer resetService = n.advertiseService("/odom/reset", &Odometry::reset_pose_callback, this);

        dynamic_reconfigure::Server<project1::intparamsConfig> dynServer;
        dynamic_reconfigure::Server<project1::intparamsConfig>::CallbackType dyn_rec_f;

        dyn_rec_f = boost::bind(&Odometry::dyn_rec_callback, this, _1, _2);
        
        dynServer.setCallback(dyn_rec_f);

        ros::spin();
    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom");
    Odometry odom;

    odom.main_loop();

    return 0;
}