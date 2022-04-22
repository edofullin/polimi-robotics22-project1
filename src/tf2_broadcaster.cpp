#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

class Tf2Broadcaster {
private:

    ros::NodeHandle n;
    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    ros::Subscriber sub;

public:

    void callback(const nav_msgs::Odometry& message) {

        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "odom";

        transformStamped.transform.translation.x = message.pose.pose.position.x;
        transformStamped.transform.translation.y = message.pose.pose.position.x;
        transformStamped.transform.translation.z = 0.0;

        transformStamped.transform.rotation.x = message.pose.pose.orientation.x;
        transformStamped.transform.rotation.y = message.pose.pose.orientation.y;
        transformStamped.transform.rotation.z = message.pose.pose.orientation.z;
        transformStamped.transform.rotation.w = message.pose.pose.orientation.w;

        br.sendTransform(transformStamped);
    }

    Tf2Broadcaster() {

        sub = n.subscribe("", 1000, &Tf2Broadcaster::callback, this);

    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "tf2_broadcaster");

    Tf2Broadcaster broadcaster;
    ros::spin();

    return 0;
}