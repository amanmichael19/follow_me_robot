#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>

using namespace ros;
using namespace std;

int main(int argc, char** argv){
    init(argc, argv, "visp_auto_tracker_node");
    NodeHandle n;
    Publisher visp_pub = n.advertise<geometry_msgs::PoseStamped>("/visp_auto_tracker/object_position", 10);
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = Time::now();
    pose.header.frame_id = "/camera";
    pose.pose.position.x = 3.0;
    pose.pose.position.y = 4.0;
    pose.pose.position.z = 0.0;
    while(ok()){
      visp_pub.publish(pose);
    }
}
