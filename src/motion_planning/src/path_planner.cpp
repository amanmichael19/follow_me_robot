#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <tf/tf.h>
#include <string>
#include <iostream>

using namespace ros;
using namespace std;

class PathPlanner{
  private:
    nav_msgs::Path _target_path;
    Subscriber _pose_sub;
    Publisher _path_pub;
    vector<geometry_msgs::PoseStamped> _path_point_poses;
    vector<geometry_msgs::PoseStamped> _untransformed_poses;
  public:
    PathPlanner(NodeHandle& n){
      _path_pub = n.advertise<nav_msgs::Path>("/generated_path", 10);
      _pose_sub = n.subscribe<geometry_msgs::PoseStamped>("/visp_auto_tracker/object_position", 10, &PathPlanner::poseCallback, this);
      formFigure8Path();
      Rate my_rate(20);
      while(ok()){
         _path_pub.publish(_target_path);
      }
    }
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
        _untransformed_poses.push_back(*msg);
    }
    void formFigure8Path(){
        double x = 0.0, y=0.0;
        for(double i = 0.0; i<6.3; i+=0.314){ //6.911 0.628
            x = 2*sin(i);
            y = 2*sin(i)*cos(i);
            makePoseStampedObject(x, y, 0.0);
        }
        makePathObject();
    }
    void makePoseStampedObject(double x, double y, double z){
      geometry_msgs::PoseStamped pose;

      pose.header.stamp = Time::now();
      pose.header.frame_id = "/map";
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.position.z = z;
      _path_point_poses.push_back(pose);
    }
    void makePathObject(){
      _target_path.header.stamp = Time::now();
      _target_path.header.frame_id = "/map";
      _target_path.poses = _path_point_poses;
    }
};

int main(int argc, char** argv){
    init(argc, argv, "path_planner_node");
    NodeHandle n;
    PathPlanner path_planner(n);
    spin();
}
