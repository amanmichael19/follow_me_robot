#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <queue>

using namespace std;
using namespace ros;

struct WayPoint{
    double x;
    double y;
    double z;
    WayPoint(double x_in, double y_in, double z_in): x(x_in), y(y_in), z(z_in){}
}

class PurePursuit{
    queue<WayPoint> generated_path;`
    double look_ahead;
    Publisher vel_pub;
    Subscriber pose_sub;

    PurePursuit(int argc, char** argv, string node_name){
        init(argc, argv, node_name);
        NodeHandle n;
        vel_pub = n.advertise<Twist>("/cmd_vel", 1);
        pose_sub = n.subscribe<Pose>("/visp...", 1, &PurePursuit::generatorCallback, this);
    }
    void generatorCallback(const geometry_msgs::Pose::ConstPtr& pose_msg){
        
    }
}


int main(int argc, char** argv){

    return 0;
}