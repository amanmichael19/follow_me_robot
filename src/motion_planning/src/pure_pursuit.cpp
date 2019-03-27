#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <queue>
#include <cmath>
#include <string>
#include <tf/tf.h>


using namespace std;
using namespace ros;

struct WayPoint{
    double x;
    double y;
    double curvature;
    WayPoint(double x_in, double y_in, double curv_in): x(x_in), y(y_in), curvature(curv_in){}
};

class PurePursuit{
    queue<WayPoint*> generated_path;
    vector<WayPoint*> figure8_path;
    double look_ahead;
    Publisher vel_pub;
    Subscriber pose_sub;
    nav_msgs::Odometry odom_input_data;


    public:
        PurePursuit(int argc, char** argv, string node_name){
            init(argc, argv, node_name);
            NodeHandle n;
            vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
            //pose_sub = n.subscribe<Pose>("/visp...", 1, &PurePursuit::generatorCallback, this);
            pose_sub = n.subscribe("/odom", 10, &PurePursuit::poseCallback, this);
            form_figure8_path();
        }
        // void generatorCallback(const geometry_msgs::Pose::ConstPtr& pose_msg){
            
        // }
        void poseCallback(const nav_msgs::Odometry::ConstPtr& odom_msg){
            odom_input_data = *odom_msg;
        }

        void form_figure8_path(){
            double x = 0.0, y=0.0;
            for(double i = 0.0; i<6.911; i+=0.628){
                x = 4*sin(i);
                y = 4*sin(i)*cos(i);
                figure8_path.push_back(new WayPoint(x, y, 0.0));
            }
        }
        double quaternionToEulerYaw(){
            tf::Quaternion quat;
            tf::quaternionMsgToTF(odom_input_data.pose.pose.orientation, quat);
            double roll, pitch, yaw;
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            return yaw;
        }
        double get_euclidean_distance(double x1, double y1, double x2, double y2){
            return sqrt(pow(x2-x1, 2) + pow(y2-y1, 2));
        }
        WayPoint* get_closest_path_point(){
            double current_x = odom_input_data.pose.pose.position.x;
            double current_y = odom_input_data.pose.pose.position.y;
            double smallest = get_euclidean_distance(current_x, current_y, figure8_path[0]->x, figure8_path[0]->y);
            WayPoint* closest = figure8_path[0];
            for(WayPoint* waypt: figure8_path){
                double diff = get_euclidean_distance(current_x, current_y, waypt->x, waypt->y);
                if(smallest > diff){
                    smallest = diff;
                    closest = waypt;
                }
            }
            return closest;
        }
        WayPoint* get_goalpt_lookAhead_distance_away(){
            // use a lookahead distance to select from waypoints
        }

        void track_path(){
            WayPoint* closest = get_closest_path_point();
        }
};


int main(int argc, char** argv){
    PurePursuit pure_pursuit(argc, argv, "pure_pursuit_node");

    return 0;
}