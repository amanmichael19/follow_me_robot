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
    bool visited;
    WayPoint(double x_in, double y_in, double curv_in): x(x_in), y(y_in), curvature(curv_in), visited(false){}
};

class PurePursuit{
    queue<WayPoint*> generated_path;
    vector<WayPoint*> figure8_path;
    vector<WayPoint*> semi_circle_path;
    double look_ahead_dist;
    Publisher vel_pub;
    Subscriber pose_sub;
    nav_msgs::Odometry odom_input_data;
    bool on_path;
    int last_visited_waypt_idx = 0;
    int next_waypt_idx = 1;


    public:
        PurePursuit(int argc, char** argv, string node_name){
            init(argc, argv, node_name);
            NodeHandle n;
            vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
            //pose_sub = n.subscribe<Pose>("/visp...", 1, &PurePursuit::generatorCallback, this);
            
            // for testing purposes, getting pose from odom
            pose_sub = n.subscribe("/odom", 10, &PurePursuit::poseCallback, this);
            form_figure8_path();
        }
        // void generatorCallback(const geometry_msgs::Pose::ConstPtr& pose_msg){
            
        // }
        void poseCallback(const nav_msgs::Odometry::ConstPtr& odom_msg){
            odom_input_data = *odom_msg;
        }
        // using figure8 to test the path tracking algorithm
        void form_figure8_path(){
            double x = 0.0, y=0.0;
            for(double i = 0.0; i<6.597; i+=0.314){ //6.911 0.628
                x = 4*sin(i);
                y = 4*sin(i)*cos(i);
                figure8_path.push_back(new WayPoint(x, y, 0.0));
            }
        }
        void semi_circle_path(){
            double x = 0.0, y = 0.0;
            for(double i=0.0; i<3.2986; i+=0.157){
                x = 5*cos(i);
                y = 5*sin(i);
                semi_circle_path.push_back(new WayPoint(x, y, 1/5));
            }
        }
        double quaternionToEulerYaw(){
            tf::Quaternion quat;
            tf::quaternionMsgToTF(odom_input_data.pose.pose.orientation, quat);
            double roll, pitch, yaw;
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            return yaw;
        }
        template<typename T>
        int sign(T val){
            return val >= 0 ? 1 : -1;
        }
        double robot_posX(){
            return odom_input_data.pose.pose.position.x;
        }
        double robot_posY(){
            return odom_input_data.pose.pose.position.y;
        }
        double get_euclidean_distance(double x1, double y1, double x2, double y2){
            return sqrt(pow(x2-x1, 2) + pow(y2-y1, 2));
        }
        WayPoint* get_closest_path_point(){
            double smallest = get_euclidean_distance(robot_posX(), robot_posY(), figure8_path[0]->x, figure8_path[0]->y);
            WayPoint* closest = figure8_path[0];
            for(WayPoint* waypt: figure8_path){
                double diff = get_euclidean_distance(robot_posX(), robot_posY(), waypt->x, waypt->y);
                if(smallest > diff){
                    smallest = diff;
                    closest = waypt;
                    if(smallest < look_ahead_dist){
                        on_path = true;
                    } else {
                        on_path = false;
                    }
                }
            }
            
            return closest;
        }

        WayPoint* get_goalpt_lookAhead_distance_away(WayPoint* closest){
            // use a lookahead distance to select from waypoints
            for(int i = last_visited_waypt_idx; i < semi_circle_path.size()-1; i++){
                WayPoint* waypt_i = semi_circle_path(i);
                WayPoint* waypt_next = semi_circle_path(i+1);
                double lower_bd_dist = get_euclidean_distance(robot_posX(), robot_posY(), waypt_i->x, waypt_i->y);
                double upper_bd_dist = get_euclidean_distance(robot_posX(), robot_posY(), waypt_next->x, waypt_next->y)
                if(lower_bd_dist < look_ahead_dist && upper_bd_dist > look_ahead_dist){
                    last_visited_waypt_idx = i;
                    next_waypt_idx = i+1;
                    double slope = (waypt_next->y - waypt_i->y) / (waypt_next->x - waypt_i->x);
                    double x_m = waypt_i->x + sign(waypt_next->x - waypt_i->x) * look_ahead_dist * sqrt(1/(1+pow(slope, 2)));
                    double y_m = waypt_i->y + sign(waypt_next->y - waypt_i->y) * look_ahead_dist * slope * sqrt(1/(1+pow(slope, 2)));
                    WayPoint* interpolated_waypt = new WayPoint(x_m, y_m, 0.0);
                    return interpolated_waypt;
                }
            }
            return nullptr;

        }

        void getDesiredCurvature(WayPoint* goalpt){
            double curvature = 2*(goalpt->x - robot_posX())/ pow(look_ahead_dist, 2);
            
        }
};


int main(int argc, char** argv){
    PurePursuit pure_pursuit(argc, argv, "pure_pursuit_node");

    return 0;
}