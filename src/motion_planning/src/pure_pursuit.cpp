#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <queue>
#include <cmath>
#include <string>
#include <tf/tf.h>

// Note there might be a memory leak problem might crash rarely, it is due to no deallocation. Will be done when
// everything else is done.

using namespace std;
using namespace ros;

struct Point{
    double x;
    double y;
    Point(double x_in, double y_in): x(x_in), y(y_in){}
};

struct WayPoint : public Point {
    double curvature;
    bool visited;
    WayPoint(double x_in, double y_in, double curv_in): Point(x_in, y_in), curvature(curv_in), visited(false){}
};

class PurePursuit{
    queue<WayPoint*> generated_path;
    vector<WayPoint*> figure8_path;
    vector<WayPoint*> semi_circle_path;
    double look_ahead_dist = 0.5;
    Publisher vel_pub;
    Subscriber pose_sub;
    nav_msgs::Odometry odom_input_data;
    int* last_visited_waypt_idx = nullptr;
    int* next_waypt_idx = nullptr;
    WayPoint* last_goalpt = nullptr;


    public:
        PurePursuit(int argc, char** argv, string node_name){
            init(argc, argv, node_name);
            NodeHandle n;
            vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
            //pose_sub = n.subscribe<Pose>("/visp...", 1, &PurePursuit::generatorCallback, this);
            
            // for testing purposes, getting pose from odom
            pose_sub = n.subscribe<nav_msgs::Odometry>("/odom", 10, &PurePursuit::poseCallback, this);
            formFigure8Path();
            formSemiCirclePath();
        }
        // void generatorCallback(const geometry_msgs::Pose::ConstPtr& pose_msg){
            
        // }
        void poseCallback(const nav_msgs::Odometry::ConstPtr& odom_msg){
            odom_input_data = *odom_msg;
        }

        /**
        * 
        * Helper functions start here
        *
        */


        // using figure8 to test the path tracking algorithm
        void formFigure8Path(){
            double x = 0.0, y=0.0;
            for(double i = 0.0; i<6.597; i+=0.314){ //6.911 0.628
                x = 4*sin(i);
                y = 4*sin(i)*cos(i);
                figure8_path.push_back(new WayPoint(x, y, 0.0));
            }
        }
        void formSemiCirclePath(){
            double x = 0.0, y = 0.0;
            for(double i=3.1415926; i > -0.1; i-=0.1570796){
                x = 3 + 3*cos(i);
                y = 3*sin(i);
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
        double robotPosX(){
            return odom_input_data.pose.pose.position.x;
        }
        double robotPosY(){
            return odom_input_data.pose.pose.position.y;
        }
        double getEuclideanDistance(double x1, double y1, double x2, double y2){
            return sqrt(pow(x2-x1, 2) + pow(y2-y1, 2));
        }

        /**
        * Shortest distance from a point (Xo,Yo) to a line ay + bx + c is
        *       |a(Yo)+b(Xo)+c|
        * d =  ____________________
        *         ___________
        *       \| a^2 + b^2
        */
        double getShortestDistToLine(double slope, WayPoint* waypt, Point* pt){
            double intercept = waypt->y - slope*waypt->x;
            double numerator = fabs(pt->y - (slope*pt->x) - intercept);
            double dist = numerator / sqrt(1 + pow(slope, 2));
            return dist;
        }
        Point* getPtOnLinePerpendicularToAPt(double slope, WayPoint* waypt, Point* pt){
            double slope_2 = -1 * slope;
            double intercept_1 = waypt->y - slope*waypt->x;
            double intercept_2 = pt->y - slope_2*pt->x;
            double intersection_x = (intercept_2 - intercept_1) / 2*slope;
            double intersection_y = (intercept_2 + intercept_1) / 2;
            return new Point(intersection_x, intersection_y);

        }
        template <typename T>
        bool betweenTwoptsOnALine(WayPoint* a, WayPoint* b, T* pt){
            double a_pt_dist = getEuclideanDistance(a->x, a->y, pt->x, pt->y);
            double pt_b_dist = getEuclideanDistance(pt->x, pt->y, b->x, b->y);
            double a_b_dist = getEuclideanDistance(a->x, a->y, b->x, b->y);
            if(a_pt_dist + pt_b_dist - a_b_dist < 0.00001){
                return true;
            } else {
                return false;
            }
        }
        /**
        *
        * Helper functions end
        *
        */
        
        /**
        * Check if the robot is on path
        *
        */
        bool onPath(){
            Point* current_pose = new Point(robotPosX(), robotPosY());
            if(last_visited_waypt_idx != nullptr){
                WayPoint* waypt_i = semi_circle_path[*last_visited_waypt_idx];
                WayPoint* waypt_next = semi_circle_path[*next_waypt_idx];
                double slope = (waypt_next->y - waypt_i->y) / (waypt_next->x - waypt_i->x);
                Point* pt_on_line = getPtOnLinePerpendicularToAPt(slope, waypt_i, current_pose);
                if(betweenTwoptsOnALine(waypt_i, waypt_next, pt_on_line) && (getShortestDistToLine(slope, waypt_i, current_pose) < look_ahead_dist)){
                    return true;
                } else {
                    return false;
                }
            } else {
                WayPoint* waypt_i = semi_circle_path[0];
                if(getEuclideanDistance(current_pose->x, current_pose->y, waypt_i->x, waypt_i->y) < look_ahead_dist){
                    return true;
                }
                return false;
            }
        }
        bool pathFinished(){
            WayPoint*& last_waypoint = semi_circle_path.back();
            if(getEuclideanDistance(robotPosX(), robotPosY(), last_waypoint->x, last_waypoint->y) < 0.001){
                return true;
            } else {
                return false;
            }
        }
        /**
        WayPoint* get_closest_path_point(){
            double smallest = get_euclidean_distance(robotPosX(), robotPosY(), figure8_path[0]->x, figure8_path[0]->y);
            WayPoint* closest = figure8_path[0];
            for(WayPoint* waypt: figure8_path){
                double diff = get_euclidean_distance(robotPosX(), robotPosY(), waypt->x, waypt->y);
                if(smallest > diff){
                    smallest = diff;
                    closest = waypt;
                    // if(smallest < look_ahead_dist){
                    //     on_path = true;
                    // } else {
                    //     on_path = false;
                    // }
                }
            }
            
            return closest;
        }*/
        WayPoint* getClosestPathPoint(){
            if(last_visited_waypt_idx != nullptr){
                return semi_circle_path[*next_waypt_idx];
            } else {
                return semi_circle_path[0];
            }
        }

        WayPoint* getGoalptLookAheadDistanceAway(){
            // use a lookahead distance to select from waypoints
            cout<<"gets here"<<endl;
            Point* current_pose = new Point(robotPosX(), robotPosY());
            if(last_visited_waypt_idx != nullptr && last_goalpt != nullptr){
                cout<<"gets in the for loop"<<endl;
                for(int i = *last_visited_waypt_idx; i < semi_circle_path.size()-1; i++){
                    WayPoint* waypt_i = semi_circle_path[i];
                    WayPoint* waypt_next = semi_circle_path[i+1];
                    // if(betweenTwoptsOnALine(waypt_i, waypt_next, current_pose)){
                    //     *last_visited_waypt_idx = i;    
                    // }
                    double upper_bd_dist = getEuclideanDistance(current_pose->x, current_pose->y, waypt_next->x, waypt_next->y);
                    if(upper_bd_dist >= look_ahead_dist){
                        *last_visited_waypt_idx = i;
                        *next_waypt_idx = i+1;
                        double slope = (waypt_next->y - waypt_i->y) / (waypt_next->x - waypt_i->x);
                        double x_m = waypt_i->x + sign(waypt_next->x - waypt_i->x) * look_ahead_dist * sqrt(1/(1+pow(slope, 2)));
                        double y_m = waypt_i->y + sign(waypt_next->y - waypt_i->y) * look_ahead_dist * slope * sqrt(1/(1+pow(slope, 2)));
                        WayPoint* interpolated_waypt = new WayPoint(x_m, y_m, 0.0);
                        return interpolated_waypt;
                    } else {
                        continue;
                    }
                }
                WayPoint* finisher_waypt = semi_circle_path.back();
                return finisher_waypt;
        
            } else {
                cout<<"it is in else"<<endl;
                if(onPath()){
                    cout<<"is it here?"<<endl;
                    last_visited_waypt_idx = new int(0);
                    next_waypt_idx = new int(1);
                    return semi_circle_path[0];
                }
            }
            cout<<"oh its null"<<endl;
            return nullptr;
            

        }

        double getDesiredCurvature(WayPoint* goalpt){
            double curvature = 2*(goalpt->x - robotPosX())/ pow(look_ahead_dist, 2);
            return curvature;
        }
        double getCurrentCurvature(WayPoint* goalpt){
            double dist = getEuclideanDistance(robotPosX(), robotPosY(), goalpt->x, goalpt->y);
            return 2*(goalpt->x - robotPosX())/ pow(dist, 2);
        }

        void basic_pursuit(double tolerance){
            Rate my_rate(10);
            geometry_msgs::Twist twist_msg;
            double forward_speed = 0.4;
            twist_msg.linear.x = forward_speed;
            twist_msg.linear.y = 0;
            twist_msg.linear.z = 0;

            twist_msg.angular.x = 0;
            twist_msg.angular.y = 0;
            
            spinOnce();
            while(!pathFinished()){
                if(onPath()){
                    cout<<"okay here"<<endl;
                    WayPoint* goalpt = getGoalptLookAheadDistanceAway();
                    cout<<"good here"<<endl;
                    double desired_curvature = getDesiredCurvature(goalpt);
                    cout<<"okay 2"<<endl;
                    Rate my_rate2(100);
                    while(getEuclideanDistance(robotPosX(), robotPosY(), goalpt->x, goalpt->y > 0.001)){
                        double current_curvature = getCurrentCurvature(goalpt);
                        double integral_curv = 0.0, derivative_curv = 0.0, lastError_curv = 0.0;
                        double Kp = 2/pow(look_ahead_dist,2);
                        double Ki = 0.01;
                        double Kd = 0.06;

                        // PD control for steering;
                        double error = goalpt->x - robotPosX();
                        integral_curv = integral_curv + error;
                        derivative_curv = error - lastError_curv;
                        lastError_curv = error;
                        // double turn = Kp*error_yaw + Ki*integral_yaw + Kd*derivative_yaw;
                        double turn = Kp*error + Kd*derivative_curv;
                        twist_msg.angular.z = turn;
                        vel_pub.publish(twist_msg);
                        spinOnce();
                        my_rate2.sleep();
                    }
                    
                    my_rate.sleep();
                }
            }
            twist_msg.angular.z = 0;
            twist_msg.linear.x = 0;
            vel_pub.publish(twist_msg);
            /**
            double desired_yaw = caculateDesiredOrientation(robotPosX, robotPosY(), waypoint, clockwise);
            if(waypoint.y - robotPosY() == 0){
                    desired_yaw = (waypoint.x - robotPosX) > 0 ? 0.0 : PI;
            }
            
            double current_yaw = quaternionToEuler().z;
            double integral_yaw = 0.0, derivative_yaw = 0.0, lastError_yaw = 0.0;
            double integral_dist = 0.0, derivative_dist = 0.0, lastError_dist = 0.0;
            double Kp = 0.2, Kp_yaw = 0.5;
            double Ki = 0.01;
            double Kd = 0.06, Kd_yaw = 0.06;
            
            cout<< "robotPosX():"<<robotPosX()<<endl;
            cout<< "robotPosY():"<<robotPosY()<<endl;
            cout<<"current_yaw:"<<current_yaw<<endl;
            cout<<"desired_yaw:"<<desired_yaw<<endl;

            Rate rate(100);
            geometry_msgs::Twist twist_msg;

            while((getEuclideanDistance(robotPosX(), robotPosY(), waypoint) > tolerance) || (desired_yaw-current_yaw > yaw_tolerance)){
                robotPosX() = odom_pose.pose.pose.position.x;
                robotPosY() = odom_pose.pose.pose.position.y;
                
                //cout<<boolalpha<<"clockwise:"<<clockwise<<endl;

                desired_yaw = caculateDesiredOrientation(robotPosX(), robotPosY(), waypoint, clockwise);
                //cout<<"desired_yaw_inside:"<<desired_yaw<<endl;
                
                current_yaw = quaternionToEuler().z;
                // if(current_yaw < 0 && !clockwise){
                //     current_yaw = PI + (PI - fabs(current_yaw));
                // }
                // if(current_yaw >0 && clockwise){
                //     current_yaw = -1 * (PI + (PI - fabs(current_yaw)));
                // }
                cout<<"current_yaw_inside:"<<current_yaw<<endl;
                cout<<"desired_yaw-current_inside:"<<desired_yaw-current_yaw<<endl;

                //PD control for velocity
                double distance_error = getEuclideanDistance(robotPosX(),robotPosY(), waypoint);
                integral_dist = integral_dist + distance_error;
                derivative_dist = distance_error - lastError_dist;
                lastError_dist = distance_error;
                // double forward_speed = Kp*distance_error + Ki*integral_dist + Kd*derivative_dist;
                double forward_speed = Kp*distance_error + Kd*derivative_dist;

                // PD control for steering;
                double error_yaw = desired_yaw - current_yaw;
                integral_yaw = integral_yaw + error_yaw;
                derivative_yaw = error_yaw - lastError_yaw;
                lastError_yaw = error_yaw;
                // double turn = Kp*error_yaw + Ki*integral_yaw + Kd*derivative_yaw;
                double turn = Kp_yaw*error_yaw + Kd_yaw*derivative_yaw;

                

                twist_msg.linear.x = forward_speed;
                twist_msg.linear.y = 0;
                twist_msg.linear.z = 0;

                twist_msg.angular.x = 0;
                twist_msg.angular.y = 0;
                twist_msg.angular.z = turn;
                vel_pub.publish(twist_msg);
                spinOnce();
                rate.sleep();
            }
            twist_msg.linear.x = 0;
            twist_msg.angular.z = 0;
            vel_pub.publish(twist_msg);
            */
        }
        void adaptive_pursuit(){
            // do adaptive pursuit
        }
};


int main(int argc, char** argv){
    PurePursuit pure_pursuit(argc, argv, "pure_pursuit_node");
    pure_pursuit.basic_pursuit(0.001);
    return 0;
}