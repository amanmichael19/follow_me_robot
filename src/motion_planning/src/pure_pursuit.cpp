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

struct Point {
    double x;
    double y;
    Point(): x(0.0), y(0.0){}
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
    double look_ahead_dist = 1;
    Publisher vel_pub;
    Subscriber pose_sub;
    nav_msgs::Odometry odom_input_data;
    int* last_visited_waypt_idx = new int(0);
    int* next_waypt_idx = new int(1);
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
        geometry_msgs::Pose currentRobotPose(){
            return odom_input_data.pose.pose;
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
        template<typename T>
        double getShortestDistToLine(double slope, WayPoint* waypt, T* pt){
            double intercept = waypt->y - slope*waypt->x;
            double numerator = fabs(pt->y - (slope*pt->x) - intercept);
            double dist = numerator / sqrt(1 + pow(slope, 2));
            return dist;
        }
        template<typename T>
        Point* getPtOnLinePerpendicularToAPt(double slope, WayPoint* waypt, T* pt){
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
            geometry_msgs::Point current_pose = currentRobotPose().position;
            if(last_visited_waypt_idx != nullptr){
                cout<<"its here"<<endl;
                WayPoint* waypt_i = semi_circle_path[*next_waypt_idx - 1];
                WayPoint* waypt_next = semi_circle_path[*next_waypt_idx];
                cout<<"waypt_i: "<<waypt_i -> x << ","<<waypt_i-> y<<endl;
                cout<<"waypt_next: "<<waypt_next -> x << ","<<waypt_next-> y<<endl;
                cout<<"last_visited_idx: "<<*last_visited_waypt_idx<<endl;
                cout<<"current_pose: "<<current_pose.x<<" , "<<current_pose.y<<endl;
                double slope = (waypt_next->y - waypt_i->y) / (waypt_next->x - waypt_i->x);
                Point* pt_on_line = getPtOnLinePerpendicularToAPt(slope, waypt_i, &current_pose);
                if(getShortestDistToLine(slope, waypt_i, &current_pose) < look_ahead_dist){
                    return true;
                } else {
                    return false;
                }
            } else {
                WayPoint* waypt_i = semi_circle_path[0];
                if(getEuclideanDistance(current_pose.x, current_pose.y, waypt_i->x, waypt_i->y) < look_ahead_dist){
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
        geometry_msgs::Point getRelativeCoordinate(geometry_msgs::Point point_msg, geometry_msgs::Pose current_pose){
            tf::Transform inverse;
            tf::poseMsgToTF(current_pose, inverse);
            tf::Transform transform = inverse.inverse();
            tf::Point p;
            pointMsgToTF(point_msg, p);
            tf::Point tf_p = transform * p;
            geometry_msgs::Point tf_point_msg;
            pointTFToMsg(tf_p, tf_point_msg);
            cout<<"tf_point_msg: "<<tf_point_msg.x<<" , "<<tf_point_msg.y<<endl;
            cout<<"yaw: "<<quaternionToEulerYaw()<<endl;
            return tf_point_msg;
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

        Point getIntersectionPtCircleLine(double slp, WayPoint* waypt_i, WayPoint* waypt_next, geometry_msgs::Point ctr){
            double ipt = waypt_i->y - slp*waypt_i->x;
            double r = look_ahead_dist;
            Point intersection_pt1;
            Point intersection_pt2;

            double x_numerator1 = sqrt(-1*pow(ipt,2) - 2*ipt*slp*ctr.x + 2*ipt*ctr.y - pow(slp*ctr.x, 2) + pow(slp*r, 2) + 2*slp*ctr.x*ctr.y - pow(ctr.y,2) + pow(r,2));
            double x_numerator2 =  -1*ipt*slp + slp*ctr.y + ctr.x;
            double x_denominator = pow(slp,2) + 1;
            intersection_pt1.x = (x_numerator1 + x_numerator2) / x_denominator;
            intersection_pt2.x = (-1*x_numerator1 + x_numerator2)/ x_denominator;
            intersection_pt1.y = slp*intersection_pt1.x + ipt;
            intersection_pt2.y = slp*intersection_pt2.x + ipt;
            if(betweenTwoptsOnALine(waypt_i, waypt_next, &intersection_pt1)){
                return intersection_pt1;
            } else {
                return intersection_pt2;
            } 

        }
        void setLastVisitedWaypt(){
            geometry_msgs::Point current_pose = currentRobotPose().position;
            for(int i = *last_visited_waypt_idx; i < semi_circle_path.size()-1; i++){
                WayPoint* waypt_i = semi_circle_path[i];
                WayPoint* waypt_next = semi_circle_path[i+1];
                double slope = (waypt_next->y - waypt_i->y) / (waypt_next->x - waypt_i->x);
                Point* pt_on_line = getPtOnLinePerpendicularToAPt(slope, waypt_i, &current_pose);
                //cout<<"does this"<<endl;
                if(betweenTwoptsOnALine(waypt_i, waypt_next, pt_on_line)){
                    //cout<<"does this too"<<endl;
                    *last_visited_waypt_idx = i;
                    break;
                }
              
            }
        }
        WayPoint* getGoalptLookAheadDistanceAway(){
            // use a lookahead distance to select from waypoints
            cout<<"gets here"<<endl;
            geometry_msgs::Point current_pose = currentRobotPose().position;
            setLastVisitedWaypt();
            if(last_visited_waypt_idx != nullptr){
                cout<<"gets in the for loop"<<endl;
                for(int i = *last_visited_waypt_idx; i < semi_circle_path.size()-1; i++){
                    WayPoint* waypt_i = semi_circle_path[i];
                    WayPoint* waypt_next = semi_circle_path[i+1];
                    // if(betweenTwoptsOnALine(waypt_i, waypt_next, current_pose)){
                    //     *last_visited_waypt_idx = i;    
                    // }
                    double upper_bd_dist = getEuclideanDistance(current_pose.x, current_pose.y, waypt_next->x, waypt_next->y);
                    if(upper_bd_dist >= look_ahead_dist){
                        *next_waypt_idx = i+1;
                        double slope = (waypt_next->y - waypt_i->y) / (waypt_next->x - waypt_i->x);
                        Point interpolated = getIntersectionPtCircleLine(slope, waypt_i, waypt_next, current_pose);
                        // double x_m = current_pose.x + sign(waypt_next->x - waypt_i->x) * look_ahead_dist * sqrt(1/(1+pow(slope, 2)));
                        // double y_m = current_pose.y + sign(waypt_next->y - waypt_i->y) * look_ahead_dist * slope * sqrt(1/(1+pow(slope, 2)));
                        WayPoint* interpolated_waypt = new WayPoint(interpolated.x, interpolated.y, 0.0);
                        return interpolated_waypt;
                    } else {
                        continue;
                    }
                }
                WayPoint* finisher_waypt = semi_circle_path.back();
                return finisher_waypt;
        
            } 
            // else {
            //     cout<<"it is in else"<<endl;
            //     if(onPath()){
            //         cout<<"is it here?"<<endl;
            //         last_visited_waypt_idx = new int(0);
            //         next_waypt_idx = new int(1);
            //         return semi_circle_path[0];
            //     }
            // }
            return nullptr;
            

        }

        // double getDesiredCurvature(WayPoint* goalpt){
        //     double curvature = 2*(goalpt->x - robotPosX())/ pow(look_ahead_dist, 2);
        //     return curvature;
        // }
        double getCurvature(WayPoint* goalpt){
            double curvature = 0;
            geometry_msgs::Point goal_point;
            goal_point.x = goalpt->x;
            goal_point.y = goalpt->y;
            goal_point.z = 0;
            geometry_msgs::Point tf_goal_point = getRelativeCoordinate(goal_point, currentRobotPose());
            double dist = getEuclideanDistance(robotPosX(), robotPosY(), goalpt->x, goalpt->y);
            if(dist > 0.0001){
                cout<<"tf_goalpt_y: "<< tf_goal_point.y<<endl;
                curvature = 2*(tf_goal_point.y)/ pow(dist, 2);
            } 
            return isnan(curvature) ? 0 : curvature;
        }

        void basic_pursuit(double tolerance){
            
            geometry_msgs::Twist twist_msg;
            double forward_speed = 0.4;
            twist_msg.linear.x = forward_speed;
            twist_msg.linear.y = 0;
            twist_msg.linear.z = 0;

            twist_msg.angular.x = 0;
            twist_msg.angular.y = 0;
            int k = 0;
            spinOnce();
            bool finished = false;
            Rate my_rate(3);
            //while(!pathFinished()){
            while(!finished){
                //if(onPath()){
                if(1){
                    cout<<"**** "<< k <<" ****"<<endl; 
                    WayPoint* goalpt = getGoalptLookAheadDistanceAway();
                    last_goalpt = goalpt;
                    double desired_curvature = getCurvature(goalpt);
                    // double integral_curv = 0.0, derivative_curv = 0.0, lastError_curv = 0.0;
                    // double Kp = 2/pow(look_ahead_dist,2);
                    // double Ki = 0.01;
                    // double Kd = 0.06;
                        cout<<"    Goal point: "<<goalpt -> x << ","<<goalpt-> y<<endl;
                        cout<<"    dist_from_goal: "<<getEuclideanDistance(robotPosX(), robotPosY(), goalpt->x, goalpt->y)<<endl;
                        cout<<"    robot_pose: "<<robotPosX()<<","<<robotPosY()<<endl;

                    // double error = goalpt->y - robotPosY();
                    // integral_curv = integral_curv + error;
                    // derivative_curv = error - lastError_curv;
                    // lastError_curv = error;
                    // double turn = Kp*error_yaw + Ki*integral_yaw + Kd*derivative_yaw;
                    double turn = desired_curvature*forward_speed;
                    cout<<"       turn: "<< turn<< endl;
                   
                    twist_msg.angular.z = turn;
                    vel_pub.publish(twist_msg);
                    spinOnce();  
                    //cout<< "Goal point: "<<goalpt -> x << ","<<goalpt-> y<<endl;
                    //cout<< "curvature: "<<desired_curvature<<endl;
                    
               
                    cout<< "desired_curvature: "<<desired_curvature<<endl;
                    cout<<"radius: "<<1/desired_curvature<<endl;
                    cout<<"euclidean dist: "<<getEuclideanDistance(robotPosX(), robotPosY(), goalpt->x, goalpt->y)<<endl;
                    cout<<"lookahead: "<<look_ahead_dist<<endl;
                    cout<<"forward speed: "<<twist_msg.linear.x<<endl;
                    ++k;
              
                    my_rate.sleep();
                } else {
                    twist_msg.angular.z = 0;
                    twist_msg.linear.x = 0;
                    vel_pub.publish(twist_msg);
                    finished = true;
                }
                
            }
            twist_msg.angular.z = 0;
            twist_msg.linear.x = 0;
            vel_pub.publish(twist_msg);
            
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