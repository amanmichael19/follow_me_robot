#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <queue>
#include <cmath>
#include <string>
#include <tf/tf.h>




using namespace std;
using namespace ros;
/**
Point object to represent cartesian points.
*/
struct Point {
    double x;
    double y;
    Point(): x(0.0), y(0.0){}
    Point(double x_in, double y_in): x(x_in), y(y_in){}
};
/**
Waypoint object representing path points.
*/
struct WayPoint : public Point {
    double curvature;
    bool visited;
    WayPoint(double x_in, double y_in, double curv_in = 0.0): Point(x_in, y_in), curvature(curv_in), visited(false){}
};

/**
PIDcontroller object with method to maintain following distance
*/
struct PIDcontroller {
  private:
     double _Kp;
     double _Ki;
     double _Kd;
     double _dt;
     double _integral = 0;
     double _derivative;
     double _last_error = 0;
     double _error;
  public:
     PIDcontroller(double Kp, double Ki, double Kd, double dt): _Kp(Kp), _Ki(Ki), _Kd(Kd), _dt(dt){}
     double maintainFollowingDistance(double expected, double current){
        _error = -1* (expected - current);
        _integral += _error * _dt;
        _derivative = (_error - _last_error) / _dt;
        _last_error = _error;
        return _Kp * _error + _Ki * _integral + _Kd * _derivative;
     }
};

/**
PurePursuit object
*/

class PurePursuit{
    private:
      nav_msgs::Path _generated_path;
      vector<WayPoint*> _figure8_path;
      vector<WayPoint*> _semi_circle_path;
      vector<WayPoint*> _polygon_path;
      double _look_ahead_dist = 0.5;
      Publisher _vel_pub;
      Subscriber _pose_sub;
      Subscriber _path_sub;
      nav_msgs::Odometry _odom_input_data;
      nav_msgs::Path _path_to_publish;
      int* _last_visited_waypt_idx = nullptr;
      int* _next_waypt_idx = nullptr;
      bool _no_run = false;
      double _following_dist = 0.7;


    public:
        PurePursuit(NodeHandle& n){
            _path_sub = n.subscribe<nav_msgs::Path>("/generated_path", 10, &PurePursuit::pathCallback, this);
            _vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
            _pose_sub = n.subscribe<nav_msgs::Odometry>("/odom", 10, &PurePursuit::poseCallback, this);
            changePathToWayPointObject();
            formSemiCirclePath();
            formPolygonPath();
            runPursuit();
        }

        /**
        callbacks
        */
        void pathCallback(const nav_msgs::Path::ConstPtr& path_msg){
            _generated_path = *path_msg;
        }

        void poseCallback(const nav_msgs::Odometry::ConstPtr& odom_msg){
            _odom_input_data = *odom_msg;
        }

        /**
        * function where the pursuit is run from
        */
        void runPursuit(){
            if(!_no_run){
                basic_pursuit();
            } else {
                ROS_ERROR("[ERROR]: can't start. there is error");
                return;
            }
        }

        /**
        *
        * Helper functions start here
        *
        */

        //changing to waypoint object since every thing was done using that object at first so
        // instead of changing everything, I have this function to change from PoseStamped object
        // to WayPoint object
        void changePathToWayPointObject(){
            Rate loop_rate(5);
            while(_generated_path.poses.size() == 0){
                spinOnce();
                loop_rate.sleep();
            }
            if(_generated_path.poses.size() != 0){
                for(geometry_msgs::PoseStamped path_point: _generated_path.poses){
                    _figure8_path.push_back(new WayPoint(path_point.pose.position.x, path_point.pose.position.y, path_point.pose.position.z));
                }
            } else {
                _no_run = true;
                ROS_ERROR("[ERROR]: No generated path");
                return;
            }
        }

        void formPolygonPath(){
            _polygon_path.push_back(new WayPoint(0.0, 0.0, 0.0));
            _polygon_path.push_back(new WayPoint(2.0, 2.0, 0.0));
            _polygon_path.push_back(new WayPoint(4.0, 2.0, 0.0));
            _polygon_path.push_back(new WayPoint(5.0, 5.0, 0.0));
            _polygon_path.push_back(new WayPoint(7.0, 4.0, 0.0));
        }
        void formSemiCirclePath(){
            double x = 0.0, y = 0.0;
            for(double i=3.1415926; i > -0.1; i-=0.1570796){
                x = 3 + 3*cos(i);
                y = 3*sin(i);
                _semi_circle_path.push_back(new WayPoint(x, y, 1/5));
            }
        }
        double quaternionToEulerYaw(){
            tf::Quaternion quat;
            tf::quaternionMsgToTF(_odom_input_data.pose.pose.orientation, quat);
            double roll, pitch, yaw;
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            return yaw;
        }
        template<typename T>
        int sign(T val){
            return val >= 0 ? 1 : -1;
        }
        double robotPosX(){
            return _odom_input_data.pose.pose.position.x;
        }
        double robotPosY(){
            return _odom_input_data.pose.pose.position.y;
        }
        geometry_msgs::Pose currentRobotPose(){
            return _odom_input_data.pose.pose;
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
        /**
        * given a point q in a plane gets a point p on a line l so that the line
        * qp is perpendicular to l
        */
        template<typename T>
        Point* getPtOnLinePerpendicularToAPt(double slope, WayPoint* waypt, T* pt){
            double slope_2;
            double intersection_x;
            double intersection_y;
            if(fabs(slope) > 0.001){
              slope_2 = -1 / slope;
              double intercept_1 = waypt->y - slope*waypt->x;
              double intercept_2 = pt->y - slope_2*pt->x;
              intersection_x = slope*(intercept_2 - intercept_1)/(pow(slope,2)+1);
              intersection_y = slope*intersection_x + intercept_1;
            } else{
              intersection_x = pt->x;
              intersection_y = waypt->y;
            }
            return new Point(intersection_x, intersection_y);
        }
        /**
        checks is a point is on a line between two other points on that line
        */
        template <typename T>
        bool betweenTwoptsOnALine(WayPoint* a, WayPoint* b, T* pt){
            double a_pt_dist = getEuclideanDistance(a->x, a->y, pt->x, pt->y);
            double pt_b_dist = getEuclideanDistance(pt->x, pt->y, b->x, b->y);
            double a_b_dist = getEuclideanDistance(a->x, a->y, b->x, b->y);
            if(a_pt_dist + pt_b_dist - a_b_dist < 0.001){
                return true;
            } else {
                return false;
            }
        }
        /**
        gets intersection point between a circle and a line
        */
        Point getIntersectionPtCircleLine(double slp, WayPoint* waypt_i, WayPoint* waypt_next, geometry_msgs::Point ctr){
            //cout<<"slp: "<< slp<<endl;

            double ipt = waypt_i->y - slp*waypt_i->x;
            double r = _look_ahead_dist;
            Point intersection_pt1;
            Point intersection_pt2;
            //cout<<"ipt: "<< ipt<<endl;
            double x_numerator1 = sqrt(-1*pow(ipt,2) - 2*ipt*slp*ctr.x + 2*ipt*ctr.y - pow(slp*ctr.x, 2) + pow(slp*r, 2) + 2*slp*ctr.x*ctr.y - pow(ctr.y,2) + pow(r,2));
            double x_numerator2 =  -1*ipt*slp + slp*ctr.y + ctr.x;
            double x_denominator = pow(slp,2) + 1;

            // cout<<"x_numerator1: "<<x_numerator1<<endl;
            // cout<<"x_numerator2: "<<x_numerator2<<endl;
            // cout<<"ctr: "<<ctr.x<<","<<ctr.y<<endl;

            intersection_pt1.x = (x_numerator1 + x_numerator2) / x_denominator;
            intersection_pt2.x = (-1*x_numerator1 + x_numerator2)/ x_denominator;
            intersection_pt1.y = slp*intersection_pt1.x + ipt;
            intersection_pt2.y = slp*intersection_pt2.x + ipt;

            // cout<<"intersection_pt1"<<intersection_pt1.x<<" , "<<intersection_pt1.y<<endl;
            // cout<<"intersection_pt2"<<intersection_pt2.x<<" , "<<intersection_pt2.y<<endl;

            if(betweenTwoptsOnALine(waypt_i, waypt_next, &intersection_pt1)){
                return intersection_pt1;
            } else {
                return intersection_pt2;
            }

        }
        /**
        gets relative coordinates
        */
        geometry_msgs::Point getRelativeCoordinate(geometry_msgs::Point point_msg, geometry_msgs::Pose current_pose){
            tf::Transform inverse;
            tf::poseMsgToTF(current_pose, inverse);
            tf::Transform transform = inverse.inverse();
            tf::Point p;
            pointMsgToTF(point_msg, p);
            tf::Point tf_p = transform * p;
            geometry_msgs::Point tf_point_msg;
            pointTFToMsg(tf_p, tf_point_msg);

            // cout<<"tf_point_msg: "<<tf_point_msg.x<<" , "<<tf_point_msg.y<<endl;
            // cout<<"yaw: "<<quaternionToEulerYaw()<<endl;

            return tf_point_msg;
        }
        /**
        *
        * Helper functions end
        *
        */

        /**
        * checks is path has finished
        */
        bool pathFinished(){
            WayPoint*& last_waypoint = _figure8_path.back();
            bool waypt_before_last_visited = _figure8_path[_figure8_path.size()-2]->visited;

            if((getEuclideanDistance(robotPosX(), robotPosY(), last_waypoint->x, last_waypoint->y) < 0.01) && waypt_before_last_visited){
                return true;
            } else {
                return false;
            }
        }

        /**
        * Check if the robot is on path
        *
        */
        bool onPath(){
            geometry_msgs::Point current_pose = currentRobotPose().position;
            if(_last_visited_waypt_idx != nullptr){
                //cout<<"its here"<<endl;
                setLastVisitedWaypt();
                WayPoint* waypt_i = _figure8_path[*_last_visited_waypt_idx];
                WayPoint* waypt_next = _figure8_path[*_last_visited_waypt_idx + 1];
                if(*_last_visited_waypt_idx == 19){
                    cout<<"waypt_i: "<<waypt_i -> x << ","<<waypt_i-> y<<endl;
                    cout<<"waypt_next: "<<waypt_next -> x << ","<<waypt_next-> y<<endl;
                }
                // cout<<"waypt_i: "<<waypt_i -> x << ","<<waypt_i-> y<<endl;
                // cout<<"waypt_next: "<<waypt_next -> x << ","<<waypt_next-> y<<endl;
                // cout<<"last_visited_idx: "<<*_last_visited_waypt_idx<<endl;
                // cout<<"current_pose: "<<current_pose.x<<" , "<<current_pose.y<<endl;

                double slope = (waypt_next->y - waypt_i->y) / (waypt_next->x - waypt_i->x);
                Point* pt_on_line = getPtOnLinePerpendicularToAPt(slope, waypt_i, &current_pose);
                if(betweenTwoptsOnALine(waypt_i, waypt_next, pt_on_line) && getShortestDistToLine(slope, waypt_i, &current_pose) < _look_ahead_dist){
                    return true;
                } else {
                    return false;
                }
            } else {
                WayPoint* waypt_i = _figure8_path[0];
                if(getEuclideanDistance(current_pose.x, current_pose.y, waypt_i->x, waypt_i->y) < _look_ahead_dist){
                    return true;
                }
                return false;
            }
        }

        WayPoint* getClosestPathPoint(){
            if(_last_visited_waypt_idx != nullptr){
                return _figure8_path[*_next_waypt_idx];
            } else {
                return _figure8_path[0];
            }
        }



        void setLastVisitedWaypt(){
            geometry_msgs::Point current_pose = currentRobotPose().position;
            if(_last_visited_waypt_idx != nullptr)
            {
                for(int i = *_last_visited_waypt_idx; i < _figure8_path.size()-1; i++)
                {
                    WayPoint* waypt_i = _figure8_path[i];
                    WayPoint* waypt_next = _figure8_path[i+1];
                    double slope = (waypt_next->y - waypt_i->y) / (waypt_next->x - waypt_i->x);
                    // cout<<"**setLastVisisted**"<<endl;
                    // cout<<"waypt_i: "<<waypt_i->x<<","<<waypt_i->y<<endl;
                    // cout<<"slope: "<<slope<<endl;
                    Point* pt_on_line = getPtOnLinePerpendicularToAPt(slope, waypt_i, &current_pose);
                    // //cout<<"does this"<<endl;
                    // cout<<"pt_on_line: "<<pt_on_line->x<<","<<pt_on_line->y<<endl;
                    // cout<<"current_pose: "<<current_pose.x<<","<<current_pose.y<<endl;
                    // cout<<"***"<<endl;
                    if(betweenTwoptsOnALine(waypt_i, waypt_next, pt_on_line)){
                        //cout<<"sets the visited idx"<<endl;
                        *_last_visited_waypt_idx = i;
                        _figure8_path[i]->visited = true;
                        break;
                    }
                }
            }
            else
            {
                _last_visited_waypt_idx = new int(0);
                _figure8_path[*_last_visited_waypt_idx]->visited = true;
                _next_waypt_idx = new int(1);
            }

        }
        WayPoint* getGoalptLookAheadDistanceAway(bool is_on_path){
            // use a lookahead distance to select from waypoints
            cout<<"gets here"<<endl;
            geometry_msgs::Point current_pose = currentRobotPose().position;
            if(is_on_path){
                setLastVisitedWaypt();
                cout<<"last_visited_waypt: "<<*_last_visited_waypt_idx<<endl;
                if(_last_visited_waypt_idx != nullptr){
                    cout<<"gets in the for loop"<<endl;
                    for(int i = *_last_visited_waypt_idx; i < _figure8_path.size(); i++){
                        if(i < _figure8_path.size()-1){
                            WayPoint* waypt_i = _figure8_path[i];
                            WayPoint* waypt_next = _figure8_path[i+1];

                            cout<<"Waypt_i: "<<waypt_i -> x << ","<<waypt_i-> y<<endl;
                            cout<<"Waypt_next: "<<waypt_next -> x << ","<<waypt_next-> y<<endl;

                            double upper_bd_dist = getEuclideanDistance(current_pose.x, current_pose.y, waypt_next->x, waypt_next->y);
                            cout<<"upper_bd_dist: "<<upper_bd_dist<<endl;
                            if(upper_bd_dist >= _look_ahead_dist){
                                *_next_waypt_idx = i+1;
                                double slope = (waypt_next->y - waypt_i->y) / (waypt_next->x - waypt_i->x);
                                Point interpolated = getIntersectionPtCircleLine(slope, waypt_i, waypt_next, current_pose);
                                cout<<"interpolated: "<<interpolated.x<<" , "<<interpolated.y<<endl;
                                WayPoint* interpolated_waypt = new WayPoint(interpolated.x, interpolated.y, 0.0);
                                return interpolated_waypt;
                            } else {
                                continue;
                            }
                        } else {
                            cout<<"gets to last waypt"<<endl;
                            *_next_waypt_idx = i;
                            return _figure8_path[i];
                        }
                    }
                } else {
                    return nullptr;
                }

            } else {
                WayPoint* closest_waypt = getClosestPathPoint();
                double slope = (closest_waypt->y - current_pose.y) / (closest_waypt->x - current_pose.x);
                WayPoint current_pose_waypt(current_pose.x, current_pose.y);
                Point interpolated = getIntersectionPtCircleLine(slope, &current_pose_waypt, closest_waypt, current_pose);
                cout<<"interpolated: "<<interpolated.x<<" , "<<interpolated.y<<endl;
                WayPoint* interpolated_waypt = new WayPoint(interpolated.x, interpolated.y, 0.0);
                return interpolated_waypt;
            }


        }

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

        void basic_pursuit(){

            geometry_msgs::Twist twist_msg;
            double forward_speed = 0.4;
            double turn = 0.0;
            double desired_curvature = 0.0;

            twist_msg.linear.x = forward_speed;
            twist_msg.linear.y = 0;
            twist_msg.linear.z = 0;

            twist_msg.angular.x = 0;
            twist_msg.angular.y = 0;

            PIDcontroller pid(0.2, 0.5, 0.01, 1/15);
            int k = 0;
            spinOnce();

            Rate my_rate(15);

            while(!pathFinished()){

                bool is_on_path = onPath();
                // Since we couldn't get Visp to work on TurtleBot camera, there is not target being
                //tracked, hence we can't say target's positon. But use the line below to maintain following
                //distance with target if that Visp works on the robot

                // double dist_from_target = getEuclideanDistance();
                // double forward_speed_increment = pid.maintainFollowingDistance(_following_dist, dist_from_target);
                // forward_speed += forward_speed_increment;

                cout<<"**** "<< k <<" ****"<<endl;
                cout<<boolalpha<<"pathfinished: "<<pathFinished()<<endl;
                cout<<boolalpha<<"is_on_path: "<<is_on_path<<endl;
                cout<<"    robot_pose: "<<robotPosX()<<","<<robotPosY()<<endl;


                if(is_on_path){

                    WayPoint* goalpt = getGoalptLookAheadDistanceAway(is_on_path);
                    if(isnan(goalpt->x) || isnan(goalpt->y) || goalpt == nullptr){
                        break;
                    }
                    desired_curvature = getCurvature(goalpt);

                    cout<<"    Goal point: "<<goalpt -> x << ","<<goalpt-> y<<endl;
                    cout<<"    dist_from_goal: "<<getEuclideanDistance(robotPosX(), robotPosY(), goalpt->x, goalpt->y)<<endl;


                    turn = desired_curvature*forward_speed;
                    cout<<"       turn: "<< turn<< endl;

                    twist_msg.angular.z = turn;

                    cout<< "desired_curvature: "<<desired_curvature<<endl;
                    cout<<"radius: "<<1/desired_curvature<<endl;
                    cout<<"euclidean dist: "<<getEuclideanDistance(robotPosX(), robotPosY(), goalpt->x, goalpt->y)<<endl;
                    cout<<"lookahead: "<<_look_ahead_dist<<endl;
                    cout<<"forward speed: "<<twist_msg.linear.x<<endl;



                } else {
                    WayPoint* goalpt = getGoalptLookAheadDistanceAway(is_on_path);
                    desired_curvature = getCurvature(goalpt);
                    turn = desired_curvature*forward_speed;
                    cout<<"       turn: "<< turn<< endl;

                    twist_msg.angular.z = turn;


                }
                ++k;
                _vel_pub.publish(twist_msg);
                spinOnce();
                my_rate.sleep();

            }
            twist_msg.angular.z = 0;
            twist_msg.linear.x = 0;
            _vel_pub.publish(twist_msg);

        }
};


int main(int argc, char** argv){
    init(argc, argv, "pure_pursuit_node");
    NodeHandle n;
    PurePursuit pure_pursuit(n);
    return 0;
}
