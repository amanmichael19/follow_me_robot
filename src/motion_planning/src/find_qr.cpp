#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/PoseStamped.h>
#define PI  3.1415926535897


using namespace ros;
Publisher pub; 

bool valid_pose;
double Z;
  
ros::Publisher movement_pub;

void statusCallBack(const std_msgs::Int8ConstPtr& msg){
	
	if (msg->data == 3)
		valid_pose = true;
	else
		valid_pose = false;
}

void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg){

    ROS_INFO("Position (x): [%f]", msg->pose.position.z);

geometry_msgs::Twist move;
 if(!valid_pose)
    {
        
        
      
        move.linear.x = 0; 
        move.angular.z = 0.3;
	
        
    }

 else if(valid_pose)
    {
    
	if (msg->pose.position.z > 0.1)
        	move.linear.x = -0.07;

	else  
		move.linear.x = 0;
        move.angular.z = 0;
        
    }
movement_pub.publish(move);

    
}



int main(int argc, char **argv){

    init(argc, argv, "find_qr");
  NodeHandle nh;
movement_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",10);
ros::Subscriber sub_visp_status = nh.subscribe("/visp_auto_tracker/status", 10,statusCallBack);
ros::Subscriber sub_visp_pos   = nh.subscribe("/visp_auto_tracker/object_position", 10, poseCallback);


ros::Rate rate(10); 
   

 while(ros::ok() ){
ROS_INFO("Hello!!");
ros::spinOnce();
        rate.sleep();
}

    return 0;
}
