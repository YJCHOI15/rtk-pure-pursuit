
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Vector3.h" 
#include "geometry_msgs/Pose2D.h" 
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <math.h>


geometry_msgs::Pose2D Pose2; //Rear GPS
geometry_msgs::Pose2D Pose1; //Front GPS

std_msgs::Float32 heading_angle;

double diff_x=0.0; 
double diff_y=0.0;

void pose1Callback(const geometry_msgs::PoseStamped& msg)
{
	Pose1.y = -(double)msg.pose.position.x;
	Pose1.x = (double)msg.pose.position.y;
}

void pose2Callback(const geometry_msgs::PoseStamped& msg)
{
	Pose2.y = -(double)msg.pose.position.x;
	Pose2.x = (double)msg.pose.position.y;
}

int main(int argc, char **argv)
{
  char buf[2];
  //FILE* error_data = fopen("/home/je/gps_slam_catkin_ws/src/two_gps_h_angle/data/error_data.txt","a");
  ros::init(argc, argv, "two_gps");

  ros::NodeHandle n;
 
  

   
    ros::Subscriber subutm = n.subscribe("/utm2",10, &pose2Callback); //Rear GPS
    ros::Subscriber subutm2 = n.subscribe("/utm",10, &pose1Callback);//Front GPS
     
    ros::Publisher angle_pub = n.advertise<std_msgs::Float32>("/gps_heading_angle",1); 
 
  ros::Rate loop_rate(10);  // 10
  
  
  while (ros::ok())
  {	
	
	diff_x = Pose1.x - Pose2.x;
	diff_y = Pose1.y - Pose2.y;
	
	heading_angle.data=atan2f(diff_y,diff_x);
	

	//Pos -> Ros Coordinate
  ROS_INFO("Rear GPS x: %.7lf Y: %.7lf",Pose1.x,Pose1.y);
  ROS_INFO("Front GPS x: %.7lf Y: %.7lf",Pose2.x,Pose2.y);
  ROS_INFO("Heading Angle : %.7lf",heading_angle.data*180/3.141592);
 
	angle_pub.publish(heading_angle);
	
	loop_rate.sleep();
    ros::spinOnce();
   
  }
  return 0;
}
