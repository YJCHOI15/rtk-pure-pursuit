#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose2D.h" 
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <sensor_msgs/Imu.h>

#define DEBUG 1

#define MAX_L_STEER -30
#define MAX_R_STEER 30
#define STEER_NEUTRAL_ANGLE 0

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI)

//waypoint를 제어하는 최소 거리 ackermann steering 에 한해서
#define MINIMUM_DISTANCE_WAYPOINT_CONTROL 1.0  

//각도가 맞을 경우 직진만 함,  Angle 이 lock 되었다고 알려줌
#define ANGLE_LOCK  2 

//거리가 가까우면 HEADING ANGLE 제어를 줄임
#define DISTANCE_ANGLE_ANGLE_LOCK 0.5
#define LOCK_CONTROL_ANGLE  3

 
double pos_x = 0.0;
double pos_y = 0.0;
double roll,pitch,yaw;
double init_yaw;
int init_flag=0;
float imu_offset = 35;
int my_target_pose_goal_id = -1 ;
double gps_heading_angle = 0;

struct Point 
{ 
	float x; 
	float y; 
	float z;
};

struct WayPoints
{
	double x;
	double y;	
} ;


double error = 0, error_old =0, error_d =0;	
double error1, error_old1, error_d1;

geometry_msgs::Pose2D my_pose,my_target_pose_goal,my_target_pose_start;

void poseCallback(const geometry_msgs::PoseStamped& msg)
{
	my_pose.x = (double)msg.pose.position.x;
	my_pose.y = (double)msg.pose.position.y;
	
	tf2::Quaternion q(
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w);
    tf2::Matrix3x3 m(q);
      
 
    m.getRPY(roll, pitch, yaw);
    my_pose.theta = yaw;		
}


void GPSHeadingAngleCallback(const std_msgs::Float32& msg)
{
	gps_heading_angle = msg.data; 
}

void gpsposeCallback(const geometry_msgs::PoseStamped& msg)
{
	my_pose.y =  (double)msg.pose.position.y;
	my_pose.x =  (double)msg.pose.position.x;
    my_pose.theta = yaw;		
}

void targetPoseCallback(const geometry_msgs::Pose2D& msg)
{
	my_target_pose_goal.y = msg.y;
	my_target_pose_goal.x = msg.x;
	my_target_pose_goal.theta = msg.theta;	
}

void targetPoseIDCallback(const std_msgs::Int16& msg)
{
    my_target_pose_goal_id = msg.data;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) 
{
      tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
      tf2::Matrix3x3 m(q);      
 
      m.getRPY(roll, pitch, yaw);
     
      yaw = yaw+ DEG2RAD(imu_offset);
      //my_pose.theta = yaw;
      //printf("%6.3lf(rad)  %6.3lf \n",yaw, yaw*180/3.14159);   
}


int heading_angle_steering_control(double c_heading_angle)
{
	int steer_angle;
	

	double pi_gain,pd_gain,p_gain;	  
	
	p_gain = 0.5;
    pd_gain = 6; 	
    
	error = c_heading_angle;
	error_d = error - error_old;
	steer_angle = (int)( p_gain * error + pd_gain * error_d  + 0.5   );		
	error_old = error;
		
	return steer_angle;	
	
}



int main(int argc, char **argv)
{
  char buf[2];
    
  ros::init(argc, argv, "cleanbot_waypoint_car_control");

  ros::NodeHandle n;
  
  ros::Subscriber sub1 = n.subscribe("/my_pose",10, &poseCallback);
  ros::Subscriber sub2 = n.subscribe("/utm_my_pose",10, &gpsposeCallback);
  
  ros::Subscriber sub3 = n.subscribe("/target_goal",10, &targetPoseCallback);
  ros::Subscriber sub4 = n.subscribe("/target_id", 1,&targetPoseIDCallback);
    
  ros::Subscriber sub5 = n.subscribe("/handsfree/imu",10,&imuCallback);
  ros::Subscriber sub6 = n.subscribe("/gps_heading_angle",1,&GPSHeadingAngleCallback);
       
  ros::Publisher car_control_pub1 = n.advertise<std_msgs::Int16>("Car_Control_cmd/W_SteerAngle_Int16", 10);
  ros::Publisher car_control_pub2 = n.advertise<std_msgs::Int16>("Car_Control_cmd/Speed_Int16", 10);
  //pub3dp tofh cnrkgka
  ros::Publisher car_control_pub3 = n.advertise<std_msgs::Float32>("Car_Control_cmd/Target_Angle", 10);
  
  ros::Rate loop_rate(5);  // 10
  
  long count = 0;
  int waypoint_arrival_flag = 0 ;
  double waypoint_distance = 0.0;
  double waypoint_angle  = 0.0;
  std_msgs::Int16 s_angle;
  
  while (ros::ok())
  {	
	/*
	my_target_pose_goal.x =  4.0;
	my_target_pose_goal.y =  1.0;
    my_pose.x =  0.0;
    my_pose.y =  1.0;
	my_pose.theta = DEG2RAD(60);	
	*/
	double waypoint_pos_base_link_x     = 0.0;
	double waypoint_pos_base_link_y     = 0.0; 
	double waypoint_pos_base_link_theta = 0.0; 
	double tf_base_map_x = 0.0,tf_base_map_y = 0.0; 
	
	tf_base_map_x = - my_pose.x;   //상대좌표로 변환  no translation
	tf_base_map_y = - my_pose.y;
	
	tf_base_map_x += my_target_pose_goal.x;   //상대좌표로 변환  no translation
	tf_base_map_y += my_target_pose_goal.y;     
    
    //printf("TF base_link to map : %lf %lf\n", tf_base_map_x,tf_base_map_y);	
    
	waypoint_pos_base_link_x = tf_base_map_x * cos(my_pose.theta)  + tf_base_map_y * sin(my_pose.theta);   // rotation_matrix
	waypoint_pos_base_link_y = -tf_base_map_x * sin(my_pose.theta) + tf_base_map_y * cos(my_pose.theta);   	
	
	waypoint_angle = atan2(waypoint_pos_base_link_y ,waypoint_pos_base_link_x);	
	waypoint_distance = sqrt(waypoint_pos_base_link_x*waypoint_pos_base_link_x  + waypoint_pos_base_link_y*waypoint_pos_base_link_y);
	
	printf("0: waypoint id : %3d , control action trial : %ld\n",my_target_pose_goal_id, count); 
	printf("1: my pose : %lf %lf %lf(deg)\n", my_pose.x,my_pose.y,RAD2DEG(my_pose.theta));	
	printf("2: goal pose : %lf %lf \n", my_target_pose_goal.x,my_target_pose_goal.y);		
	printf("3: Way point(base_link) : %lf %lf\n", waypoint_pos_base_link_x,waypoint_pos_base_link_y);
	printf("4: WayPoint Distance %6.3lf , WayPoint Angle %6.3lf \n",waypoint_distance, RAD2DEG(waypoint_angle));	
	printf("\n\n");
	
	
	printf("=========== Heading Angle Correction Motion Enable =========== \n");
	s_angle.data = heading_angle_steering_control(RAD2DEG(waypoint_angle));
	
	
	if((RAD2DEG(waypoint_angle) >=-ANGLE_LOCK) && (RAD2DEG(waypoint_angle) <=ANGLE_LOCK))
	{
		printf("-------------------------\n");
		printf("Heading Angle is locked!!\n");
		printf("-------------------------\n");
	}	
		
	s_angle.data = s_angle.data%360; // 각도 확인 할것 ROS 좌표축에 맞도록 수정했음 2022.07.10	

    ROS_INFO("Steering Angle %d",s_angle.data);
    car_control_pub1.publish(s_angle);
    printf("\n\n\n\n");
    
	loop_rate.sleep();
    ros::spinOnce();
    ++count;
  }
  return 0;
}

