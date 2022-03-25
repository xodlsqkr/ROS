#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Range.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

std_msgs::Bool flag_AEB;
std_msgs::Float32 delta_range;
std_msgs::Float32 old_sonar_range;
geometry_msgs::Twist cmd_vel_msg;
nav_msgs::Odometry delta,data,old_data;

double VX,VY;

void UltraSonarCallback(const sensor_msgs::Range::ConstPtr& msg)
{
	//ROS_INFO("Sonar Seq: [%d]", msg->header.seq);
	//ROS_INFO("Sonar Range: [%f]", msg->range);
	
	if(old_sonar_range.data != msg->range)
	{
		delta_range.data = msg->range - old_sonar_range.data;
		//ROS_INFO("Delta Range : [%f]",delta_range.data);
	}
	old_sonar_range.data = msg->range;
	
	if(msg->range <= 1.0)
	{
		//ROS_INFO("AEB_Activation!!");
		flag_AEB.data = true;
	}
	else
	{
		flag_AEB.data = false;
	}
}

void CarControlCallback(const geometry_msgs::Twist& msg)
{
	//ROS_INFO("Cmd_vel : linear_x[%f]", msg.linear.x);
	cmd_vel_msg = msg;
	//ROS_INFO("cmd_vel :linear_x[%f]", cmd_vel_msg.linear.x);
}

void UltraSonarCallback1(const sensor_msgs::Range::ConstPtr& msg)
{
	//ROS_INFO("Sonar Seq: [%d]", msg->header.seq);
	//ROS_INFO("Sonar Range: [%f]", msg->range);
}

void odomCallback(const nav_msgs::Odometry& msg)
{
	ROS_INFO(" X[%.2lf]  Y[%.2lf] ", msg.pose.pose.position.x, msg.pose.pose.position.y);	
	data.pose.pose.position.x = msg.pose.pose.position.x;
	data.pose.pose.position.y = msg.pose.pose.position.y;
	
	if(old_data.pose.pose.position.x != data.pose.pose.position.x){
		delta.pose.pose.position.x = data.pose.pose.position.x - old_data.pose.pose.position.x;
	}
	if(old_data.pose.pose.position.y != data.pose.pose.position.y){
		delta.pose.pose.position.y = data.pose.pose.position.y - old_data.pose.pose.position.y;
	}
	
	old_data.pose.pose.position.x = data.pose.pose.position.x;
	old_data.pose.pose.position.y = data.pose.pose.position.y;
	
	VX = delta.pose.pose.position.x / 0.02;
	VY = delta.pose.pose.position.y / 0.02;
	
	ROS_INFO("vx = [%.2lf], vy = [%.2lf] ",VX,VY);
}

int main(int argc, char **argv)
{
	int count = 0;
	old_sonar_range.data = 0;
	ros::init(argc, argv, "aeb_controller");
	std::string odom_sub_topic = "/ackermann_steering_controller/odom";
	
	ros::NodeHandle n;
	ros::Rate loop_rate(10); //10
	
	ros::Subscriber sub_odom = n.subscribe(odom_sub_topic, 10,&odomCallback); 
	ros::Subscriber sub = n.subscribe("range", 1000,UltraSonarCallback);
	ros::Subscriber sub1 = n.subscribe("/RangeSonar1", 1000,UltraSonarCallback1);
	ros::Subscriber cmd_vel_sub = n.subscribe("/cmd_vel", 10,CarControlCallback);
	
	
	ros::Publisher pub =  n.advertise<std_msgs::Bool>("/aeb_activation_flag",1);
	ros::Publisher pub_cmd_vel =  n.advertise<geometry_msgs::Twist>("/ackermann_steering_controller/cmd_vel",10);
	
	while(ros::ok())
	{
	  if((count%10)==0)
		{
			pub.publish(flag_AEB);
		}
		if(flag_AEB.data == true)
		{
			cmd_vel_msg.linear.x = 0;
			pub_cmd_vel.publish(cmd_vel_msg);
		}
		else
		{
			pub_cmd_vel.publish(cmd_vel_msg);
		}
		
		loop_rate.sleep();
		ros::spinOnce();
		++count;
	}
	return 0;
}
