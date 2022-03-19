#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Range.h"    
#include "geometry_msgs/Twist.h" 
#include "std_msgs/Int32.h"

std_msgs::Bool flag_AEB;

void UltraSonarCallback(const sensor_msgs::Range::ConstPtr& msg)
{
	//ROS_INFO("Sonar Seq: [%d]" , msg->header.seq);
	ROS_INFO("Sonar Range: [%f]" , msg->range);
	
	if(msg->range <=1.0){
		ROS_INFO("AEB_Activation!!");
		flag_AEB.data = true;
	}
	else{
		flag_AEB.data = false; 
	}
}

int main(int argc, char **argv)
{
	int count = 0;
	//std_msgs::Int32 seq;
	
	ros ::init(argc, argv, "aeb_controller");
	
	ros ::NodeHandle n;
	
	ros ::Rate loop_rate(1); // 10
	
	ros ::Subscriber sub =n.subscribe("range",1000,UltraSonarCallback);
	
	ros::Publisher pub1 = n.advertise<std_msgs::Bool>("range2", 1000);
	//ros::Publisher pub2 = n.advertise<std_msgs::Int32>("seq", 1000);
	
	
	while (ros::ok()){
		//seq.data = count;
		pub1.publish(flag_AEB);
		//pub2.publish(seq);
		loop_rate.sleep();
		ros::spinOnce();
		++count;
	}
	return 0 ;
}
