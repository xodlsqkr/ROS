#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Range.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

#define odom_distance 100
#define odom_frequency 50 //hz

nav_msgs::Odometry velocity, delta_pos, old_pos, pos;
geometry_msgs::Twist cmd_vel_msg;
std_msgs::Bool flag_AEB;
std_msgs::Float32 delta_range;
std_msgs::Float32 old_sonar_range;

float x, y, distance = 0;
float delta_x = 0, delta_y = 0, old_x = 0, old_y = 0;
float vx, vy;
float aeb_collision_distance = 200;

void odomCallback(const nav_msgs::Odometry& msg) {
	pos.pose.pose.position.x = msg.pose.pose.position.x;
	pos.pose.pose.position.y = msg.pose.pose.position.y;
	
	delta_pos.pose.pose.position.x = pos.pose.pose.position.x - old_pos.pose.pose.position.x;
	delta_pos.pose.pose.position.y = pos.pose.pose.position.y - old_pos.pose.pose.position.y;
	
	old_pos.pose.pose.position.x = msg.pose.pose.position.x;
	old_pos.pose.pose.position.y = msg.pose.pose.position.y;
	
	velocity.twist.twist.linear.x = delta_pos.pose.pose.position.x / odom_frequency;
	velocity.twist.twist.linear.y = delta_pos.pose.pose.position.y / odom_frequency;
	old_x = x;
	old_y = y;
	x = msg.pose.pose.position.x;
	y = msg.pose.pose.position.y;
	delta_x = x - old_x;
	delta_y = y - old_y;
	vx = delta_x * odom_frequency;
	vy = delta_y * odom_frequency;
	if (vx < 0) {
		vx *= -1;
	}

	aeb_collision_distance = 3.0 - (vx * (0.7 + 0.1) * 0.22778 * 2.5);
	
	if(x != old_x && old_x != 0) {
		if(delta_x > 0) {
			distance += delta_x;
		}
		else if(delta_x < 0) {
			distance += -delta_x;
		}
	}
	ROS_INFO("distance ( %.2f )", distance);
	
	if(distance >= aeb_collision_distance) {
		ROS_INFO("AEB_Activation");
		flag_AEB.data=true;
	}
	else {
		flag_AEB.data=false;
    }
	
	
}
void UltraSonarCallback(const sensor_msgs::Range::ConstPtr& msg) {
  //ROS_INFO("Sonar Seq : [%d]", msg->header.seq);
  //ROS_INFO("Sonar Range : [%f]", msg->range);
  if(msg->range <= 1.8) {
	  ROS_INFO("AEB Activation!!");
	  flag_AEB.data = true;
  }
  else {
	  flag_AEB.data = false;
  }
  delta_range.data = old_sonar_range.data-msg->range;
  //ROS_INFO("delta_range : [%f]", delta_range.data);
  old_sonar_range.data = msg->range;
}

void UltraSonarCallback2(const sensor_msgs::Range::ConstPtr& msg) {
  //ROS_INFO("Sonar2 Seq : [%d]", msg->header.seq);
  //ROS_INFO("Sonar2 Range : [%f]", msg->range);
}

void CarControlCallback(const geometry_msgs::Twist& msg) {
	//ROS_INFO("Cmd_vel = linear x [%f]", msg.linear.x);
	cmd_vel_msg = msg;
}


int main(int argc, char **argv) {
  int count = 0;
  old_sonar_range.data = 0;
  
  ros::init(argc, argv, "aeb_controller2");

  ros::NodeHandle n;
  
  std::string odom_sub_topic = "/ackermann_steering_controller/odom";
  
  ros::Subscriber sub = n.subscribe("/range", 1000, UltraSonarCallback);
  ros::Subscriber sonar_sub = n.subscribe("/RangeSonar1", 1000, UltraSonarCallback2);
  ros::Subscriber cmd_vel_sub = n.subscribe("/cmd_vel", 10, &CarControlCallback);
  ros::Subscriber sub_odom = n.subscribe(odom_sub_topic, 10, &odomCallback);
  
  ros::Publisher pub_cmd_vel = n.advertise<geometry_msgs::Twist>("ackermann_steering_controller/cmd_vel", 10);
  ros::Publisher pub_aeb_activation_flag = n.advertise<std_msgs::Bool>("aeb_activation_flag", 1);
  ros::Publisher pub_delta_range = n.advertise<std_msgs::Float32>("delta_range", 1);
  ros::Publisher pub_velocity = n.advertise<nav_msgs::Odometry>("/velocity", 10);
  
  ros::Rate loop_rate(10); 
  
  while(ros::ok()) {
	  if((count%10)==0) {
		 pub_aeb_activation_flag.publish(flag_AEB);
	  }
	  
	  if(flag_AEB.data == true) {
		 cmd_vel_msg.linear.x = 0;
		 pub_cmd_vel.publish(cmd_vel_msg);
	  }
	  
	  else {
		 pub_cmd_vel.publish(cmd_vel_msg);
	  }
	  pub_delta_range.publish(old_sonar_range);
	  pub_velocity.publish(velocity);
	  loop_rate.sleep();
	  ros::spinOnce();	
	  count++;
  }
  return 0;
}
