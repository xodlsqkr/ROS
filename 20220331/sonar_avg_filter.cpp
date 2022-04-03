#include "ros/ros.h"
#include "sensor_msgs/Range.h"

using namespace std;

sensor_msgs::Range range;

void RangeCallBack(const sensor_msgs::Range::ConstPtr& msg) {
	
	float r1,r2,r3,r4,r5;
	r1=r2=r3=r4=r5=msg->range;
	float sum = r1+r2+r3+r4+r5;
	float range = sum/5.0;
	
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "sonar_avg_filter");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/range", 1000, RangeCallBack);
	ros::Publisher pub_avg_range = n.advertise<sensor_msgs::Range>("/range_avg", 1000);
	
	ros::Rate loop_rate(10);
	
	while(ros::ok()) 
	{
		pub_avg_range.publish(range);
		ROS_INFO("avg = [%f]",range.range);
		loop_rate.sleep();
		ros::spinOnce();
	}
	
	return 0;
	
}
