#include "ros/ros.h"
#include "geometry_msgs/Pose.h"

void chattercallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	if (msg->position.y > 4)
	 ROS_INFO("DANGER");
	else
	 ROS_INFO("SAFE");
}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"Mukesh");
	ros::NodeHandle n;
	ros::Subscriber sub=n.subscribe("greetings",1000,chattercallback);
	ros::spin();
	return 0;
}
