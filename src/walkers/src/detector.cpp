#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include <cmath>

void callback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
	ROS_INFO("walker moved to %f, %f", msg->point.x, msg->point.y);
}

int main(int argc, char *argv[])
{
	// initialize ROS system.
	ros::init(argc, argv, "detector");

	// this node is "us".
	ros::NodeHandle node;

	// our subscription to the walker.
	// second arg is size of message buffer - overflow gets discarded.
	ros::Subscriber subscription =
		node.subscribe("randomwalker", 1000, callback);

	ros::spin();
}
