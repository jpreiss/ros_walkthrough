#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Bool.h"
#include <cmath>

struct Detector
{
	ros::Publisher publisher;
	bool prev_inside = false;

	void callback(const geometry_msgs::PointStamped::ConstPtr &msg)
	{
		// compute the distance between the robot and the origin.
		geometry_msgs::Point const &p = msg->point;
		float dist = sqrt(p.x * p.x + p.y * p.y);
		bool is_inside = dist < 1.0;

		// send the message indicating if the robot is close to the origin.
		std_msgs::Bool is_inside_msg;
		is_inside_msg.data = is_inside;
		publisher.publish(is_inside_msg);

		// log some info to the console.
		if (!prev_inside && is_inside) {
			ROS_INFO("robot entered the center.");
		}
		if (prev_inside && !is_inside) {
			ROS_INFO("robot left the center.");
		}
		prev_inside = is_inside;
	}
};

int main(int argc, char *argv[])
{
	Detector detector;

	// initialize ROS system.
	ros::init(argc, argv, "detector");

	// this node is "us".
	ros::NodeHandle node;

	// advertise our topic of Bool messages.
	// second arg is size of message buffer - overflow gets discarded.
	detector.publisher = 
		node.advertise<std_msgs::Bool>("robot_in_center", 1000);

	// our subscription to the walker.
	// second arg is size of message buffer - overflow gets discarded.
	//
	// note syntax for using an object method as a callback 
	// instead of a free function.
	//
	ros::Subscriber subscription =
		node.subscribe("randomwalker", 1000, &Detector::callback, &detector);

	ros::spin();

	// notice there's no loop rate!
}
