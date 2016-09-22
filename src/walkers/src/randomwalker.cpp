#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include <random>

int main(int argc, char *argv[])
{
	// initialize ROS system.
	ros::init(argc, argv, "randomwalker");

	// this node is "us".
	ros::NodeHandle node;

	// advertise our topic of Pose2D messages.
	// second arg is size of message buffer - overflow gets discarded.
	ros::Publisher publisher = 
		node.advertise<geometry_msgs::Pose2D>("randomwalker", 1000);

	// random number generator
	std::random_device rd;
	std::default_random_engine prng(rd());
	std::normal_distribution<float> normal_dist;

	// our pose (x, y, angle)
	geometry_msgs::Pose2D pose;

	while (ros::ok()) {
		// update the pose with a random walk
		pose.x += normal_dist(prng);
		pose.y += normal_dist(prng);

		// publish the message
		publisher.publish(pose);
		ros::spinOnce();

		// throttle our update rate to 100ms
		ros::Rate loop_rate(100);
		loop_rate.sleep();
	}
}
