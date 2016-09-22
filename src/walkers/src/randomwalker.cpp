#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
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
		node.advertise<geometry_msgs::PointStamped>("randomwalker", 1000);

	// random number generator
	std::random_device rd;
	std::default_random_engine prng(rd());
	std::normal_distribution<float> normal_dist(0, 0.01);

	// our position
	// we do 2D movement, but using 3D for rviz compatibility
	geometry_msgs::PointStamped position;

	// tell ROS that this point is in the global coordiniate frame
	position.header.frame_id = "/map";

	float vx = 0;
	float vy = 0;

	while (ros::ok()) {
		// damping
		vx *= 0.99;
		vy *= 0.99;
		// attraction to origin
		vx -= 0.0005 * position.point.x;
		vy -= 0.0005 * position.point.y;
		// random walking
		vx += normal_dist(prng);
		vy += normal_dist(prng);

		// integrate velocity
		position.point.x = position.point.x + vx;
		position.point.y = position.point.y + vy;

		// publish the message
		publisher.publish(position);
		ros::spinOnce();

		// throttle our update rate to 100ms
		ros::Rate loop_rate(100);
		loop_rate.sleep();
	}
}
