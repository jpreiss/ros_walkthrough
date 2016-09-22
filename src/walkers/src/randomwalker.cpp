#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"
#include "tf/transform_datatypes.h"
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
		node.advertise<geometry_msgs::PoseArray>("randomwalker", 1000);

	// random number generator
	std::random_device rd;
	std::default_random_engine prng(rd());
	std::normal_distribution<float> normal_dist(0, 0.01);

	int const N = 20;

	// array of 3d positions + orientations
	// we don't need orientations, but using it for rviz compatibility
	geometry_msgs::PoseArray poses;
	poses.poses.resize(N);

	// tell ROS that the poses are in the global coordiniate frame
	poses.header.frame_id = "/map";

	float vx[20] = {0};
	float vy[20] = {0};

	while (ros::ok()) {
		for (int i = 0; i < N; ++i) {
			// damping
			vx[i] *= 0.99;
			vy[i] *= 0.99;
			// attraction to origin
			vx[i] -= 0.0005 * poses.poses[i].position.x;
			vy[i] -= 0.0005 * poses.poses[i].position.y;
			// random walking
			vx[i] += normal_dist(prng);
			vy[i] += normal_dist(prng);

			// integrate velocity
			poses.poses[i].position.x += vx[i];
			poses.poses[i].position.y += vy[i];

			// make the arrow in rviz point up
			tf::Quaternion q;
			q.setRPY(0, -M_PI / 2, 0);
			tf::quaternionTFToMsg(q, poses.poses[i].orientation);
		}

		// publish the message
		poses.header.stamp = ros::Time::now();
		publisher.publish(poses);
		ros::spinOnce();

		// throttle our update rate to 30hz
		ros::Rate loop_rate(30);
		loop_rate.sleep();
	}
}
