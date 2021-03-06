#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"
#include "std_msgs/Bool.h"
#include "walkers/counter.h"
#include <cmath>

struct Detector
{
	ros::Publisher publisher;
	ros::ServiceClient counter_client;

	bool prev_inside = false;
	float radius = 1.0;

	void callback(const geometry_msgs::PoseArray::ConstPtr &msg)
	{
		bool any_inside = false;

		for (auto &&pose : msg->poses) {
			// compute the distance between the robot and the origin.
			geometry_msgs::Point const &p = pose.position;
			float dist = sqrt(p.x * p.x + p.y * p.y);
			if (dist < radius) {
				any_inside = true;
				break;
			}
		}

		// send the message indicating if the robot is close to the origin.
		std_msgs::Bool is_inside_msg;
		is_inside_msg.data = any_inside;
		publisher.publish(is_inside_msg);

		// log some info to the console.
		if (!prev_inside && any_inside) {
			ROS_INFO("robot entered the center.");
			walkers::counter service_call;
			if (counter_client.call(service_call)) {
				ROS_INFO("total center entries: %ld", service_call.response.total);
			}
			else {
				ROS_WARN("failed to call counter service!");
			}
		}
		if (prev_inside && !any_inside) {
			ROS_INFO("all robots left the center.");
		}
		prev_inside = any_inside;
	}
};

int main(int argc, char *argv[])
{
	Detector detector;

	ros::init(argc, argv, "detector");
	ros::NodeHandle node;

	bool ok = node.getParam("center_radius", detector.radius);
	if (!ok) {
		ROS_WARN("failed to get radius parameter.");
	}

	// advertise our topic of Bool messages.
	// second arg is size of message buffer - overflow gets discarded.
	detector.publisher = 
		node.advertise<std_msgs::Bool>("detector", 1000);

	// establish the handle to the service.
	detector.counter_client = 
		node.serviceClient<walkers::counter>("counter");

	// subscribe to the walkers
	ros::Subscriber subscription =
		node.subscribe("randomwalker", 1000, &Detector::callback, &detector);

	ros::spin();
}
