#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "std_msgs/Bool.h"

ros::Publisher publisher;
float radius = 1.0;

void callback(const std_msgs::Bool::ConstPtr &msg)
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/map";
	marker.header.stamp = ros::Time::now();

	// this ID makes rviz overwrite the old marker
	// instead of adding another marker.
	marker.ns = "circle_indicator";
	marker.id = 0;
	marker.action = visualization_msgs::Marker::ADD;

	// make the marker reflect the distance threshold of 1.0.
	marker.type = visualization_msgs::Marker::CYLINDER;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.scale.x = radius * 2;
	marker.scale.y = radius * 2;
	marker.scale.z = 0.1;

	// set the color according to whether a robot is in the center or not.
	if (msg->data) {
		// yellow
		marker.color.r = 1.0f;
		marker.color.g = 0.8f;
		marker.color.b = 0.0f;
	}
	else {
		// blue
		marker.color.r = 0.3f;
		marker.color.g = 0.1f;
		marker.color.b = 1.0f;
	}
	marker.color.a = 1.0f; // alpha channel - opacity
	
	publisher.publish(marker);
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "circle_indicator");
	ros::NodeHandle node;

	bool ok = node.getParam("center_radius", radius);
	if (!ok) {
		ROS_WARN("failed to get radius parameter.");
	}

	// advertise our marker topic.
	publisher = node.advertise<visualization_msgs::Marker>(
		"circle_indicator", 1000);

	// our subscription to the detector.
	ros::Subscriber subscription =
		node.subscribe("detector", 1000, callback);

	ros::spin();
}
