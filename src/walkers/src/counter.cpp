#include "ros/ros.h"
#include "walkers/counter.h"
#include <cstdint>

int64_t total = 0;

bool count(walkers::counter::Request  &req,
           walkers::counter::Response &res)
{
	++total;
	res.total = total;
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "counter_server");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("counter", count);
	ros::spin();
}
