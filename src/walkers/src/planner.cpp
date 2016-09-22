#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/Path.h"
#include "ompl/geometric/SimpleSetup.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/RealVectorBounds.h"
#include "ompl/base/ScopedState.h"
#include <cmath>

// the path will start and end +- this distance in X.
static float const PATH_DIST = 6.0;
static float const OBSTACLE_RADIUS = 0.3;

namespace ob = ompl::base;
namespace og = ompl::geometric;

struct Planner
{
	ros::Publisher publisher;
	ob::StateSpacePtr space;
	og::SimpleSetup ss;

	Planner() : space(new ompl::base::RealVectorStateSpace(2)), ss(space)
	{
		// initialize the path planning inputs that don't change.
		ob::RealVectorBounds bounds(2);
		bounds.setLow(-PATH_DIST * 1.01);
		bounds.setHigh(PATH_DIST * 1.01);
		space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

		ob::ScopedState<> start(space);
		ob::ScopedState<> goal(space);
		start[0] = PATH_DIST; start[1] = 0.0;
		goal[0] = -PATH_DIST; goal[1] = 0.0;
		ss.setStartAndGoalStates(start, goal);
	}

	void callback(const geometry_msgs::PoseArray::ConstPtr &msg)
	{
		// function to test if a point query from the sampling-based planner
		// is valid, i.e. it does not intersect any obstacles.
		auto is_valid = [&msg](ob::State const *state) -> bool
		{
			// OMPL's API is not so elegant.
			auto const *state_R2 = state->as<ob::RealVectorStateSpace::StateType>();
			float x = (*state_R2)[0];
			float y = (*state_R2)[1];

			// allow start and end points to be valid even if an obstacle is there,
			// to simplify the demo.
			if (std::abs(std::abs(x) - PATH_DIST) < 0.001) {
				return true;
			}

			// loop over all obstacles and check distance to the query point.
			for (auto &&pose : msg->poses) {
				geometry_msgs::Point const &p = pose.position;
				float dx = p.x - x;
				float dy = p.y - y;
				float dist = sqrt(dx * dx + dy * dy);
				if (dist < OBSTACLE_RADIUS) {
					return false;
				}
			}

			return true;
		};

		ss.clear();
		ss.setStateValidityChecker(is_valid);
		ob::PlannerStatus solved = ss.solve();
		if (solved) {
			// this makes the path more straight
			ss.simplifySolution();

			// convert the OMPL path into a ROS path
			og::PathGeometric &path = ss.getSolutionPath();
			std::vector<ob::State *> const &states = path.getStates();
			size_t const n_states = states.size();

			nav_msgs::Path ros_path;
			ros_path.header.stamp = ros::Time::now();
			ros_path.header.frame_id = "/map";
			ros_path.poses.resize(n_states);

			for (size_t i = 0; i < n_states; ++i) {
				ob::State const *state = states[i];
				auto const *state_R2 = 
					state->as<ob::RealVectorStateSpace::StateType>();
				ros_path.poses[i].pose.position.x = (*state_R2)[0];
				ros_path.poses[i].pose.position.y = (*state_R2)[1];
			}

			// publish to our topic
			publisher.publish(ros_path);
		}
	}
};

int main(int argc, char *argv[])
{
	Planner planner;

	ros::init(argc, argv, "planner");
	ros::NodeHandle node;

	planner.publisher = 
		node.advertise<nav_msgs::Path>("planner", 1000);

	ros::Subscriber subscription =
		node.subscribe("randomwalker", 1000, &Planner::callback, &planner);

	ros::spin();
}
