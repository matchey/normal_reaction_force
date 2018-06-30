
#include <ros/ros.h>
#include "normal_reaction_force/normal_reaction_force.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_normal_reaction_force");
	ros::NodeHandle n;

	normal_reaction_force::VectorField vf;

	return 0;
}

