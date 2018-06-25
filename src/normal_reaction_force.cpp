
#include "normal_reaction_force/normal_reaction_force.h"

namespace normal_reaction_force{

	VectorField::VectorField()
	{
	}

	VectorField::~VectorField() {}

	//private
	// void VectorField::callback()
	// {
	// }
	bool VectorField::isOnLine(const State4d& obstacle)
	{
		// own.postion = obstacle.position + t * obstacle.velocity ?
		//
	}

	void VectorField::clustering()
	{
		// double x = own.position.x();
	}

} // namespace normal_reaction_force
