
//
// src: vector_field.cpp
//
// last update: '18.xx.xx
// author: matchey
//
// memo:
//

#include "normal_reaction_force/vector_field.h"

namespace normal_reaction_force{

	VectorField::VectorField()
		: grid_dim(320), m_per_cell(0.5)
	{
		val.resize(320);
		for(int i=0; i<320; ++i){
			val[i].resize(320);
		}
	}

	VectorField::VectorField(const int& grid_dim_, const double& m_per_cell_)
		: grid_dim(grid_dim_), m_per_cell(m_per_cell_)
	{
		val.resize(grid_dim_);
		for(int i=0; i<grid_dim_; ++i){
			val[i].resize(grid_dim_);
		}
	}

	VectorField::~VectorField() {}

	void VectorField::init(const Eigen::Vector2d& vec)
	{
		for(int i = 0; i < grid_dim; ++i){
			for(int j = 0; j < grid_dim; ++j){
				val[i][j] = vec;
			}
		}
	}

	// void VectorField::setCellSize(const double& m_per_cell_)
	// {
	// 	m_per_cell = m_per_cell_;
	// }

	std::vector<Eigen::Vector2d>& VectorField::operator[](const int& i){
		return val[i];
	}

	Eigen::Vector2d& VectorField::operator()(const int& i, const int& j){
		return val[i][j];
	}

} // namespace normal_reaction_force

