
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

	template<int _Dims>
	VectorField<_Dims>::VectorField()
		: grid_dim(_Dims), m_per_cell(0.5)
	{
	}

	template<int _Dims>
	VectorField<_Dims>::VectorField(const double& m_per_cell_)
		: grid_dim(_Dims), m_per_cell(m_per_cell_)
	{
	}

	template<int _Dims>
	VectorField<_Dims>::~VectorField() {}

	template<int _Dims>
	void VectorField<_Dims>::init(const Eigen::Vector2d& vec)
	{
		for(unsigned i = 0; i < grid_dim; ++i){
			for(unsigned j = 0; j < grid_dim; ++j){
				val[i][j] = vec;
			}
		}
	}

	template<int _Dims>
	void VectorField<_Dims>::setCellSize(const double& m_per_cell_)
	{
		m_per_cell = m_per_cell_;
	}

	template<int _Dims>
	Eigen::Vector2d* VectorField<_Dims>::operator[](const int& i){
		return val[i];
	}

	template<int _Dims>
	Eigen::Vector2d& VectorField<_Dims>::operator()(const int& i, const int& j){
		return val[i][j];
	}

} // namespace normal_reaction_force

