
//
// include: vector_field.h
//
// last update: '18.xx.xx
// author: matchey
//
// memo:
//

#ifndef VECTOR_FIELD_H
#define VECTOR_FIELD_H

#include <Eigen/Core>

namespace normal_reaction_force{

	template <int _Dims>
	class VectorField{
		public:
		VectorField();
		VectorField(const double&);
		~VectorField();
		void setCellSize(const double&);
		void init(const Eigen::Vector2d&);
		Eigen::Vector2d* operator[](const int&);
		Eigen::Vector2d& operator()(const int&, const int&);
		void publish();

		private:
		int grid_dim; // grid_dimensions [個]
		double m_per_cell; // cell_size [m]
		Eigen::Vector2d val[_Dims][_Dims];
	};

} // namespace normal_reaction_force

#endif

