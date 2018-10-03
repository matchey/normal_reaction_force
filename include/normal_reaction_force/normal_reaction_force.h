
//
// include: normal_reaction_force.h
//
// last update: '18.xx.xx
// author: matchey
//
// memo:
//

#ifndef NORMAL_REACTION_FORCE_H
#define NORMAL_REACTION_FORCE_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <boost/multi_array.hpp>

namespace normal_reaction_force{

	namespace vmsgs = visualization_msgs;

	typedef pcl::PointNormal PointN;
	typedef pcl::PointCloud<PointN> pcNormal;
	typedef pcNormal::Ptr pcNormalPtr;

	typedef std::vector< std::vector<Eigen::Vector2d> > Field;

	struct State4d{
		Eigen::Vector2d position;
		Eigen::Vector2d velocity;
	};

	class VectorField{
		public:
		VectorField();
		~VectorField();
		void setObstacles(const pcNormalPtr&);
		void setDistances(const std::vector<State4d>&, const unsigned&);
		void velocityConversion(std::vector<State4d>&);

		private:
		void constructGrid(); // create vector field
		void setDirections(); // calclate directions of vector field
		void setMagnitudes(); // calclate magnitude of vector field

		void setDistances(const std::vector<State4d>&); // set distances of human to human
		double distance(const Eigen::Vector2d&, const Eigen::Vector2d&); // human[i] to human[j]
		void fill(Field&, const Eigen::Vector2d&);

		void publish(); // for debug

		ros::NodeHandle node; // for debug
		ros::Publisher _publisher; // for debug

		Field field; // normal vector field
		int grid_dim; // grid_dimensions [個]
		double m_per_cell; // cell_size [m]

		unsigned npoints;
		pcNormalPtr obstacles;
		Eigen::MatrixXd distances;
		unsigned nhumans;

		double range; // max distance to obstacle with influence [m]
		double expand; // safe margin of Obstacle[m]
		double step_size; // 何ステップ先までみるか[回]
	};

} // namespace normal_reaction_force

#endif

