
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
#include <visualization_msgs/Marker.h>
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
		void setHumans(const vmsgs::MarkerArray::ConstPtr&);
		void velocityConversion(State4d&);
		void velocityConversion(const State4d&, Eigen::Vector2d&);

		private:
		template<class T_src, class T_tgt>
		double distance(const T_src&, const T_tgt&);
		void constructGrid();
		void publish(); // for debug

		// subscribeとかは他のノードでやって、フィールド作るだけのクラスにするか
		// ros::Subscriber obstacle_subscriber;
		ros::NodeHandle node; // for debug
		ros::Publisher _publisher; // for debug

		int grid_dim; // grid_dimensions [個]
		double m_per_cell; // cell_size [m]
		Field field;
		double range; // max distance to obstacle with influence [m]
		double expand; // safe margin of Obstacle[m]
		double step_size; // 何秒先までみるか[s]

		pcNormalPtr obstacles;
		State4d own; // (x, y, vx, vy)
	};

} // namespace normal_reaction_force

#endif

