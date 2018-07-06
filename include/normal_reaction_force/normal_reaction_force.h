
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
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace normal_reaction_force{
	typedef pcl::PointNormal PointN;
	typedef pcl::PointCloud<PointN> pcNormal;
	typedef pcNormal::Ptr pcNormalPtr;

	struct State4d{
		Eigen::Vector2d position;
		Eigen::Vector2d velocity;
	};

	class VectorField{
		public:
		VectorField();
		~VectorField();
		void setObstacles(const pcNormalPtr&);
		void velocityConversion(State4d&);
		void velocityConversion(const State4d&, Eigen::Vector2d&);

		private:
		// void callback();
		bool isOnLine(const PointN&); // in setObsOnLine()
		void setObsOnLine(pcNormalPtr&); // in clustering()
		void clustering(); // in velocityConversion()

		// subscribeとかは他のノードでやって、フィールド作るだけのクラスにするか
		// ros::Subscriber obstacle_subscriber;
		ros::NodeHandle node; // for debug
		ros::Publisher _publisher; // for debug
		double range; // [m]
		double expand; // [m]
		double step_size; // 何秒先までみるか[s]
		pcNormalPtr obstacles;
		std::vector<State4d> clusters; // (x, y, normal_x, normal_y)
		State4d own; // (x, y, vx, vy)
	};

} // namespace normal_reaction_force

#endif

