
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

// #include <ros/ros.h>
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

		private:
		// void callback();
		bool isOnLine(const PointN&);
		void setObsOnLine(pcNormalPtr&);
		void clustering();

		// subscribeとかは他のノードでやって、フィールド作るだけのクラスにするか
		// ros::Subscriber obstacle_subscriber;
		// ros::Publisher _publisher;
		double range; // [m]
		pcNormalPtr obstacles;
		std::vector<State4d> clusters; // (x, y, normal_x, normal_y)
		State4d own; // (x, y, vx, vy)
	};

} // namespace normal_reaction_force

#endif
