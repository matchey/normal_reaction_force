
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
		void clustering();
		bool isOnLine(const State4d&);

		// subscribeとかは他のノードでやって、フィールド作るだけのクラスにするか
		// ros::Subscriber obstacle_subscriber;
		// ros::Publisher _publisher;
		double range;
		pcNormalPtr obstacles;
		std::vector<State4d> clusters;
		State4d own;
	};

} // namespace normal_reaction_force

#endif

