
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

namespace normal_reaction_force{

	namespace vmsgs = visualization_msgs;

	typedef pcl::PointNormal PointN;
	typedef pcl::PointCloud<PointN> pcNormal;
	typedef pcNormal::Ptr pcNormalPtr;

	struct State4d{
		Eigen::Vector2d position;
		Eigen::Vector2d velocity;
	};

	struct Obstacle{
		Eigen::Vector2d position;
		Eigen::Vector2d normal;
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
		// void callback();
		bool isOnLine(const PointN&); // in setObsOnLine()
		void setObsOnLine(pcNormalPtr&); // in clustering()
		void clustering(); // in velocityConversion()
		template<class T_src, class T_tgt>
		double distance(const T_src&, const T_tgt&);

		// subscribeとかは他のノードでやって、フィールド作るだけのクラスにするか
		// ros::Subscriber obstacle_subscriber;
		ros::NodeHandle node; // for debug
		ros::Publisher _publisher; // for debug
		double range; // [m]
		double expand; // [m]
		double step_size; // 何秒先までみるか[s]
		pcNormalPtr obstacles; // Normal に速度を足しこむとか??
		std::vector<Obstacle> clusters; // obstacle(x, y, normal_x, normal_y, vx, vy)
		State4d own; // (x, y, vx, vy)
	};

} // namespace normal_reaction_force

#endif

