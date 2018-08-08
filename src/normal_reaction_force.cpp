
//
// src: normal_reaction_force.cpp
//
// last update: '18.xx.xx
// author: matchey
//
// memo:
//

#include <sensor_msgs/PointCloud2.h> // for debug
#include <pcl_conversions/pcl_conversions.h> // for debug
#include <tf/transform_broadcaster.h> // for debug
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include "mmath/binarion.h"
#include "mmath/common/logistic.h"
#include "normal_reaction_force/normal_reaction_force.h"

namespace normal_reaction_force{

	VectorField::VectorField()
		// : own({{0,0}, {0,0}})
	{
		// ros::param::param<double>
		// 	("/normal_reaction_force/range", range, 1.2);

		ros::param::param<int>
			("/normal_reaction_force/grid_dimensions", grid_dim, 320);

		ros::param::param<double>
			("/normal_reaction_force/cell_size", m_per_cell, 0.3);

		ros::param::param<double>
			("/normal_reaction_force/expand", expand, 0.1);

		ros::param::param<double>
			("/path_prediction/step_size", step_size, 40);

		field.resize(grid_dim);
		for(int i=0; i<grid_dim; ++i){
			field[i].resize(grid_dim);
		}

		range = step_size * 1.1 / 10 + expand; // velocity = 1.1 [m/s], roop_late = 10 [Hz]

		// _publisher = node.advertise<sensor_msgs::PointCloud2>("obsOnLine", 10); // for debug
		_publisher = node.advertise<visualization_msgs::MarkerArray>("vectorField", 10); // for debug
	}

	VectorField::~VectorField() {}

	void VectorField::setObstacles(const pcNormalPtr& obs_cloud)
	{
		obstacles = obs_cloud;
		constructGrid();
	}

	// void VectorField::setHumans(const vmsgs::MarkerArray::ConstPtr& humans)
	// {
	// }

	void VectorField::velocityConversion(std::vector<State4d>& humans)
	{
	}
	
	// void VectorField::velocityConversion(State4d& own_state)
	// {
	// 	// own = own_state;
    //
    //
	// 	// const double speed = own.velocity.norm();
	// }

	// template<class T_src, class T_tgt>
	// double VectorField::distance(const T_src& pos_src, const T_tgt& pos_tgt)
	// {
	// 	return sqrt(pow(pos_src.x() - pos_tgt.x, 2) + pow(pos_src.y() - pos_tgt.y, 2));
	// }
    //
	// template double VectorField::distance(const Eigen::Vector2d&, const geometry_msgs::Point&);
	// template double VectorField::distance(const Eigen::Vector2d&, const PointN&);

	void VectorField::constructGrid()
	{
		int cnt[grid_dim][grid_dim];
		for (int x = 0; x < grid_dim; x++) {
			for (int y = 0; y < grid_dim; y++) {
				cnt[x][y] = 0;
			}
		}

		Eigen::Vector2d v = {0.0, 0.0};
		for(int i = 0; i<grid_dim; ++i){
			std::fill(field[i].begin(), field[i].end(), v);
		}

		unsigned npoints = obstacles->points.size();
		for(unsigned idx = 0; idx < npoints; ++idx){
			int i = 0, j = 0;
			double i_max;
			bool is2x; // direction{ x:true, y:false }

			const Eigen::Vector2d normal = {-obstacles->points[idx].normal_x,
										    -obstacles->points[idx].normal_y};
			const double theta = atan2(normal.y(), normal.x());

			const int x = ((grid_dim/2)+obstacles->points[idx].x/m_per_cell);
			const int y = ((grid_dim/2)+obstacles->points[idx].y/m_per_cell);

			if((-M_PI/4 <= theta && theta < M_PI/4) || (3*M_PI/4 <= theta || theta < -3*M_PI/4)){
				i = x;
				i_max = range * cos(theta) / m_per_cell;
				is2x = true;
			}else{ // y
				i = y;
				i_max = range * sin(theta) / m_per_cell;
				is2x = false;
			}

			if(i < 0 || i > grid_dim){ continue; }

			while(1){
				if(-M_PI/4 <= theta && theta < 3*M_PI/4){ // +
					++i;
					if(is2x ? i>grid_dim || i > x+i_max : i>grid_dim || i > y+i_max){ break; }
				}else{ // -
					--i;
					if(is2x ? i < 0 || i < x + i_max : i < 0 || i < y + i_max){ break; }
				}
				if(is2x){ j = y + (i-x) * tan(theta);
				}else{ j = x + (i-y) / tan(theta); }
				if(j >= 0 && j < grid_dim){
					if(is2x){
						field[i][j] += (1 - mmath::logistic(abs(i-x))) * normal;
						++cnt[i][j];
					}else{
						field[j][i] += (1 - mmath::logistic(abs(i-y))) * normal;
						++cnt[j][i];
					}
				}
			}
		}
		for(int i = 0; i < grid_dim; ++i){
			for(int j = 0; j < grid_dim; ++j){
				if(field[i][j].x() || field[i][j].y()){
					field[i][j] = 1.0/cnt[i][j] * field[i][j];
				}
			}
		}
		// publish(); // for debug
	}
	
	void VectorField::publish()
	{
		visualization_msgs::Marker arrow;

		// marker.header.frame_id = "/map";
		arrow.header.stamp = ros::Time::now();
		arrow.header.frame_id = "/velodyne";

		arrow.ns = "/vf/arrow";
		int id = 0;

		arrow.type = visualization_msgs::Marker::ARROW;
		arrow.action = visualization_msgs::Marker::ADD;
		// arrow.action = visualization_msgs::Marker::DELETE;
		// arrow.action = visualization_msgs::Marker::DELETEALL;

		arrow.scale.x = 0.3; // length
		// arrow.scale.x = 1.0; // length
		arrow.scale.y = 0.06; // width
		arrow.scale.z = 0.06; // height

		arrow.color.r = 0.8f;
		arrow.color.g = 0.1f;
		arrow.color.b = 1.0f;
		arrow.color.a = 0.6;

		// arrow.lifetime = ros::Duration(1.0);

		visualization_msgs::MarkerArray markers;

		double grid_offset=grid_dim/2.0*m_per_cell;

		arrow.pose.position.z = 0.0;
		double yaw;

		for(int i = 0; i < grid_dim; ++i){
			for(int j = 0; j < grid_dim; ++j){
				if(field[i][j].x() || field[i][j].y()){
					arrow.pose.position.x = -grid_offset + (i*m_per_cell+m_per_cell/2.0);
					arrow.pose.position.y = -grid_offset + (j*m_per_cell+m_per_cell/2.0);
					yaw = atan2(field[i][j].y(), field[i][j].x());
					arrow.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
					arrow.id = id;
					arrow.scale.x = sqrt(pow(field[i][j].x(),2) + pow(field[i][j].y(), 2));
					markers.markers.push_back(arrow);
					++id;
				}
			}
		}

		_publisher.publish(markers);
	}

} // namespace normal_reaction_force
