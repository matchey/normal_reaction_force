
//
// src: normal_reaction_force.cpp
//
// last update: '18.xx.xx
// author: matchey
//
// memo:
//

/** for debug **/
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
/** for debug **/

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include "mmath/binarion.h"
#include "mmath/common/logistic.h"
#include "normal_reaction_force/normal_reaction_force.h"

namespace normal_reaction_force{

	VectorField::VectorField()
	{
		ros::param::param<int>
			("/normal_reaction_force/grid_dimensions", grid_dim, 280);

		ros::param::param<double>
			("/normal_reaction_force/cell_size", m_per_cell, 0.15);

		ros::param::param<double>
			("/normal_reaction_force/expand", expand, 0.5);

		ros::param::param<double>
			("/path_prediction/step_size", step_size, 40);

		field.resize(grid_dim);
		for(int i=0; i<grid_dim; ++i){
			field[i].resize(grid_dim);
		}

		// range = step_size * 1.1 / 10 + expand; // velocity = 1.1 [m/s], roop_late = 10 [Hz]
		range = 1.1 * 3; // velocity = 1.1 [m/s], time = 2 [s]

		// _publisher = node.advertise<sensor_msgs::PointCloud2>("obsOnLine", 10); // for debug
		_publisher = node.advertise<visualization_msgs::MarkerArray>("vectorField", 10); // for debug
	}

	VectorField::~VectorField() {}

	void VectorField::setObstacles(const pcNormalPtr& obs_cloud)
	{
		obstacles = obs_cloud;
		npoints = obstacles->points.size();
		constructGrid();
	}

	void VectorField::setDistances(const std::vector<State4d>& humans, const unsigned& nums)
	{
		nhumans = nums;
		distances.resize(nhumans, nhumans);
		// in_range.resize(nhumans, nhumans);

		double dist;
		for(unsigned i = 0; i < nhumans; ++i){
			distances.coeffRef(i, i) = 0;
			for(unsigned j = i+1; j < nhumans; ++j){
				dist = distance(humans[i].position, humans[j].position);
				distances.coeffRef(i, j) = dist - expand;
				distances.coeffRef(j, i) = dist - expand;
			}
		}
	}

	void VectorField::velocityConversion(std::vector<State4d>& humans)
	{
		setDistances(humans);

		Eigen::Vector2d human_normal;
		Eigen::Vector2d human_force;
		Eigen::Vector2d other2own;
		int human_count;

		std::vector<Eigen::Vector2d> velocity_converted;
		velocity_converted.resize(nhumans);

		for(unsigned idx = 0; idx < nhumans; ++idx){
			int row = (grid_dim/2) + humans[idx].position.x() / m_per_cell;
			int col = (grid_dim/2) + humans[idx].position.y() / m_per_cell;
			velocity_converted[idx] = humans[idx].velocity;
			if(0 <= row && row < grid_dim && 0 <= col && col < grid_dim){
				if(field[row][col].x() || field[row][col].y()){
					double dot_prod = humans[idx].velocity.dot(field[row][col]);
					if(dot_prod < 0){
						velocity_converted[idx] -= dot_prod * field[row][col];
					}
				}
			}
			human_force = {0.0, 0.0};
			human_count = 0;
			for(unsigned i = 0; i < nhumans; ++i){
				if(i != idx && distances.coeff(idx, i) < range){
					other2own = (humans[idx].position - humans[i].position).normalized();
					human_normal = (1 - mmath::logistic(distances.coeff(idx, i))) * other2own;
					double dot_prod = (humans[idx].velocity - humans[i].velocity).dot(human_normal);
					if(dot_prod < 0){
						human_force += 0.005 * dot_prod * human_normal;
						++human_count;
					}
				}
			}
			if(human_count){
				velocity_converted[idx] -= human_force / human_count;
			}
		}
		for(unsigned idx = 0; idx < nhumans; ++idx){
			double curve_rate
				     = humans[idx].velocity.normalized().dot(velocity_converted[idx].normalized());
			if(curve_rate < 0.3){
				humans[idx].velocity *= 0.01;
			}else if(curve_rate < 0.60){
				double norm_bfr = humans[idx].velocity.norm();
				double norm_aft = velocity_converted[idx].norm();
				if(norm_bfr < norm_aft){
					// humans[idx].velocity = norm_bfr / norm_aft * velocity_converted[idx];
					velocity_converted[idx] *= norm_bfr / norm_aft;
				}else{
					// humans[idx].velocity = velocity_converted[idx];
				}
				humans[idx].velocity = 0.2 * velocity_converted[idx] + 0.8 * humans[idx].velocity;
			}else{
				humans[idx].velocity = velocity_converted[idx];
			}
		}
	}

	// private
	void VectorField::constructGrid()
	{
		setDirections();
		setMagnitudes();

		static int m_pub = 0;
		if(m_pub == 20){
			publish(); // for debug
			m_pub = 0;
		}else{
			++m_pub;
		}
	}

	void VectorField::setDirections()
	{
		fill(field, Eigen::MatrixXd::Zero(2, 1));

		for(unsigned idx = 0; idx < npoints; ++idx){
			int i = 0, j = 0;
			double i_max;
			bool is2x; // direction{ x:true, y:false }

			const Eigen::Vector2d normal = {-obstacles->points[idx].normal_x,
										    -obstacles->points[idx].normal_y};
			const double theta = atan2(normal.y(), normal.x());

			const int x = ((grid_dim/2) + obstacles->points[idx].x / m_per_cell);
			const int y = ((grid_dim/2) + obstacles->points[idx].y / m_per_cell);

			if((-M_PI/4 <= theta && theta < M_PI/4) || (3*M_PI/4 <= theta || theta < -3*M_PI/4)){
				i = x;
				i_max = range * cos(theta) / m_per_cell;
				is2x = true;
			}else{ // y
				i = y;
				i_max = range * sin(theta) / m_per_cell;
				is2x = false;
			}
			if(i < 0 || i >= grid_dim){ continue; }
			while(1){
				if(is2x){
					j = y + (i-x) * tan(theta);
				}else{
					j = x + (i-y) / tan(theta);
				}
				if(j >= 0 && j < grid_dim){
					int row = j; int col = i; int dist_of = y;
					if(is2x){
						row = i; col = j; dist_of = x;
					}
					field[row][col] +=  (1 - mmath::logistic(abs(i-dist_of) - expand)) * normal;
				}
				if(-M_PI/4 <= theta && theta < 3*M_PI/4){ // +
					++i;
					if(is2x ? i>=grid_dim || i > x+i_max : i>=grid_dim || i > y+i_max){ break; }
				}else{ // -
					--i;
					if(is2x ? i < 0 || i < x + i_max : i < 0 || i < y + i_max){ break; }
				}
			}
		}
	}

	void VectorField::setMagnitudes()
	{
		int cnt[grid_dim][grid_dim];
		Field magni(grid_dim, std::vector<Eigen::Vector2d>(grid_dim, Eigen::MatrixXd::Zero(2, 1)));

		std::fill(cnt[0], cnt[grid_dim], 0);
		fill(magni, Eigen::MatrixXd::Zero(2, 1));

		for(unsigned idx = 0; idx < npoints; ++idx){
			int i = 0, j = 0;
			double i_max;
			bool is2x; // direction{ x:true, y:false }

			const Eigen::Vector2d normal = {-obstacles->points[idx].normal_x,
										    -obstacles->points[idx].normal_y};
			const double theta = atan2(normal.y(), normal.x());

			const int x = ((grid_dim/2) + obstacles->points[idx].x / m_per_cell);
			const int y = ((grid_dim/2) + obstacles->points[idx].y / m_per_cell);

			if((-M_PI/4 <= theta && theta < M_PI/4) || (3*M_PI/4 <= theta || theta < -3*M_PI/4)){
				i = x;
				i_max = range * cos(theta) / m_per_cell;
				is2x = true;
			}else{ // y
				i = y;
				i_max = range * sin(theta) / m_per_cell;
				is2x = false;
			}
			if(i < 0 || i >= grid_dim){ continue; }
			while(1){
				if(is2x){
					j = y + (i-x) * tan(theta);
				}else{
					j = x + (i-y) / tan(theta);
				}
				if(j >= 0 && j < grid_dim){
					int row = j; int col = i; int dist_of = y;
					if(is2x){
						row = i; col = j; dist_of = x;
					}
					if(field[row][col].normalized().dot(normal) > 0.99){
						magni[row][col] += (1 - mmath::logistic(abs(i-dist_of) - expand)) * normal;
						++cnt[row][col];
					}
				}
				if(-M_PI/4 <= theta && theta < 3*M_PI/4){ // +
					++i;
					if(is2x ? i>=grid_dim || i > x+i_max : i>=grid_dim || i > y+i_max){ break; }
				}else{ // -
					--i;
					if(is2x ? i < 0 || i < x + i_max : i < 0 || i < y + i_max){ break; }
				}
			}
		}
		for(int i = 0; i < grid_dim; ++i){
			for(int j = 0; j < grid_dim; ++j){
				if(magni[i][j].x() || magni[i][j].y()){
					if(cnt[i][j] < 2){
						magni[i][j] = {0.0, 0.0};
					}else{
						magni[i][j] /= cnt[i][j];
					}
				}
			}
		}
		field = magni;
	}

	void VectorField::setDistances(const std::vector<State4d>& humans)
	{
		double dist;
		for(unsigned i = 0; i < nhumans; ++i){
			for(unsigned j = i+1; j < nhumans; ++j){
				dist = distance(humans[i].position, humans[j].position);
				distances.coeffRef(i, j) = dist - expand;
				distances.coeffRef(j, i) = dist - expand;
			}
		}
	}

	double VectorField::distance(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2)
	{
		double x_diff = p1.x() - p2.x();
		double y_diff = p1.y() - p2.y();

		return sqrt(x_diff * x_diff + y_diff * y_diff);
	}
	
	void VectorField::fill(Field& _field, const Eigen::Vector2d& _val)
	{
		for(int i = 0; i<grid_dim; ++i){
			std::fill(_field[i].begin(), _field[i].end(), _val);
		}
	}

	void VectorField::publish() // for debug
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
