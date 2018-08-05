
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
		: own({{0,0}, {0,0}})
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
			("/normal_reaction_force/step_size", step_size, 3);

		field.resize(grid_dim);
		for(int i=0; i<grid_dim; ++i){
			field[i].resize(grid_dim);
		}

		range = step_size * 1.1 + expand; // velocity = 1.1 [m/s]

		// _publisher = node.advertise<sensor_msgs::PointCloud2>("obsOnLine", 10); // for debug
		_publisher = node.advertise<visualization_msgs::MarkerArray>("vectorField", 10); // for debug
	}

	VectorField::~VectorField() {}

	void VectorField::setObstacles(const pcNormalPtr& obs_cloud)
	{
		obstacles = obs_cloud;
		constructGrid();
		static int cnt = 0;
		if(cnt == 5){
			publish();
		}
		++cnt;
	}

	void VectorField::setHumans(const vmsgs::MarkerArray::ConstPtr& humans)
	{
		for(auto arrow = humans->markers.begin(); arrow != humans->markers.end(); ++arrow){
			double yaw = atan2(2*(arrow->pose.orientation.w * arrow->pose.orientation.z
								+ arrow->pose.orientation.x * arrow->pose.orientation.y),
					1 - 2*(pow(arrow->pose.orientation.y, 2) + pow(arrow->pose.orientation.z, 2)));

			double dist = distance(own.position, arrow->pose.position);
			// if(1.0 < dist && dist < range){
			if(1.0 < dist){
				Eigen::Vector2d obs2own = {own.position.x() - arrow->pose.position.x,
										   own.position.y() - arrow->pose.position.y};
				Obstacle cluster = {{arrow->pose.position.x, arrow->pose.position.y},
																			 obs2own,
							  {arrow->scale.x * cos(yaw), arrow->scale.x * sin(yaw)}};
				clusters.push_back(cluster);
			}
		}
	}

	void VectorField::velocityConversion(State4d& own_state)
	{
		own = own_state;

		clustering(); // own.positionに向かう法線を持つobstalceをclustering

		const double speed = own.velocity.norm();

		for(auto obs = clusters.begin(); obs != clusters.end(); ++obs){
			Eigen::Vector2d own2obs = obs->position - own.position;
			if(own2obs.dot(own.velocity) > 0){ // obstacleに向かう速度なら
				double apart = own2obs.norm(); // obstacleまでの距離
				if(own2obs.dot(obs->normal) > 0){
					obs->normal *= -1;
				}
				double dotProd = fabs(obs->normal.dot(own.velocity));
				double dist = speed * step_size; // 1 step_size で進む距離
				if(apart - expand < 0){
					own.velocity += dotProd * obs->normal;
				}else if(apart - expand < dist){
					own.velocity += (1 - (apart - expand) / dist) * dotProd * obs->normal;
					// own.velocity += 1/pow(2, apart)* dotProd * (1 - (apart - expand) / dist) * obs->velocity;
				}

				own.velocity = speed * own.velocity.normalized();
			}
		}

		clusters.clear();
	}

	void VectorField::velocityConversion(const State4d& own_state, Eigen::Vector2d& velocity)
	{
		own = own_state;
		// double speed = own.velocity.norm();
		clustering();

		// velocity = {0.0, 0.0};
		velocity = own.velocity;

		// std::cout << "\n" << std::endl;
		// std::cout << "\nvelocity before :" << std::endl;
		// std::cout << velocity << std::endl;

		// std::cout << "clusters.size() : " << clusters.size() << std::endl;
		// std::cout << velocity << std::endl;

		// std::cout << "clusters.size() : " << clusters.size() << std::endl;
		for(auto obs = clusters.begin(); obs != clusters.end(); ++obs){
			// std::cout << std::distance(clusters.begin(), obs) << std::endl;
			// std::cout << "obs pos :" << std::endl;
			// std::cout << obs->position << std::endl;
			// std::cout << "obs vel :" << std::endl;
			// std::cout << obs->velocity << std::endl;
			Eigen::Vector2d own2obs = obs->position - own.position;
			// std::cout << "obs :" << std::endl;
			// std::cout << obs->position << std::endl;
			// std::cout << "own2obs :" << std::endl;
			// std::cout << own2obs << std::endl;
			if(own2obs.dot(velocity) > 0){ // obstacleに向かう速度なら
				double apart = own2obs.norm(); // obstacleまでの距離
				if(own2obs.dot(obs->normal) > 0){
					obs->normal *= -1;
				}
				// velocity += fabs(obs->normal.dot(own.velocity)) * obs->normal * speed / apart;
				double speed = fabs(obs->normal.dot(velocity));
				double dist = velocity.norm() * step_size; // 1 step_size で進む距離
				// std::cout << "dist = " << dist << std::endl;
				// std::cout << "apart = " << apart << std::endl;
				if(apart - expand < 0){
					// velocity += fabs(obs->velocity.dot(own.velocity)) * obs->velocity;
					// std::cout << "in expand :" << std::endl;
					// std::cout << velocity << std::endl;
					velocity += speed * obs->normal;
					// velocity += fabs(obs->velocity.dot(own.velocity)) * obs->velocity.normalized();
				}else if(apart - expand < dist){
					// velocity += fabs(obs->velocity.dot(own.velocity)) * obs->normal
					// velocity += fabs(obs->normal.dot(velocity)) * obs->normal
					// std::cout << "\nvelocity before :" << std::endl;
					// std::cout << velocity << std::endl;
					velocity += speed * (1 - (apart - expand) / dist) * obs->normal;
					// velocity += 1/pow(2, apart)* speed * (1 - (apart - expand) / dist) * obs->velocity;
					// std::cout << "velocity after :" << std::endl;
					// std::cout << velocity << std::endl;
				}else{
					// velocity += speed * obs->velocity * 0.3;
				}
				velocity = own.velocity.norm() * velocity.normalized();
			}
		}

		clusters.clear();

		// std::cout << "velocity after :" << std::endl;
		// std::cout << velocity << std::endl;
	}

	//private
	// void VectorField::callback()
	// {
	// }
	bool VectorField::isOnLine(const PointN& obstacle)
	{
		if(distance(own.position, obstacle) < 1.0){
			return false;
		}

		const double threshold = 15.0 * M_PI / 180;

		// obstacle pointの法線の傾き
		Binarion bina_normal(atan2(obstacle.normal_y, obstacle.normal_x));

		// obstacleと現在位置を結ぶ線分の傾き
		Binarion bina_link(atan2(obstacle.y - own.position.y(), obstacle.x - own.position.x()));

		double deviation = fabs(bina_normal.deviation(bina_link));

		if(deviation > M_PI){
			deviation -= M_PI;
		}

		// 傾きが等しければ法線ベクトル上に現在位置がある
		return deviation < threshold;
	}

	void VectorField::setObsOnLine(pcNormalPtr& obs_on_line)
	{
		if(obstacles->points.empty()) return;

		pcl::KdTreeFLANN<PointN> kdtree;
		kdtree.setInputCloud (obstacles);

		PointN searchPoint; // each cludter's position
		std::vector<int> pointIdx; // index of each point within radius search
		std::vector<float> pointSquaredDistance;

		searchPoint.x = own.position.x();
		searchPoint.y = own.position.y();
		searchPoint.z = 0.0;

		if ( kdtree.radiusSearch (searchPoint, range, pointIdx, pointSquaredDistance) > 0 )
		{
			for (size_t i = 0; i < pointIdx.size (); ++i){
				// std::cout << "    "  <<   obstacles->points[ pointIdx[i] ].x 
				// 	<< " " << obstacles->points[ pointIdx[i] ].y 
				// 	<< " " << obstacles->points[ pointIdx[i] ].z 
				// 	<< " (squared distance: " << pointSquaredDistance[i] << ")" << std::endl;
				if(isOnLine(obstacles->points[pointIdx[i]])){
					// std::cout << "Online :" << i << std::endl;
					obs_on_line->points.push_back(obstacles->points[pointIdx[i]]);
				}else{
					// std::cout << "offline :" << i << std::endl;
				}
			}
		}

	}

	void VectorField::clustering()
	{
		pcNormalPtr obs_on_line (new pcNormal);
		setObsOnLine(obs_on_line);

		/* for debug */
		// static int cnt = 0;
		// if(cnt == 0){
		// 	sensor_msgs::PointCloud2 pc2;
		// 	pcl::toROSMsg(*obs_on_line, pc2);
		// 	pc2.header.frame_id = "/velodyne";
		// 	pc2.header.stamp = ros::Time::now();
		// 	_publisher.publish(pc2);
		// }else if(cnt == 60){
		// 	cnt = 0;
		// }else{
		// 	cnt++;
		// }
		/* ********* */

		if(obs_on_line->points.empty()) return;

		pcl::search::KdTree<PointN>::Ptr tree (new pcl::search::KdTree<PointN>);
		tree->setInputCloud (obs_on_line);

		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<PointN> ec;
		// ec.setClusterTolerance (0.02); // 2cm
		ec.setClusterTolerance (0.15);
		ec.setMinClusterSize (10);
		// ec.setMaxClusterSize (1500);
		ec.setMaxClusterSize (15000);
		ec.setSearchMethod (tree);
		ec.setInputCloud (obs_on_line);
		ec.extract (cluster_indices);

		// std::cout << "cluster_indices.size() : " << cluster_indices.size() << std::endl;
		// int j = 0;
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin ();
				                                             it != cluster_indices.end (); ++it)
		{
			Obstacle cluster = {{0, 0}, {0, 0}, {0, 0}};
			int npoints = 0;
			// pcNormalPtr cloud_cluster (new pcNormal);
			for (std::vector<int>::const_iterator pit = it->indices.begin ();
					                               pit != it->indices.end (); ++pit){
				// cloud_cluster->points.push_back (obs_on_line->points[*pit]); /#<{(|
				cluster.position.x() += obs_on_line->points[*pit].x;
				cluster.position.y() += obs_on_line->points[*pit].y;
				cluster.normal.x() += obs_on_line->points[*pit].normal_x;
				cluster.normal.y() += obs_on_line->points[*pit].normal_y;
				npoints++;
			}
			// if(npoints > 5){
			if(npoints > 0){
				// cluster.position.x() /= npoints;
				// cluster.position.y() /= npoints;
				cluster.position /= npoints;
				// cluster.normal.x() /= npoints;
				// cluster.normal.y() /= npoints;
				cluster.normal /= npoints;
				clusters.push_back(cluster);
			}
			// cloud_cluster->width = cloud_cluster->points.size ();
			// cloud_cluster->height = 1;
			// cloud_cluster->is_dense = true;
            //
			// std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
			// std::stringstream ss;
			// ss << "cloud_cluster_" << j << ".pcd";
			// writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); /#<{(|
			// j++;
		}
	}
	
	template<class T_src, class T_tgt>
	double VectorField::distance(const T_src& pos_src, const T_tgt& pos_tgt)
	{
		return sqrt(pow(pos_src.x() - pos_tgt.x, 2) + pow(pos_src.y() - pos_tgt.y, 2));
	}

	template double VectorField::distance(const Eigen::Vector2d&, const geometry_msgs::Point&);
	template double VectorField::distance(const Eigen::Vector2d&, const PointN&);

	void VectorField::constructGrid()
	{
		// Eigen::Vector2d field[grid_dim][grid_dim];
		int cnt[grid_dim][grid_dim];
		// memset(&cnt, 1, grid_dim*grid_dim);
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

			// if (x >= 0 && x < grid_dim && y >= 0 && y < grid_dim){
			// 	field[x][y] = normal;
			// 	++cnt[x][y];
			// }
			// continue;

			using std::cout;
			using std::endl;
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
						// field[i][j] += normal;
						++cnt[i][j];
						// field[i][j].normalize();
					}else{
						field[j][i] += (1 - mmath::logistic(abs(i-y))) * normal;
						// field[j][i] += normal;
						++cnt[j][i];
						// field[j][i].normalize();
					}
				}
			}
		}
		for(int i = 0; i < grid_dim; ++i){
			for(int j = 0; j < grid_dim; ++j){
				if(field[i][j].x() || field[i][j].y()){
					field[i][j] = 1.0/cnt[i][j] * field[i][j];
					// field[i][j].normalize();
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
