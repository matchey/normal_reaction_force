
//
// src: normal_reaction_force.cpp
//
// last update: '18.xx.xx
// author: matchey
//
// memo:
//

// #include <sensor_msgs/PointCloud2.h> // for debug
// #include <pcl_conversions/pcl_conversions.h> // for debug
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include "mmath/binarion.h"
#include "normal_reaction_force/normal_reaction_force.h"

namespace normal_reaction_force{

	VectorField::VectorField()
		: own({{0,0}, {0,0}})
	{
		// ros::param::param<double>
		// 	("/normal_reaction_force/range", range, 1.2);

		ros::param::param<double>
			("/normal_reaction_force/expand", expand, 0.1);

		ros::param::param<double>
			("/normal_reaction_force/step_size", step_size, 3);

		range = step_size * 2.0 + expand;

		// _publisher = node.advertise<sensor_msgs::PointCloud2>("obsOnLine", 10); // for debug
	}

	VectorField::~VectorField() {}

	void VectorField::setObstacles(const pcNormalPtr& obs_cloud)
	{
		obstacles = obs_cloud;
	}

	void VectorField::velocityConversion(const State4d& own_state, Eigen::Vector2d& velocity)
	{
		own = own_state;
		// double speed = own.velocity.norm();
		clustering();

		// velocity = {0.0, 0.0};
		velocity = own.velocity;

		// std::cout << "\n" << std::endl;
		// std::cout << "velocity before :" << std::endl;
		// std::cout << velocity << std::endl;

		// std::cout << "clusters.size() : " << clusters.size() << std::endl;
		// std::cout << velocity << std::endl;

		// std::cout << "clusters.size() : " << clusters.size() << std::endl;
		for(auto it = clusters.begin(); it != clusters.end(); ++it){
			// std::cout << std::distance(clusters.begin(), it) << std::endl;
			// std::cout << "obs pos :" << std::endl;
			// std::cout << it->position << std::endl;
			// std::cout << "obs vel :" << std::endl;
			// std::cout << it->velocity << std::endl;
			Eigen::Vector2d own2obs = it->position - own.position;
			// std::cout << "obs :" << std::endl;
			// std::cout << it->position << std::endl;
			// std::cout << "own2obs :" << std::endl;
			// std::cout << own2obs << std::endl;
			if(own2obs.dot(velocity) > 0){ // obstacleに向かう速度なら
				double apart = own2obs.norm(); // obstacleまでの距離
				if(own2obs.dot(it->velocity) > 0){
					it->velocity *= -1;
				}
				// velocity += fabs(it->velocity.dot(own.velocity)) * it->velocity * speed / apart;
				double speed = fabs(it->velocity.dot(velocity));
				double dist = velocity.norm() * step_size; // 1 step_size で進む距離
				// std::cout << "dist = " << dist << std::endl;
				// std::cout << "apart = " << apart << std::endl;
				if(apart - expand < 0){
					// velocity += fabs(it->velocity.dot(own.velocity)) * it->velocity;
					// std::cout << "in expand :" << std::endl;
					// std::cout << velocity << std::endl;
					velocity += speed * it->velocity;
					// velocity += fabs(it->velocity.dot(own.velocity)) * it->velocity.normalized();
				}else if(apart - expand < dist){
					// velocity += fabs(it->velocity.dot(own.velocity)) * it->velocity
					// velocity += fabs(it->velocity.dot(velocity)) * it->velocity
					// std::cout << "velocity before :" << std::endl;
					// std::cout << velocity << std::endl;
					velocity += 1/pow(2, apart)* speed * (1 - (apart - expand) / dist) * it->velocity;
				}else{
					// velocity += speed * it->velocity * 0.3;
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
		// 	pc2.header.frame_id = "/map";
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
		ec.setMaxClusterSize (1500);
		ec.setSearchMethod (tree);
		ec.setInputCloud (obs_on_line);
		ec.extract (cluster_indices);

		// std::cout << "cluster_indices.size() : " << cluster_indices.size() << std::endl;
		// int j = 0;
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin ();
				                                             it != cluster_indices.end (); ++it)
		{
			State4d cluster = {{0, 0}, {0, 0}};
			int npoints = 0;
			// pcNormalPtr cloud_cluster (new pcNormal);
			for (std::vector<int>::const_iterator pit = it->indices.begin ();
					                               pit != it->indices.end (); ++pit){
				// cloud_cluster->points.push_back (obs_on_line->points[*pit]); /#<{(|
				cluster.position.x() += obs_on_line->points[*pit].x;
				cluster.position.y() += obs_on_line->points[*pit].y;
				cluster.velocity.x() += obs_on_line->points[*pit].normal_x;
				cluster.velocity.y() += obs_on_line->points[*pit].normal_y;
				npoints++;
			}
			// if(npoints > 5){
			if(npoints > 0){
				// cluster.position.x() /= npoints;
				// cluster.position.y() /= npoints;
				cluster.position /= npoints;
				// cluster.velocity.x() /= npoints;
				// cluster.velocity.y() /= npoints;
				cluster.velocity /= npoints;
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

} // namespace normal_reaction_force
