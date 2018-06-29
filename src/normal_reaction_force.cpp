
//
// src: normal_reaction_force.cpp
//
// last update: '18.xx.xx
// author: matchey
//
// memo:
//

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include "mmath/binarion.h"
#include "normal_reaction_force/normal_reaction_force.h"

namespace normal_reaction_force{

	VectorField::VectorField()
	{
	}

	VectorField::~VectorField() {}

	//private
	// void VectorField::callback()
	// {
	// }
	bool VectorField::isOnLine(const PointN& obstacle)
	{
		const double threshold = 5.0 * M_PI / 180;

		// obstacle pointの法線の傾き
		Binarion bina_normal(atan2(obstacle.normal_y, obstacle.normal_x));

		// obstacleと現在位置を結ぶ線分の傾き
		Binarion bina_link(atan2(obstacle.y - own.position.y(), obstacle.x - own.position.x()));

		// 傾きが等しければ法線ベクトル上に現在位置がある
		return fabs(bina_normal.deviation(bina_link)) < threshold;
	}

	void VectorField::setObsOnLine(pcNormalPtr& obs_on_line)
	{
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
					obs_on_line->points.push_back(obstacles->points[pointIdx[i]]);
				}
			}
		}

	}

	void VectorField::clustering()
	{
		pcNormalPtr obs_on_line (new pcNormal);
		setObsOnLine(obs_on_line);

		pcl::search::KdTree<PointN>::Ptr tree (new pcl::search::KdTree<PointN>);
		tree->setInputCloud (obs_on_line);

		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<PointN> ec;
		ec.setClusterTolerance (0.02); // 2cm
		ec.setMinClusterSize (100);
		ec.setMaxClusterSize (25000);
		ec.setSearchMethod (tree);
		ec.setInputCloud (obs_on_line);
		ec.extract (cluster_indices);

		// int j = 0;
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin ();
				                                             it != cluster_indices.end (); ++it)
		{
			State4d cluster;
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
			if(npoints > 5){
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
