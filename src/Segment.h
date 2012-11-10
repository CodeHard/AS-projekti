/*
Copyright (c) 2012 Tommi Tikkanen, Eero Laukkanen

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#pragma once
#include <boost/ptr_container/ptr_vector.hpp>
#include <vector>
#include "DataTypes.h"

#include <pcl/point_cloud.h>
#include <pcl/common/angles.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>



namespace askinect
{

template<typename PointT>
class Segment
{
private:
    typename pcl::PointCloud<PointT> cloudCopy;
    typename pcl::PointCloud<PointT>::CloudVectorType segments;
	pcl::SACSegmentation<PointT> seg;
	pcl::ModelCoefficients::Ptr coefficients;
	pcl::PointIndices::Ptr inliers;
public:
    Segment() : coefficients (new pcl::ModelCoefficients), inliers (new pcl::PointIndices)
    {
		// Optional
		seg.setOptimizeCoefficients (true);
		// Mandatory
		//seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setDistanceThreshold (0.03);

		//assume 45 +- 40deg imaging angle - this is to limit the found planes to horizontal surfaces, for example walls would not count
		//seg.setAxis(Eigen::Vector3f(0, -1, -1));
		//seg.setEpsAngle(pcl::deg2rad(40.0f));
    }
    ~Segment() {}

    pcl::ExtractIndices<PointT> findSurface (typename askinect::ModelData<PointT>& model) {

    	pcl::copyPointCloud(model.rawCloud, cloudCopy);

    	seg.setInputCloud(cloudCopy.makeShared());
		seg.setInputCloud(cloudCopy.makeShared());
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0) {
			std::cerr
					<< "Could not estimate a planar model for the given dataset."
					<< std::endl;
		}

		// Extract the inliers
		pcl::ExtractIndices<PointT> extract;
		extract.setInputCloud(cloudCopy.makeShared());
		extract.setIndices(inliers);

		// Indices that belong to cloud are assumed to be the ground surface
		extract.setNegative(false);
		extract.filter(model.surface);

		return extract;
    }

    /* 1. Find table surface plane and place it in model.surface
     * 2. Calculate a convex hull (box above the table), where objects on the table are located
     * 3. Cluster the objects in the convex hull
     * 4. Save clusters to model.segments
     */
	void segment( typename askinect::ModelData<PointT>& model ) {
		model.segments.clear();
		pcl::copyPointCloud(model.rawCloud, cloudCopy);
		/////// 1. Find table surface plane and place it in model.surface ///////
		pcl::ExtractIndices<PointT> extract = findSurface(model);
		extract.setNegative(true);
		extract.filter(cloudCopy);

		if(cloudCopy.points.size() == model.rawCloud.points.size()) {
			std::cout << "Ground surface was not found, but it is needed for segmentation." << std::endl;
			return;
		}
		//--> cloudCopy should now be without the surface, i.e. table removed and ready for clustering
		/////// 2. Calculate a convex hull (borders of the table) and extrude it to a box above the surface, where objects are located //////
		pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCopy(new pcl::PointCloud<pcl::PointXYZ>());
		// Project the model inliers
		pcl::copyPointCloud(cloudCopy, *xyzCopy);

		pcl::PointCloud<pcl::PointXYZ>::Ptr surfCopy(new pcl::PointCloud<pcl::PointXYZ>());
		pcl::copyPointCloud(model.surface, *surfCopy);

		// Create a Convex Hull representation of the projected inliers
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::ConvexHull<pcl::PointXYZ> chull;
		chull.setInputCloud(surfCopy);
		chull.reconstruct(*cloud_hull);

		// Segment those points that are in the polygonal prism
		pcl::ExtractPolygonalPrismData<pcl::PointXYZ> ex;
		ex.setInputCloud (xyzCopy);
		ex.setInputPlanarHull (cloud_hull);
		pcl::PointIndices::Ptr tableTopCloud (new pcl::PointIndices);
		ex.segment (*tableTopCloud);

		pcl::ExtractIndices<pcl::PointXYZ> ex2;
		ex2.setInputCloud(xyzCopy);
		ex2.setIndices(tableTopCloud);
		// Indices that belong to cloud are assumed to be the ground surface
		ex2.setNegative(false);
		ex2.filter(*xyzCopy);

		/////// 3. Cluster the objects in the convex hull ///////
		// Creating the KdTree object for the search method of the extraction
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
				new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud(xyzCopy);

		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		ec.setClusterTolerance(0.02); // 2cm
		ec.setMinClusterSize(800);
		ec.setMaxClusterSize(25000);
		ec.setSearchMethod(tree);
		ec.setInputCloud(xyzCopy);
		ec.extract(cluster_indices);

		int j = 0;
		for (std::vector<pcl::PointIndices>::const_iterator it =
				cluster_indices.begin(); it != cluster_indices.end(); ++it) {
		    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
		    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
		      cloud_cluster->points.push_back (xyzCopy->points[*pit]); //*
		    cloud_cluster->width = cloud_cluster->points.size ();
		    cloud_cluster->height = 1;
		    cloud_cluster->is_dense = true;

			//std::cout << "PointCloud representing the Cluster: "
			//		<< cloud_cluster->points.size() << " data points."
			//		<< std::endl;
			typename pcl::PointCloud<PointT>::Ptr cloud_cluster_rgb (new pcl::PointCloud<PointT>);
			pcl::copyPointCloud(*cloud_cluster, *cloud_cluster_rgb);
			model.segments.push_back(*cloud_cluster_rgb);
			j++;
		}
	}
};

}
