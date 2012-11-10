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

		//assume 45 +- 40deg imaging angle
		//seg.setAxis(Eigen::Vector3f(0, -1, -1));
		//seg.setEpsAngle(pcl::deg2rad(40.0f));
    }
    ~Segment() {}

    // not in use
	/*typename pcl::PointCloud<PointT>::CloudVectorType& segment(typename askinect::ModelData<PointT>& model)
	{
		segments.clear();
		pcl::copyPointCloud(model.rawCloud, cloudCopy);
		//iterate cloud as many times as new segments are found
		for(int i=0; i<5; i++) {
			seg.setInputCloud(cloudCopy.makeShared());
			seg.segment (*inliers, *coefficients);
			if (inliers->indices.size () == 0)
			{
			  std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
			  break;
			}

			// Extract the inliers
			pcl::ExtractIndices<PointT> extract;
			extract.setInputCloud (cloudCopy.makeShared());
			extract.setIndices (inliers);

			// Indices that belong to cloud are added to segment vector
			extract.setNegative (false);
			pcl::PointCloud<PointT> segment;
			extract.filter (segment);
			segments.push_back(segment);

			// Indices that did not belong to the found segment are left to cloudCopy and searched for other segments
			extract.setNegative (true);
			extract.filter(cloudCopy);
			//cloud_filtered.swap (cloud_f);
			//pcl::PointCloud<PointT> segment(cloudCopy, *inliers);
		}
		//pcl::copyPointCloud(segments, model.segments);
		return segments;
	}
*/
    pcl::ExtractIndices<PointT> findSurface (typename askinect::ModelData<PointT>& model) {
    	//currently defined in constructor
		// Optional
		//seg.setOptimizeCoefficients (true);
		// Mandatory
		//seg.setModelType (pcl::SACMODEL_PLANE);
		//seg.setMethodType (pcl::SAC_RANSAC);
		//seg.setDistanceThreshold (0.03);

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
		/////// 2. Calculate a convex hull (box above the table), where objects on the table are located //////
		pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCopy(new pcl::PointCloud<pcl::PointXYZ>());
		// Project the model inliers
		pcl::copyPointCloud(cloudCopy, *xyzCopy);
		/*		std::cout << "Table has " << xyzCopy->points.size() << " points." << std::endl;
		pcl::ProjectInliers<pcl::PointXYZ> proj;
		proj.setModelType(pcl::SACMODEL_PLANE);
		std::cout << "Table has -1" << std::endl;
		proj.setInputCloud(xyzCopy);
		proj.setModelCoefficients(coefficients);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>());
		proj.filter(*cloud_projected);
		std::cout << "Table has 0" << std::endl;
		// Create a Convex Hull representation of the projected inliers
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>());
		pcl::ConvexHull<pcl::PointXYZ> chull;
		chull.setInputCloud(cloud_projected);
		std::cout << "Table has 1" << std::endl;
		chull.reconstruct(*cloud_hull);
		std::cout << "Table has 2" << std::endl;
*/		/////// 3. Cluster the objects in the convex hull ///////
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

			std::cout << "PointCloud representing the Cluster: "
					<< cloud_cluster->points.size() << " data points."
					<< std::endl;
			typename pcl::PointCloud<PointT>::Ptr cloud_cluster_rgb (new pcl::PointCloud<PointT>);
			pcl::copyPointCloud(*cloud_cluster, *cloud_cluster_rgb);
			model.segments.push_back(*cloud_cluster_rgb);
			j++;
		}
		std::cout << "Found " << j << " segments." << std::endl;
		//old planar extraction
		/*
		//iterate cloud as many times as new segments are found
		for (int i = 0; i < 5; i++) {
			seg.setInputCloud(cloudCopy.makeShared());
			seg.segment(*inliers, *coefficients);
			if (inliers->indices.size() == 0) {
				std::cerr
						<< "Could not estimate a planar model for the given dataset."
						<< std::endl;
				break;
			}

			// Extract the inliers
			pcl::ExtractIndices<PointT> extract;
			extract.setInputCloud(cloudCopy.makeShared());
			extract.setIndices(inliers);

			// Indices that belong to cloud are added to segment vector
			extract.setNegative(false);
			pcl::PointCloud<PointT> segment;
			extract.filter(segment);
			model.segments.push_back(segment);

			// Indices that did not belong to the found segment are left to cloudCopy and searched for other segments
			extract.setNegative(true);
			extract.filter(cloudCopy);
			//cloud_filtered.swap (cloud_f);
			//pcl::PointCloud<PointT> segment(cloudCopy, *inliers);
		}
		*/
	}



	/*
	typename pcl::PointCloud<PointT>::CloudVectorType& segmentCylinders(
					const typename pcl::PointCloud<PointT>::ConstPtr &cloud) {
				// All the objects needed
				pcl::PCDReader reader;
				pcl::PassThrough < PointT > pass;
				pcl::NormalEstimation<PointT, pcl::Normal> ne;
				pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
				pcl::PCDWriter writer;
				pcl::ExtractIndices<PointT> extract;
				pcl::ExtractIndices<pcl::Normal> extract_normals;
				pcl::search::KdTree<PointT>::Ptr tree(
						new pcl::search::KdTree<PointT>());

				// Datasets
				pcl::PointCloud<PointT>::Ptr cloud_filtered(
						new pcl::PointCloud<PointT>);
				pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(
						new pcl::PointCloud<pcl::Normal>);
				pcl::PointCloud<PointT>::Ptr cloud_filtered2(
						new pcl::PointCloud<PointT>);
				pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(
						new pcl::PointCloud<pcl::Normal>);
				pcl::ModelCoefficients::Ptr coefficients_plane(
						new pcl::ModelCoefficients), coefficients_cylinder(
						new pcl::ModelCoefficients);
				pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices),
						inliers_cylinder(new pcl::PointIndices);

				// Read in the cloud data
				reader.read("table_scene_mug_stereo_textured.pcd", *cloud);
				std::cerr << "PointCloud has: " << cloud->points.size()
						<< " data points." << std::endl;

				// Build a passthrough filter to remove spurious NaNs
				pass.setInputCloud(cloud);
				pass.setFilterFieldName("z");
				pass.setFilterLimits(0, 1.5);
				pass.filter(*cloud_filtered);
				std::cerr << "PointCloud after filtering has: "
						<< cloud_filtered->points.size() << " data points."
						<< std::endl;

				// Estimate point normals
				ne.setSearchMethod(tree);
				ne.setInputCloud(cloud_filtered);
				ne.setKSearch(50);
				ne.compute(*cloud_normals);
				// Create the segmentation object for the planar model and set all the parameters
				seg.setOptimizeCoefficients(true);
				seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
				seg.setNormalDistanceWeight(0.1);
				seg.setMethodType(pcl::SAC_RANSAC);
				seg.setMaxIterations(100);
				seg.setDistanceThreshold(0.03);
				seg.setInputCloud(cloud_filtered);
				seg.setInputNormals(cloud_normals);
				// Obtain the plane inliers and coefficients
				seg.segment(*inliers_plane, *coefficients_plane);
				std::cerr << "Plane coefficients: " << *coefficients_plane
						<< std::endl;

				// Extract the planar inliers from the input cloud
				extract.setInputCloud(cloud_filtered);
				extract.setIndices(inliers_plane);
				extract.setNegative(false);

				// Write the planar inliers to disk
				pcl::PointCloud<PointT>::Ptr cloud_plane(
						new pcl::PointCloud<PointT>());
				extract.filter(*cloud_plane);
				std::cerr << "PointCloud representing the planar component: "
						<< cloud_plane->points.size() << " data points."
						<< std::endl;
				writer.write("table_scene_mug_stereo_textured_plane.pcd",
						*cloud_plane, false);

				// Remove the planar inliers, extract the rest
				extract.setNegative(true);
				extract.filter(*cloud_filtered2);
				extract_normals.setNegative(true);
				extract_normals.setInputCloud(cloud_normals);
				extract_normals.setIndices(inliers_plane);
				extract_normals.filter(*cloud_normals2);
				// Remove the planar inliers, extract the rest
				extract.setNegative(true);
				extract.filter(*cloud_filtered2);
				extract_normals.setNegative(true);
				extract_normals.setInputCloud(cloud_normals);
				extract_normals.setIndices(inliers_plane);
				extract_normals.filter(*cloud_normals2);

				// Create the segmentation object for cylinder segmentation and set all the parameters
				seg.setOptimizeCoefficients(true);
				seg.setModelType(pcl::SACMODEL_CYLINDER);
				seg.setMethodType(pcl::SAC_RANSAC);
				seg.setNormalDistanceWeight(0.1);
				seg.setMaxIterations(10000);
				seg.setDistanceThreshold(0.05);
				seg.setRadiusLimits(0, 0.1);
				seg.setInputCloud(cloud_filtered2);
				seg.setInputNormals(cloud_normals2);

				// Obtain the cylinder inliers and coefficients
				seg.segment(*inliers_cylinder, *coefficients_cylinder);
				std::cerr << "Cylinder coefficients: " << *coefficients_cylinder
						<< std::endl;

				// Write the cylinder inliers to disk
				extract.setInputCloud(cloud_filtered2);
				extract.setIndices(inliers_cylinder);
				extract.setNegative(false);
				pcl::PointCloud<PointT>::Ptr cloud_cylinder(
						new pcl::PointCloud<PointT>());
				extract.filter(*cloud_cylinder);
				if (cloud_cylinder->points.empty())
					std::cerr << "Can't find the cylindrical component."
							<< std::endl;
				else {
					std::cerr
							<< "PointCloud representing the cylindrical component: "
							<< cloud_cylinder->points.size() << " data points."
							<< std::endl;
					writer.write("table_scene_mug_stereo_textured_cylinder.pcd",
							*cloud_cylinder, false);
				}
			}
	*/

};

}
