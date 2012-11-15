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
#include <pcl/point_cloud.h>
#include "RecognizeResults.h"
#include <boost/ptr_container/ptr_vector.hpp>
#include "../CloudDB.h"
//copypaste
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/kdtree/kdtree.h>
typedef pcl::PointXYZRGBA PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;

std::string model_filename_;
std::string scene_filename_;

//Algorithm params
bool show_keypoints_ (false);
bool show_correspondences_ (false);
bool use_cloud_resolution_ (false);
bool use_hough_ (true);
float model_ss_ (0.01f);
float scene_ss_ (0.01f);
float rf_rad_ (0.015f);
float descr_rad_ (0.03f);
float cg_size_ (0.01f);
float cg_thresh_ (5.0f);

double
computeCloudResolution (const pcl::PointCloud<PointType>::ConstPtr &cloud)
{
  double res = 0.0;
  int n_points = 0;
  int nres;
  std::vector<int> indices (2);
  std::vector<float> sqr_distances (2);
  pcl::search::KdTree<PointType> tree;
  tree.setInputCloud (cloud);

  for (size_t i = 0; i < cloud->size (); ++i)
  {
    if (! pcl_isfinite ((*cloud)[i].x))
    {
      continue;
    }
    //Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
    if (nres == 2)
    {
      res += sqrt (sqr_distances[1]);
      ++n_points;
    }
  }
  if (n_points != 0)
  {
    res /= n_points;
  }
  return res;
}

namespace askinect
{


template<typename T, typename T2>
void recognize(boost::ptr_vector< pcl::PointCloud<T> >& models,
		pcl::PointCloud<T2> segment) {
	std::map<int, std::string> items;
	items.insert(std::pair<int, std::string>(0, "akku"));
	items.insert(std::pair<int, std::string>(1, "housut"));
	items.insert(std::pair<int, std::string>(2, "koff"));
	items.insert(std::pair<int, std::string>(3, "lakerol"));
	items.insert(std::pair<int, std::string>(4, "urquell"));

	/*
	items[1] = "housut";
	items[2] = "koff";
	items[3] = "lakerol";
	items[4] = "urquell";
	*/
	size_t bestFitIdx = 0;
	size_t bestFit = 0;
	//std::map<std::string, CloudModel<T> >::iterator it = models.begin();
	for (int model_idx = 0; model_idx < models.size(); model_idx++) {
		//std::cout << "best fit this far: " << bestFit << ". \n";
		//std::cout << "handling item " << model_idx << ", " << items[model_idx] << std::endl;

		pcl::PointCloud<PointType>::Ptr model(new pcl::PointCloud<PointType>());
		pcl::PointCloud<PointType>::Ptr model_keypoints(
				new pcl::PointCloud<PointType>());
		pcl::PointCloud<PointType>::Ptr scene(new pcl::PointCloud<PointType>());
		pcl::PointCloud<PointType>::Ptr scene_keypoints(
				new pcl::PointCloud<PointType>());
		pcl::PointCloud<NormalType>::Ptr model_normals(
				new pcl::PointCloud<NormalType>());
		pcl::PointCloud<NormalType>::Ptr scene_normals(
				new pcl::PointCloud<NormalType>());
		pcl::PointCloud<DescriptorType>::Ptr model_descriptors(
				new pcl::PointCloud<DescriptorType>());
		pcl::PointCloud<DescriptorType>::Ptr scene_descriptors(
				new pcl::PointCloud<DescriptorType>());

		pcl::copyPointCloud(models.at(model_idx), *model);
		pcl::copyPointCloud(segment, *scene);

		//
		//  Set up resolution invariance
		//
		if (use_cloud_resolution_) {
			float resolution = static_cast<float>(computeCloudResolution(model));
			if (resolution != 0.0f) {
				model_ss_ *= resolution;
				scene_ss_ *= resolution;
				rf_rad_ *= resolution;
				descr_rad_ *= resolution;
				cg_size_ *= resolution;
			}

			std::cout << "Model resolution:       " << resolution << std::endl;
			std::cout << "Model sampling size:    " << model_ss_ << std::endl;
			std::cout << "Scene sampling size:    " << scene_ss_ << std::endl;
			std::cout << "LRF support radius:     " << rf_rad_ << std::endl;
			std::cout << "SHOT descriptor radius: " << descr_rad_ << std::endl;
			std::cout << "Clustering bin size:    " << cg_size_ << std::endl
					<< std::endl;
		}

		//
		//  Compute Normals
		//
		pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
		norm_est.setKSearch(10);
		norm_est.setInputCloud(model);
		norm_est.compute(*model_normals);

		norm_est.setInputCloud(scene);
		norm_est.compute(*scene_normals);

		//
		//  Downsample Clouds to Extract keypoints
		//
		pcl::PointCloud<int> sampled_indices;

		pcl::UniformSampling<PointType> uniform_sampling;
		uniform_sampling.setInputCloud(model);
		uniform_sampling.setRadiusSearch(model_ss_);
		uniform_sampling.compute(sampled_indices);
		pcl::copyPointCloud(*model, sampled_indices.points, *model_keypoints);
		std::cout << "Model total points: " << model->size()
				<< "; Selected Keypoints: " << model_keypoints->size()
				<< std::endl;

		uniform_sampling.setInputCloud(scene);
		uniform_sampling.setRadiusSearch(scene_ss_);
		uniform_sampling.compute(sampled_indices);
		pcl::copyPointCloud(*scene, sampled_indices.points, *scene_keypoints);
		std::cout << "Scene total points: " << scene->size()
				<< "; Selected Keypoints: " << scene_keypoints->size()
				<< std::endl;

		//
		//  Compute Descriptor for keypoints
		//
		pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
		descr_est.setRadiusSearch(descr_rad_);

		descr_est.setInputCloud(model_keypoints);
		descr_est.setInputNormals(model_normals);
		descr_est.setSearchSurface(model);
		descr_est.compute(*model_descriptors);

		descr_est.setInputCloud(scene_keypoints);
		descr_est.setInputNormals(scene_normals);
		descr_est.setSearchSurface(scene);
		descr_est.compute(*scene_descriptors);

		//
		//  Find Model-Scene Correspondences with KdTree
		//
		pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences());

		pcl::KdTreeFLANN<DescriptorType> match_search;
		match_search.setInputCloud(model_descriptors);

		//  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
		for (size_t i = 0; i < scene_descriptors->size(); ++i) {
			std::vector<int> neigh_indices(1);
			std::vector<float> neigh_sqr_dists(1);
			if (!pcl_isfinite (scene_descriptors->at (i).descriptor[0])) //skipping NaNs
			{
				continue;
			}
			int found_neighs = match_search.nearestKSearch(
					scene_descriptors->at(i), 1, neigh_indices,
					neigh_sqr_dists);
			if (found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
					{
				pcl::Correspondence corr(neigh_indices[0], static_cast<int>(i),
						neigh_sqr_dists[0]);
				model_scene_corrs->push_back(corr);
			}
		}
		std::cout << "Correspondences found: " << model_scene_corrs->size()
				<< std::endl;

		///
		///  DEFINE BEST FIT
		///  (finds biggest correspondance/model_keypoints -ratio
		///
		std::cout << "Fit on this iteration: " << model_scene_corrs->size()*(1.0/model_keypoints->size()) << std::endl;
		if(bestFit < model_scene_corrs->size()*(1.0/model_keypoints->size())) {
			bestFit = model_scene_corrs->size()*(1.0/model_keypoints->size());
			bestFitIdx = model_idx;
		}


		//
		//  Actual Clustering
		//
		std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
		std::vector<pcl::Correspondences> clustered_corrs;

		//  Using Hough3D
		if (use_hough_) {
			//
			//  Compute (Keypoints) Reference Frames only for Hough
			//
			pcl::PointCloud<RFType>::Ptr model_rf(
					new pcl::PointCloud<RFType>());
			pcl::PointCloud<RFType>::Ptr scene_rf(
					new pcl::PointCloud<RFType>());

			pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType,
					RFType> rf_est;
			rf_est.setFindHoles(true);
			rf_est.setRadiusSearch(rf_rad_);

			rf_est.setInputCloud(model_keypoints);
			rf_est.setInputNormals(model_normals);
			rf_est.setSearchSurface(model);
			rf_est.compute(*model_rf);

			rf_est.setInputCloud(scene_keypoints);
			rf_est.setInputNormals(scene_normals);
			rf_est.setSearchSurface(scene);
			rf_est.compute(*scene_rf);

			//  Clustering
			pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
			clusterer.setHoughBinSize(cg_size_);
			clusterer.setHoughThreshold(cg_thresh_);
			clusterer.setUseInterpolation(true);
			clusterer.setUseDistanceWeight(false);

			clusterer.setInputCloud(model_keypoints);
			clusterer.setInputRf(model_rf);
			clusterer.setSceneCloud(scene_keypoints);
			clusterer.setSceneRf(scene_rf);
			clusterer.setModelSceneCorrespondences(model_scene_corrs);

			//clusterer.cluster (clustered_corrs);
			clusterer.recognize(rototranslations, clustered_corrs);
		} else // Using GeometricConsistency
		{
			pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
			gc_clusterer.setGCSize(cg_size_);
			gc_clusterer.setGCThreshold(cg_thresh_);

			gc_clusterer.setInputCloud(model_keypoints);
			gc_clusterer.setSceneCloud(scene_keypoints);
			gc_clusterer.setModelSceneCorrespondences(model_scene_corrs);

			//gc_clusterer.cluster (clustered_corrs);
			gc_clusterer.recognize(rototranslations, clustered_corrs);
		}

		//
		//  Output results
		//
		std::cout << "Model instances found: " << rototranslations.size()
				<< std::endl;
		for (size_t i = 0; i < rototranslations.size(); ++i) {
			std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
			std::cout << "        Correspondences belonging to this instance: "
					<< clustered_corrs[i].size() << std::endl;

			// Print the rotation matrix and translation vector
			Eigen::Matrix3f rotation = rototranslations[i].block<3, 3>(0, 0);
			Eigen::Vector3f translation = rototranslations[i].block<3, 1>(0, 3);

			printf("\n");
			printf("            | %6.3f %6.3f %6.3f | \n", rotation(0, 0),
					rotation(0, 1), rotation(0, 2));
			printf("        R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0),
					rotation(1, 1), rotation(1, 2));
			printf("            | %6.3f %6.3f %6.3f | \n", rotation(2, 0),
					rotation(2, 1), rotation(2, 2));
			printf("\n");
			printf("        t = < %0.3f, %0.3f, %0.3f >\n", translation(0),
					translation(1), translation(2));
		}
	}
	std::cout << "bestFitIdx = " << bestFitIdx <<"\n";
	if(bestFitIdx >= 0)
		std::cout << "Recognition result: the selected object is " << items[bestFitIdx] << std::endl;
	return;
}



template<typename T>
RecognizeResults recognize(const CloudDB<T> &models, const pcl::PointCloud<T> &test_cloud)
{
    RecognizeResults results;
    return results;
}




}
