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
#include <iostream>
#include <string>
#include <boost/make_shared.hpp>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>
#include <pcl/ros/conversions.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace pcl::registration;
using namespace pcl::common;

namespace askinect
{

template<typename T>
class Register
{
private:
	typename pcl::PointCloud<T>::Ptr previousCloudFiltered;
	Eigen::Matrix4f cumulativeTransform;
	bool isInitialized;

	////////////////////////////////////////////////////////////////////////////////
	void
	estimateKeypoints (typename const PointCloud<T>::Ptr &src,
					   typename const PointCloud<T>::Ptr &tgt,
					   const PointCloud<Normal>::Ptr &srcNormals,
					   const PointCloud<Normal>::Ptr &tgtNormals,
					   PointCloud<T> &keypoints_src,
					   PointCloud<T> &keypoints_tgt)
	{
		PointCloud<PointWithScale> keypoints_src_scale, keypoints_tgt_scale;

		SIFTKeypoint<T, PointWithScale> sift;

		sift.setScales(0.001, 3, 2);
		sift.setMinimumContrast(5);

		sift.setInputCloud(src);
		sift.compute(keypoints_src_scale);

		sift.setInputCloud(tgt);
		sift.compute(keypoints_tgt_scale);

		copyPointCloud(keypoints_src_scale, keypoints_src);
		copyPointCloud(keypoints_tgt_scale, keypoints_tgt);
	}

	////////////////////////////////////////////////////////////////////////////////
	void
	estimateNormals (typename const PointCloud<T>::Ptr &src,
					 typename const PointCloud<T>::Ptr &tgt,
					 PointCloud<Normal> &normals_src,
					 PointCloud<Normal> &normals_tgt)
	{
		NormalEstimationOMP<T, Normal> normal_est;
		normal_est.setInputCloud (src);
		normal_est.setRadiusSearch (0.03);  // 3cm
		normal_est.compute (normals_src);

		normal_est.setInputCloud (tgt);
		normal_est.compute (normals_tgt);
	}

	////////////////////////////////////////////////////////////////////////////////
	void
	estimateFPFH (typename const PointCloud<T>::Ptr &src,
				  typename const PointCloud<T>::Ptr &tgt,
				  typename const PointCloud<Normal>::Ptr &normals_src,
				  typename const PointCloud<Normal>::Ptr &normals_tgt,
				  typename const PointCloud<T>::Ptr &keypoints_src,
				  typename const PointCloud<T>::Ptr &keypoints_tgt,
				  PointCloud<FPFHSignature33> &fpfhs_src,
				  PointCloud<FPFHSignature33> &fpfhs_tgt)
	{
		FPFHEstimationOMP<T, Normal, FPFHSignature33> fpfh_est;
		fpfh_est.setInputCloud (keypoints_src);
		fpfh_est.setInputNormals (normals_src);
		fpfh_est.setRadiusSearch (0.06); // 6cm
		fpfh_est.setSearchSurface (src);
		fpfh_est.compute (fpfhs_src);

		fpfh_est.setInputCloud (keypoints_tgt);
		fpfh_est.setInputNormals (normals_tgt);
		fpfh_est.setSearchSurface (tgt);
		fpfh_est.compute (fpfhs_tgt);
	}

	////////////////////////////////////////////////////////////////////////////////
	void
	findCorrespondences (const PointCloud<FPFHSignature33>::Ptr &fpfhs_src,
						 const PointCloud<FPFHSignature33>::Ptr &fpfhs_tgt,
						 Correspondences &all_correspondences)
	{
		CorrespondenceEstimation<FPFHSignature33, FPFHSignature33> est;
		est.setInputSource (fpfhs_src);
		est.setInputTarget (fpfhs_tgt);
		est.determineReciprocalCorrespondences (all_correspondences);
	}

	////////////////////////////////////////////////////////////////////////////////
	void
	rejectBadCorrespondences (const CorrespondencesPtr &all_correspondences,
							  typename const PointCloud<T>::Ptr &keypoints_src,
							  typename const PointCloud<T>::Ptr &keypoints_tgt,
							  Correspondences &remaining_correspondences)
	{
		CorrespondenceRejectorSampleConsensus<T> rej;
		rej.setInputSource (keypoints_src);
		rej.setInputTarget (keypoints_tgt);
		rej.setInputCorrespondences (all_correspondences);
		rej.getCorrespondences (remaining_correspondences);
	}


	////////////////////////////////////////////////////////////////////////////////
	void
	computeTransformation (typename const PointCloud<T>::Ptr &src,
						   typename const PointCloud<T>::Ptr &tgt,
						   Eigen::Matrix4f &transform,
						   int &numberOfCorrespondences)
	{
		// Compute normals for all points keypoint
		PointCloud<Normal>::Ptr normals_src (new PointCloud<Normal>),
				   normals_tgt (new PointCloud<Normal>);
		estimateNormals (src, tgt, *normals_src, *normals_tgt);
		print_info ("Estimated %d and %d normals for the source and target datasets.\n", normals_src->points.size (), normals_tgt->points.size ());

		// Get an uniform grid of keypoints
		typename PointCloud<T>::Ptr keypoints_src (new PointCloud<T>),
				 keypoints_tgt (new PointCloud<T>);

		estimateKeypoints (src, tgt, normals_src, normals_tgt, *keypoints_src, *keypoints_tgt);
		print_info ("Found %d and %d keypoints for the source and target datasets.\n", keypoints_src->points.size (), keypoints_tgt->points.size ());

		askinect::FileHandler files("../../test/data/testregister/");
		files.writePointCloudToFile("keypoints1.pcd", *keypoints_src);
		files.writePointCloudToFile("keypoints2.pcd", *keypoints_tgt);

		// Compute FPFH features at each keypoint
		PointCloud<FPFHSignature33>::Ptr fpfhs_src (new PointCloud<FPFHSignature33>),
				   fpfhs_tgt (new PointCloud<FPFHSignature33>);
		estimateFPFH (src, tgt, normals_src, normals_tgt, keypoints_src, keypoints_tgt, *fpfhs_src, *fpfhs_tgt);

		// Copy the data and save it to disk
		/*  PointCloud<PointNormal> s, t;
		  copyPointCloud<PointXYZ, PointNormal> (*keypoints_src, s);
		  copyPointCloud<Normal, PointNormal> (normals_src, s);
		  copyPointCloud<PointXYZ, PointNormal> (*keypoints_tgt, t);
		  copyPointCloud<Normal, PointNormal> (normals_tgt, t);*/

		// Find correspondences between keypoints in FPFH space
		CorrespondencesPtr all_correspondences (new Correspondences),
						   good_correspondences (new Correspondences);
		findCorrespondences (fpfhs_src, fpfhs_tgt, *all_correspondences);

		// Reject correspondences based on their XYZ distance
		rejectBadCorrespondences (all_correspondences, keypoints_src, keypoints_tgt, *good_correspondences);

		std::cout << "Found good correspondences: " << good_correspondences->size() << std::endl;

		numberOfCorrespondences = good_correspondences->size();

		// Obtain the best transformation between the two sets of keypoints given the remaining correspondences
		TransformationEstimationSVD<T, T> trans_est;
		trans_est.estimateRigidTransformation (*keypoints_src, *keypoints_tgt, *good_correspondences, transform);
	}

	typename pcl::PointCloud<T>::Ptr initialFilter(const pcl::PointCloud<T> &cloud)
	{

		// remove nans
		typename pcl::PointCloud<T>::Ptr filtered(new pcl::PointCloud<T>);
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(cloud, *filtered, indices);

		// limit z-distance
		pcl::PassThrough<T> pass;
		pass.setInputCloud (filtered);
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (-1.5, 1.5);
		pass.filter (*filtered);

		// down sample
		pcl::VoxelGrid<T> sor;
		sor.setInputCloud (filtered);
		sor.setLeafSize (0.01f, 0.01f, 0.01f);
		typename pcl::PointCloud<T>::Ptr output(new pcl::PointCloud<T>);
		sor.filter (*output);

		return output;
	}

public:
	Register() : isInitialized(false) {}
	~Register() {}

	const pcl::PointCloud<T> registerNew(const pcl::PointCloud<T> &newCloud)
	{
		if (isInitialized)
		{
			typename pcl::PointCloud<T>::Ptr filtered = initialFilter(newCloud);

			// Compute the best transformtion
			Eigen::Matrix4f transform;
			int numberOfCorrespondences;
			computeTransformation (filtered, previousCloudFiltered, transform, numberOfCorrespondences);

			if (numberOfCorrespondences < 10)
			{
				std::cout << "bad frame, skipping it" << std::endl;
				PointCloud<T> cloud;
				return cloud;
			}

			cumulativeTransform *= transform;

			/*typename PointCloud<T>::Ptr newFiltered (new PointCloud<T>);
			transformPointCloud(*filtered, *newFiltered, transform);

			IterativeClosestPoint<T, T> icp;

			icp.setInputSource(newFiltered);
			icp.setInputTarget(previousCloudFiltered);

			PointCloud<T> final;
			icp.align(final);

			std::cout << "icp has converged: " << icp.hasConverged() << std::endl;
			std::cout << "final transformation: " << icp.getFinalTransformation() << std::endl;

			cumulativeTransform *= icp.getFinalTransformation();*/

			PointCloud<T> returnCloud;
			transformPointCloud(newCloud, returnCloud, cumulativeTransform);

			previousCloudFiltered = filtered;

			return returnCloud;

		}
		else
		{
			previousCloudFiltered = initialFilter(newCloud);
			cumulativeTransform = Eigen::Matrix4f::Identity();
			isInitialized = true;

			return newCloud;
		}
	}
};

}
