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
#include <boost/make_shared.hpp>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>


namespace askinect
{

template<typename T>
class Register
{
private:
    pcl::PointCloud<T> previousCloud;
    bool isInitialized;

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
        sor.setLeafSize (0.03f, 0.03f, 0.03f);
        typename pcl::PointCloud<T>::Ptr output(new pcl::PointCloud<T>);
        sor.filter (*output);

        return output;
    }

    typename pcl::PointCloud<pcl::Normal>::Ptr getNormals(typename pcl::PointCloud<T>::Ptr cloud)
    {
        // Create the normal estimation class, and pass the input dataset to it
        pcl::NormalEstimation<T, pcl::Normal> ne;
        ne.setInputCloud (cloud);

        // Create an empty kdtree representation, and pass it to the normal estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        pcl::search::KdTree<T>::Ptr tree (new pcl::search::KdTree<T> ());
        ne.setSearchMethod (tree);

        // Output datasets
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

        // Use all neighbors in a sphere of radius 3cm
        ne.setRadiusSearch (0.03);

        // Compute the features
        ne.compute (*cloud_normals);

        return cloud_normals;
    }

    typename const pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr getFeatures(typename pcl::PointCloud<T>::Ptr cloud)
    {
        pcl::PointCloud<pcl::Normal>::Ptr normals = getNormals(cloud);
        pcl::FPFHEstimationOMP<T, pcl::Normal, pcl::FPFHSignature33> fpfh;
        fpfh.setInputCloud (cloud);
        fpfh.setInputNormals (normals);
        pcl::search::KdTree<T>::Ptr tree (new pcl::search::KdTree<T>);
        fpfh.setSearchMethod (tree);
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());
        fpfh.setRadiusSearch (0.05);
        fpfh.compute (*fpfhs);

        const pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr returnVal(fpfhs);
        return returnVal;
    }

public:
    Register() : isInitialized(false) {}
    ~Register() {}

    const pcl::PointCloud<T> &registerNew(const pcl::PointCloud<T> &newCloud)
    {
        typename pcl::PointCloud<T>::Ptr filtered = initialFilter(newCloud);

        if (isInitialized)
        {
			askinect::FileHandler files("../../test/data/testregister/");

			files.writePointCloudToFile("filtered.pcd", *filtered);

            typename pcl::PointCloud<T>::Ptr previousCloudPtr(new pcl::PointCloud<T>);
            *previousCloudPtr = previousCloud;

            pcl::SampleConsensusInitialAlignment<T, T, pcl::FPFHSignature33> sac;

            std::cout << "Getting features..." << std::endl;

            sac.setInputSource(filtered);
            sac.setSourceFeatures(getFeatures(filtered));

            sac.setInputTarget(previousCloudPtr);
            sac.setTargetFeatures(getFeatures(previousCloudPtr));

            typename pcl::PointCloud<T>::Ptr alignedCloud(new pcl::PointCloud<T>);

            std::cout << "Doing first alignment..." << std::endl;

            sac.align(*alignedCloud);

			files.writePointCloudToFile("first-alignment.pcd", *alignedCloud);

            pcl::IterativeClosestPoint<T, T> icp;

            icp.setInputSource(alignedCloud);

            icp.setInputTarget(previousCloudPtr);

            // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
            icp.setMaxCorrespondenceDistance (0.2);
            icp.setRANSACOutlierRejectionThreshold(0.2);
            // Set the maximum number of iterations (criterion 1)
            icp.setMaximumIterations (50);
            // Set the transformation epsilon (criterion 2)
            icp.setTransformationEpsilon (1e-8);
            // Set the euclidean distance difference epsilon (criterion 3)
            icp.setEuclideanFitnessEpsilon (10);

            pcl::PointCloud<T> alignedCloud2;

            std::cout << "Doing ICP..." << std::endl;

            icp.align(alignedCloud2);

            std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
            std::cout << icp.getFinalTransformation() << std::endl;

            previousCloud = alignedCloud2;

        }
        else
        {
            previousCloud = *filtered;
            isInitialized = true;
        }

        return previousCloud;
    }
};

}
