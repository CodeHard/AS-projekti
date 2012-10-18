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
#include <pcl/point_cloud.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

namespace askinect
{

template<typename T>
class Register
{
private:
    pcl::PointCloud<T> previousCloud;
    bool isInitialized;

public:
    Register() : isInitialized(false) {}
    ~Register() {}

    const pcl::PointCloud<T> &registerNew(const pcl::PointCloud<T> &newCloud)
    {

        if (isInitialized)
        {
            boost::shared_ptr<const pcl::PointCloud<T> > newCloudPtr(&newCloud);
            boost::shared_ptr<const pcl::PointCloud<T> > previousCloudPtr(&previousCloud);

            typename pcl::PointCloud<T>::Ptr filtered_cloud (new pcl::PointCloud<T>);
            typename pcl::ApproximateVoxelGrid<T> approximate_voxel_filter;
            approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);

            approximate_voxel_filter.setInputCloud (newCloudPtr);
            approximate_voxel_filter.filter (*filtered_cloud);

            typename pcl::NormalDistributionsTransform<T, T> ndt;
            // Setting scale dependent NDT parameters
            // Setting minimum transformation difference for termination condition.
            ndt.setTransformationEpsilon (0.01);
            // Setting maximum step size for More-Thuente line search.
            ndt.setStepSize (0.1);
            //Setting Resolution of NDT grid structure (VoxelGridCovariance).
            ndt.setResolution (1.0);
            // Setting max number of registration iterations.
            ndt.setMaximumIterations (35);
            // Setting point cloud to be aligned.
            ndt.setInputCloud (filtered_cloud);
            // Setting point cloud to be aligned to.
            ndt.setInputTarget (previousCloudPtr);

            // Calculating required rigid transform to align the input cloud to the target cloud.
            typename pcl::PointCloud<T>::Ptr output_cloud (new pcl::PointCloud<T>);
            ndt.align (*output_cloud, Eigen::Matrix4f::Identity());

            std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
                      << " score: " << ndt.getFitnessScore () << std::endl;

            // Transforming unfiltered, input cloud using found transform.
            pcl::transformPointCloud (newCloud, *output_cloud, ndt.getFinalTransformation ());

            previousCloud = *output_cloud;

        }
        else
        {

            previousCloud = newCloud;
            isInitialized = true;

        }

        return previousCloud;

    }
};

}
