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
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <vector>

namespace askinect
{

template<typename T>
class Segment
{
private:
    typename pcl::PointCloud<T> cloudCopy;
    typename pcl::PointCloud<T>::CloudVectorType segments;
	pcl::SACSegmentation<T> seg;
	pcl::ModelCoefficients::Ptr coefficients;
	pcl::PointIndices::Ptr inliers;
public:
    Segment() : coefficients (new pcl::ModelCoefficients), inliers (new pcl::PointIndices)
    {
		// Optional
		seg.setOptimizeCoefficients (true);
		// Mandatory
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setDistanceThreshold (0.03);
    }
    ~Segment() {}


	typename pcl::PointCloud<T>::CloudVectorType& segment(const typename pcl::PointCloud<T>::ConstPtr &cloud)
	{
		segments.clear();
		pcl::copyPointCloud(*cloud, cloudCopy);
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
			pcl::ExtractIndices<T> extract;
			extract.setInputCloud (cloudCopy.makeShared());
			extract.setIndices (inliers);

			// Indices that belong to cloud are added to segment vector
			extract.setNegative (false);
			pcl::PointCloud<T> segment;
			extract.filter (segment);
			segments.push_back(segment);

			// Indices that did not belong to the found segment are left to cloudCopy and searched for other segments
			extract.setNegative (true);
			extract.filter(cloudCopy);
			//cloud_filtered.swap (cloud_f);
			//pcl::PointCloud<T> segment(cloudCopy, *inliers);
		}


		return segments;
	}
};

}
