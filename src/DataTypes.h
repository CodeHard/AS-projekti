#pragma once
#include <pcl/point_cloud.h>
#include <boost/ptr_container/ptr_vector.hpp>

namespace askinect {

template<typename PointT>
struct ModelData {
	typename pcl::PointCloud<PointT> rawCloud;
	typename pcl::PointCloud<PointT>::CloudVectorType segments;
	typename pcl::PointCloud<PointT> surface;
	// add anything that needs to be passed from algorithms to GUI
	ModelData()
	{
	}
};

}
