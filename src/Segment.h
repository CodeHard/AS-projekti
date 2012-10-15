#pragma once
#include <pcl/point_cloud.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <vector>

namespace askinect {

  template<typename T>
  std::vector<pcl::PointCloud<T> > Segment(const pcl::PointCloud<T> &cloud) {
    std::vector<pcl::PointCloud<T> > segments;
    return segments;
  }

}