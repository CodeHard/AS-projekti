#pragma once
#include <pcl/point_cloud.h>

namespace askinect {

  template<typename T>
  pcl::PointCloud<T> calibrate(const pcl::PointCloud<T> &cloud) {
    pcl::PointCloud<T> new_cloud;
    return new_cloud;
  }

}