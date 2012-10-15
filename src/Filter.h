#pragma once
#include <pcl/point_cloud.h>

namespace askinect {

  template<typename T>
  class Filter {
    private:
      pcl::PointCloud<T> model;

    public:
      Filter() {}
      ~Filter() {}

      const pcl::PointCloud<T> &UpdateModel(const pcl::PointCloud<T> &new_cloud) {
        pcl::PointCloud<T> cld;
        model = cld;
        return model;
      }
  };

}