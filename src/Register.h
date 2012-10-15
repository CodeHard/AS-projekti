#pragma once
#include <pcl/point_cloud.h>

namespace askinect {

  template<typename T>
  class Register {
    private:
      pcl::PointCloud<T> previous_cloud;

    public:
      Register() {}
      ~Register() {}

      const pcl::PointCloud<T> &RegisterNew(const pcl::PointCloud<T> &new_cloud) {
        pcl::PointCloud<T> cld;
        previous_cloud = cld;
        return previous_cloud;
      }
  };

}