#pragma once
#include <pcl/point_cloud.h>
#include <string>

namespace askinect {

  template<typename T>
  class CloudModel {
    private:
      pcl::PointCloud<T> cloud;
      std::string id;

    public:
      CloudModel() {}

      CloudModel(std::string id, pcl::PointCloud<T> cloud) {
        id = id;
        cloud = cloud;
      }

      ~CloudModel() {}

      std::string getId() {
        return id;
      }

      pcl::PointCloud<T> getCloud() {
        return cloud;
      }
  };

}