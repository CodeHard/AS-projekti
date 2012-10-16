#pragma once
#include <map>
#include <string>
#include <pcl/point_cloud.h>
#include "CloudModel.h"

namespace askinect {

  template<typename T>
  class CloudDB {
    private:
      std::map<std::string, CloudModel<T> > clouds;

    public:
      CloudDB() {}
      ~CloudDB() {}

      void addCloud(std::string id, const pcl::PointCloud<T> &cloud) {
        auto cloud_model = CloudModel<T>(id, cloud);
        clouds[id] = cloud_model;
      }

      const CloudModel<T> &getCloud(std::string id) {
        return clouds[id];
      }
  };

}