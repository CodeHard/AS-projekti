#pragma once
#include <pcl/point_cloud.h>
#include "RecognizeResults.h"

namespace askinect {

  template<typename T>
  RecognizeResults recognize(const CloudDB<T> &models, const pcl::PointCloud<T> &test_cloud) {
    RecognizeResults results;
    return results;
  }

}