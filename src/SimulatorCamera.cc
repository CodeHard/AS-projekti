#include <iomanip>
#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include "SimulatorCamera.h"
#include "FileHandler.h"

namespace askinect {

  template<typename PointT>
  void SimulatorCamera<PointT>::start() {
    std::stringstream ss;
    int i = 0;
    while (i < 1000) {
      ss << filename << std::setfill('0') << std::setw(6) << i << ".pcd";
      pcl::PointCloud<PointT> cloud;
      FileHandler::loadPointCloudFromFile(ss.str(), cloud);
      sig(&cloud);
      i++;

      boost::this_thread::sleep(boost::posix_time::milliseconds(1000 / refreshRate));
    }
  }

}