#pragma once
#include <pcl/point_cloud.h>
#include <boost/function.hpp>
#include <boost/signals2/signal.hpp>
#include <string>

class boost::signals2::connection;

namespace askinect {

  template<typename PointT>
  class SimulatorCamera {
  private:
    boost::signals2::signal<void (typename pcl::PointCloud<PointT>::ConstPtr const&)> sig;
  public:
    int refreshRate;
    std::string filename;

    SimulatorCamera() : refreshRate(30), filename("") {}

    ~SimulatorCamera() {}

  	template<typename T>
    boost::signals2::connection registerCallback(const boost::function<T> &callback)
    {
        return sig.connect(callback);
    }

    void start();
  };

}