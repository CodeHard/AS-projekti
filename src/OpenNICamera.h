#pragma once
#include <pcl/io/openni_grabber.h>
#include <boost/function.hpp>

class boost::signals2::connection;

namespace askinect {

  class OpenNICamera {
  private:
    pcl::OpenNIGrabber interface;

  public:
    OpenNICamera();

    ~OpenNICamera() {}

    template<typename T>
    boost::signals2::connection registerCallback(const boost::function<T> &callback)
    {
      return interface.registerCallback(callback);
    }
  };

}