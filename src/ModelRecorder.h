/*
Copyright (c) 2012 Tommi Tikkanen, Eero Laukkanen

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#pragma once
#include <pcl/point_cloud.h>
#include <boost/function.hpp>
#include <boost/signals2/signal.hpp>
#include "Filter.h"
#include "Register.h"

class boost::signals2::connection;

namespace askinect
{

template<typename CameraT, typename PointT>
class ModelRecorder
{
private:
    Register<PointT> reg;
    Filter<PointT> filter;
    boost::signals2::signal<void (typename std::vector<pcl::PointCloud<PointT> >&)> sig;
public:
    CameraT camera;

    ModelRecorder()
    {
        boost::function<void (const pcl::PointCloud<PointT>::ConstPtr &)> f =
            boost::bind (&ModelRecorder::capture_cb_, this, _1);
        camera.registerCallback(f);
    }

    ~ModelRecorder() {}

    template<typename T>
    boost::signals2::connection registerCallback(const boost::function<T> &callback)
    {
        return sig.connect(callback);
    }

    void capture_cb_(typename pcl::PointCloud<PointT>::ConstPtr const &cloud)
    {
        auto calibrated = calibrate(*cloud);
        auto registered = reg.registerNew(calibrated);
        auto filteredModel = filter.updateModel(registered);
        auto segmented = segment(filteredModel);
        sig(segmented);
    }

    void start()
    {
        cam.start();
    }
};

}