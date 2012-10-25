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

#include <pcl/io/openni_grabber.h>
#include <boost/function.hpp>
#include <boost/regex.hpp>
#include <boost/make_shared.hpp>
#include <pcl/point_cloud.h>
#include <boost/function.hpp>
#include <boost/signals2/signal.hpp>
#include "ScannerGUI.h"

class boost::signals2::connection;


namespace askinect
{

template<class PointT>
class CameraWrapper
{
private:
    pcl::Grabber *interface;
    pcl::PointCloud<PointT> data;

    //pcl::PointCloud<PointT>::ConstPtr & data;
public:

    boost::signals2::signal<void (typename pcl::PointCloud<PointT>::ConstPtr const &)> dataSignal;

    CameraWrapper()
    {
        interface = new pcl::OpenNIGrabber();
        // make callback function that is called as a new frame is available
        boost::function<void (const typename pcl::PointCloud<PointT>::ConstPtr &)> f = boost::bind (&CameraWrapper::capture_cb_, this, _1);
        interface->registerCallback(f);
    }
    ~CameraWrapper()
    {

    }

    template<typename A>
    boost::signals2::connection routeDepthDataTo(const boost::function<A>& callbackFunction)
    {
    	return dataSignal.connect(callbackFunction);
    }

    void startCapturing()
    {
        interface->start();
    }
    void stopCapturing()
    {
        interface->stop();
    }
/*
    template<typename T>
    boost::signals2::connection registerCallback(const boost::function<T> &callback)
    {
        return dataSignal.connect(callback);
    }
*/

    void capture_cb_ (const typename pcl::PointCloud<PointT>::ConstPtr &cloud)
    {
    	dataSignal();
    	//data = pcl::PointCloud<PointT>(cloud);
        //dataSignal(boost::make_shared(data));
    }

};

}
