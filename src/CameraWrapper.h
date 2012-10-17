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
#include "ScannerGUI.h"

namespace askinect
{

class CameraWrapper
{
private:
    pcl::Grabber *interface;
    ScannerGUI &gui;
    //pcl::PointCloud<pcl::PointXYZRGB> data;
    //pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & data;
public:
    CameraWrapper(ScannerGUI &gui) : gui(gui)
    {
        interface = new pcl::OpenNIGrabber();

        // make callback function from member function
        boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &)> f =
            boost::bind (&CameraWrapper::capture_cb_, this, _1);

        interface->registerCallback(f);
    }
    ~CameraWrapper()
    {

    }
    void startCapturing()
    {
        interface->start();
    }
    void stopCapturing()
    {
        interface->stop();
    }


    void capture_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
    {
        gui.update(cloud);
        //data = pcl::PointCloud<pcl::PointXYZRGBA>(cloud);
    }
    /*
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & getPointCloud() {
        return data;
    }
    */
};

}