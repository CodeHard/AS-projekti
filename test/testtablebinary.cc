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

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include "../src/ModelRecorder.h"

int main ()
{
    auto directory = "../../test/data/table_binary/";

    std::cout << "Creating model recorder..." << std::endl;

    askinect::ModelRecorder<askinect::SimulatorCamera<pcl::PointXYZRGB>, pcl::PointXYZRGB> rec;
    rec.camera.refreshRate = 30;
    rec.camera.directory = directory;

    std::cout << "Creating cloud viewer..." << std::endl;

    pcl::visualization::CloudViewer v ("Testing table_binary folder");

    std::cout << "Creating lambda function..." << std::endl;

    boost::function<void (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr)> func = [&v] (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud)
    {
      v.showCloud(cloud);
    };

    std::cout << "Registering lambda function as a callback..." << std::endl;

    rec.registerSingleCloudCallback(func);

    std::cout << "Starting model recorder..." << std::endl;

    rec.start();

    std::cout << "Waiting for viewer stop event..." << std::endl;

    while (!v.wasStopped())
    {}

    std::cout << "Viewer was stopped, exiting..." << std::endl;

    return 0;
}