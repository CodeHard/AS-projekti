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
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include "../src/Register.h"
#include "../src/FileHandler.h"

int main ()
{
    askinect::FileHandler files("../../test/data/testregister/");

	std::cout << "Loading point clouds from two files..." << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB> cloud, cloud2;
    files.loadPointCloudFromFile("frame1.pcd", cloud);
    files.loadPointCloudFromFile("frame2.pcd", cloud2);

	std::cout << "Reading files completed. Starting registration..." << std::endl;

    askinect::Register<pcl::PointXYZRGB> reg;
    auto cloud3 = reg.registerNew(cloud);

	files.writePointCloudToFile("pre-result.pcd", cloud3);
	
	std::cout << "First cloud registered. Registering the second cloud..." << std::endl;

    auto cloud4 = reg.registerNew(cloud2);

	std::cout << "All clouds registered. Saving resulting cloud..." << std::endl;

    files.writePointCloudToFile("result.pcd", cloud4);

    return 0;
}