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

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
namespace askinect
{

template <typename PointT>
class FileHandler
{
private:
	std::string targetDirectory;
	pcl::PCDWriter writer;
	pcl::PCDReader reader;
public:
	FileHandler(std::string targetDirectory) : targetDirectory(targetDirectory)
	{
		writer = pcl::PCDWriter();
		reader = pcl::PCDReader();
	}

    /*  Should this take a pointcloud or sensor_msgs::PointCloud2??
     *
     */
    bool writePointCloudToFile(std::string &filename, typename pcl::PointCloud<PointT> &cloud)
    {

        // copy before writing, in case that reference contents change during the write
        const pcl::PointCloud<PointT> constCloudCopy = cloud;
        const std::string completeFilename = targetDirectory + filename;
        writer.write(completeFilename, constCloudCopy);
        return true;
    }
    /*bool writePointCloudToFile(std::string &filename, typename pcl::PointCloud<pcl::PointXYZRGB> &cloud)
    {

        // copy before writing, in case that reference contents change during the write
        const pcl::PointCloud<pcl::PointXYZRGB> constCloudCopy = cloud;
        const std::string completeFilename = targetDirectory + filename;
        writer.write(completeFilename, constCloudCopy);
        return true;
    }*/

    bool loadPointCloudFromFile(
        const std::string &filename, typename pcl::PointCloud<PointT> &cloud)
    {
        if (reader.read(filename, cloud) == -1)
        {
            PCL_ERROR("Couldn't load file.\n");
            return false;
        }
        return true;
    }
};
}
