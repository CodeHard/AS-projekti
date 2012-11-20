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
#include <iostream>
#include <pcl/point_cloud.h>
#include <boost/function.hpp>
#include <boost/signals2/signal.hpp>
#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/filesystem.hpp>
#include <boost/regex.hpp>
#include <boost/make_shared.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include <string>
#include "FileHandler.h"
#include "Common.h"
#include "CloudDB.h"

class boost::signals2::connection;

typedef pcl::PointXYZRGB LoadPointT;

namespace askinect
{

template<typename PointT>
class DatabaseReader
{
private:
    boost::signals2::signal<void (typename pcl::PointCloud<PointT>::ConstPtr const &)> sig;
public:
    std::string directory;
    boost::ptr_vector< pcl::PointCloud<PointT> > referenceModels;

    DatabaseReader(std::string dir) :directory(dir), referenceModels() {}

    ~DatabaseReader() {}

    template<typename T>
    boost::signals2::connection registerCallback(const boost::function<T> &callback)
    {
        return sig.connect(callback);
    }

    boost::ptr_vector< pcl::PointCloud<PointT> >& getModels() {
    	return referenceModels;
    }

    void loadFiles()
    {
        FileHandler<PointT> fileHandler(directory);
        boost::regex filter(".*\\.pcd");
        auto files = getFiles(directory, filter);

        sort(files.begin(), files.end());

        auto file = files.begin();
        while (file != files.end())
        {
            std::cout << "(DatabaseReader.h) Reading file " << *file << std::endl;
            pcl::PointCloud<LoadPointT> cloud;
            fileHandler.loadPointCloudFromFile(*file, cloud);
            typename pcl::PointCloud<PointT>* cloudWithCorrectType = new pcl::PointCloud<PointT>();
            pcl::copyPointCloud(cloud, *cloudWithCorrectType);
            referenceModels.push_back(cloudWithCorrectType);
            //referenceModels.addCloud(std::string(*file), cloudWithCorrectType);
            //sig(boost::make_shared<const pcl::PointCloud<PointT> >(cloud));
            file++;
        }
        std::cout << "loading files complete\n";
    }
};

}
