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

#include <pcl/visualization/cloud_viewer.h>

class ObjectScanner;

namespace askinect
{

//template <typename CameraT, typename PointT>
class ScannerGUI
{
private:
    pcl::visualization::CloudViewer viewer;
    //ObjectScanner<CameraT, PointT>& scanner;
public:
    void keyboardCallback(const pcl::visualization::KeyboardEvent &event, void* none)
    {
        if (event.getKeySym() == "space" && event.keyDown())
        {
            std::cout << "space pressed!" << std::endl;
            //scanner.doSomething();
        }
        if (event.getKeyCode() == 39 && event.keyDown())	// choose next pointcloud by right arrow
        {
            std::cout << "moving to next pointcloud" << std::endl;
        }
		if (event.getKeyCode() == 40 && event.keyDown())	// choose previous pointcloud by left arrow
		{
			std::cout << "moving to previous pointcloud" << std::endl;
		}
    }

    ScannerGUI() : viewer("openni viewer")
    {
        // TODO: fix this, but how?
    	//boost::function<void (const pcl::visualization::KeyboardEvent &, void*)> f =
        //boost::bind (&ScannerGUI::keyboardCallback, this, _1);
    	//viewer.registerKeyboardCallback(f);
    }

    ~ScannerGUI()
    {
    }


    bool running()
    {
        return !viewer.wasStopped();
    }
    /*  \brief Visualizes point cloud and possibly handles keyboard input (?)
     *	TODO: update to template
     */
    void update(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
    {

        // visualize
        viewer.showCloud(cloud);
    }

    void drawRawData(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {
        viewer.showCloud(cloud);
    }

    void drawSegments(typename pcl::PointCloud<pcl::PointXYZRGBA>::CloudVectorType& clouds) {

    	for(unsigned short i=0; i<clouds.size(); i++) {
    		pcl::PointCloud<pcl::PointXYZ> singleColorCloud;
    		pcl::PointCloud<pcl::PointXYZRGBA> coloredCloud;
    		pcl::copyPointCloud(clouds.at(i), singleColorCloud);
    		pcl::copyPointCloud(singleColorCloud, coloredCloud);
    		viewer.showCloud(coloredCloud.makeShared());
    	}
    }




};

}
