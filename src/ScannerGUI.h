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

#include <pcl/visualization/pcl_visualizer.h>
#include <sstream>
#include <boost/ptr_container/ptr_vector.hpp>

class ObjectScanner;

namespace askinect
{

class ScannerGUI
{
private:
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	boost::ptr_vector<pcl::PointCloud<pcl::PointXYZRGB> > coloredSegments;
	boost::ptr_vector<pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> > colorHandlers;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr backgroundCloud;
    //ObjectScanner<CameraT, PointT>& scanner;
public:
    void keyboardCallback(const pcl::visualization::KeyboardEvent &event)
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

    ScannerGUI() : viewer(new pcl::visualization::PCLVisualizer("Object scanner")), coloredSegments(), colorHandlers(), backgroundCloud(new pcl::PointCloud<pcl::PointXYZRGB>)
    {
    	boost::function<void (const pcl::visualization::KeyboardEvent &)> f =
        boost::bind (&ScannerGUI::keyboardCallback, this, _1);
    	viewer->registerKeyboardCallback(f);
    }

    ~ScannerGUI()
    {
    }


    bool running()
    {
        return !viewer->wasStopped();
    }
    /*  \brief Visualizes point cloud and possibly handles keyboard input (?)
     *	TODO: update to template
     */
    void update()
    {
    	std::cout << "gui.update();" << std::endl;
    	viewer->spin();
    	std::cout << "gui.update();" << std::endl;
    }
/*
    template <typename PointT>
    void drawRawData(const typename pcl::PointCloud<PointT>::ConstPtr &cloud) {
    	viewer->removePointCloud("background");
        viewer->addPointCloud<PointT>(cloud, "background");
    }

        template <typename PointT>
    void drawSegments(typename std::vector <pcl::PointCloud<PointT> >& newSegments) {
    */
    void drawRawData(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
    {
    	//std::cout << "drawRaw upd" << std::endl;
		pcl::copyPointCloud(*cloud, *backgroundCloud);
    	if(! viewer->updatePointCloud<pcl::PointXYZRGB>(backgroundCloud, std::string("background")))
    	{
    		//std::cout << "drawRaw add" << std::endl;
    		viewer->addPointCloud<pcl::PointXYZRGB>(backgroundCloud, std::string("background"));
    	}
    	//std::cout << "drawRaw add ready" << std::endl;
	}

    void drawSegments(pcl::PointCloud<pcl::PointXYZRGBA>::CloudVectorType& newSegments) {
    	//std::cout << "drawSeg" << std::endl;
    	for(unsigned short i=0; i<newSegments.size(); i++) {
    		//std::cout << "Seg " << i << std::endl;
    		//std::cout << "newSegments.size()" << newSegments.size() << std::endl;
    		//std::cout << "coloredSegments.size()" << coloredSegments.size() << std::endl;
    		if(coloredSegments.size() <= i) {
    			//std::cout << "Seg " << i << "push back new PC" << std::endl;
    			coloredSegments.push_back(new pcl::PointCloud<pcl::PointXYZRGB>());
    			//std::cout << "Seg " << i << "push back new PC" << std::endl;
    		}
    		//result
    		//pcl::PointCloud<PointT>::Ptr coloredCloud(new pcl::PointCloud<PointT>);

    		// Add the extracted plane to the result point cloud
			uint8_t r, g, b;
			if( i%3 == 0 )
			{
				r = i*48%255;
				g = 0;
				b = 0;
			}
			else if( i%3 == 1 )
			{
				r = 0;
				g = i*48%255;
				b = 0;
			}
			else
			{
				r = 0;
				g = 0;
				b = i*48%255;
			}
			pcl::PointXYZRGB xyzrgb_point = pcl::PointXYZRGB(r, g, b);

			for( int point_number=0; point_number<(int)newSegments.at(i).points.size(); point_number++ )
			{
				xyzrgb_point.x = newSegments.at(i).at(point_number).x;
				xyzrgb_point.y = newSegments.at(i).at(point_number).y;
				xyzrgb_point.z = newSegments.at(i).at(point_number).z;
				coloredSegments.at(i).push_back(xyzrgb_point);
			}

	    	std::stringstream segmentName;
	    	segmentName << "segment" << i;
    		//std::cout << "Seg " << i << "upd PC" << std::endl;
    		if(! viewer->updatePointCloud<pcl::PointXYZRGB>(coloredSegments.at(i).makeShared(), segmentName.str()) )
    		{
    			//colorHandlers.push_back(new pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(coloredSegments.at(i).makeShared()));
    			//std::cout << "Seg " << i << "add PC" << std::endl;
    			// .at(i) probably is a bit unsafe
    			viewer->addPointCloud<pcl::PointXYZRGB>(coloredSegments.at(i).makeShared(), /*colorHandlers.at(i),*/ segmentName.str());
    			//viewer->initCameraParameters ();
    		}
    		//std::cout << "Seg " << i << std::endl;
    	}
    	std::cout << "drawSeg" << std::endl;
    }




};

}
