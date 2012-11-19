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
#include "../DataTypes.h"
#include <boost/signals2/signal.hpp>

class boost::signals2::connection;
//class ObjectScanner;

namespace askinect
{

class ScannerGUI
{
private:
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	boost::ptr_vector<pcl::PointCloud<pcl::PointXYZRGB> > coloredSegments;
	boost::ptr_vector<pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> > colorHandlers;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr backgroundCloud;
	int currentSegmentIdx;
	bool newDataWanted;
	boost::signals2::signal<void (std::string)> saveObjectSignal;
public:

	ScannerGUI() : viewer(new pcl::visualization::PCLVisualizer("Object scanner")),
		coloredSegments(),
		colorHandlers(),
		backgroundCloud(new pcl::PointCloud<pcl::PointXYZRGB>),
		currentSegmentIdx(-1),
		newDataWanted(true)
	{
		boost::function<void (const pcl::visualization::KeyboardEvent &)> f =
			boost::bind (&ScannerGUI::keyboardCallback, this, _1);
		viewer->registerKeyboardCallback(f);



		std::string guidetext = "Keyboard controls:\n  Left/Right arrows = Select segment\n  S = Save model\n  R = Refresh data\n  Q = Quit";
		viewer->addText(guidetext, 250, 30, "guideText");
		viewer->addCoordinateSystem(0.3);
		viewer->setCameraPosition(0.0, 0.0, -3.0, 0.0, -1.0, 0.0, 0);
	}

	~ScannerGUI()
	{
	}

	template<typename T>
	boost::signals2::connection registerSaveObjectCallback(
			const boost::function<T> &callback) {
		return saveObjectSignal.connect(callback);
	}

	void keyboardCallback(const pcl::visualization::KeyboardEvent &event)
	{
		if (event.getKeySym() == "s" && event.keyDown())
		{
			std::string objectName;
			std::cout << "Saving, please enter object name:" << std::endl;
			std::cin >> objectName;
			saveObjectSignal(objectName);

		}
		if (event.getKeySym() == "r" && event.keyDown ())
		{
			std::cout << "Refreshing clouds." << std::endl;

			//viewer->removeAllPointClouds();
			//coloredSegments.clear();
			currentSegmentIdx = -1;
			newDataWanted = true;
		}
		if ( (event.getKeySym() == "space" || event.getKeySym() == "Right")  && event.keyDown ())
		{
			currentSegmentIdx++;
			if (currentSegmentIdx >= coloredSegments.size())
				currentSegmentIdx = 0;
			std::cout << "Next plane selected. #" << currentSegmentIdx << std::endl;
			drawColoredSegments();
		}
		if (event.getKeySym() == "Left" && event.keyDown ())
		{
			currentSegmentIdx--;
			if (currentSegmentIdx < 0)
				currentSegmentIdx = coloredSegments.size() - 1;
			if (currentSegmentIdx < 0)
				currentSegmentIdx = 0;
			std::cout << "Previous plane selected. #" << currentSegmentIdx << std::endl;
			drawColoredSegments();
		}
	}


	pcl::PointCloud<pcl::PointXYZRGB>& getCurrentCloud() {
		return coloredSegments.at(currentSegmentIdx);
	}

	bool running()
	{
		return !viewer->wasStopped();
	}
	/*  \brief Visualizes point cloud and possibly handles keyboard input (?)
	 *  TODO: update to template
	 */
	void update()
	{
		viewer->spin();
	}

	template <typename PointT>
	void addOrUpdateCloud(typename pcl::PointCloud<PointT>::Ptr cloud, std::string id)
	{
		if ( !viewer->updatePointCloud(cloud, id))
		{
			viewer->addPointCloud<PointT>(cloud, id);
		}
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
				cloud->at(0).r, cloud->at(0).g, cloud->at(0).b, id);
	}

	// same as addOrUpdateCloud, but with different color
	template <typename PointT>
	void addOrUpdateSelectedCloud(typename pcl::PointCloud<PointT>::Ptr cloud, std::string id)
	{
		if ( !viewer->updatePointCloud(cloud, id))
		{
			viewer->addPointCloud<PointT>(cloud, id);
		}
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
				0.9, 0.9, 0.9, id);
	}

	// same as addOrUpdateCloud, but with different color

	void addOrUpdateBackground(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string id)
	{
		/*
		// purkkaa, rgb colors not showing at all
		if(colorHandlers.size() == 0) {
			colorHandlers.push_back(new pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(cloud));
		}
		else {
			colorHandlers.clear();
			colorHandlers.push_back(new pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(cloud));
		}
		if( !viewer->updatePointCloud(cloud, colorHandlers.at(0), id)) {
			viewer->addPointCloud<pcl::PointXYZRGB>(cloud, colorHandlers.at(0), id);
		}
		*/
		std::cout << "Adding cloud..." << std::endl;
		if ( !viewer->updatePointCloud<pcl::PointXYZRGB>(cloud, id))
		{
			std::cout << "Cloud could not be updated, adding it..." << std::endl;
			viewer->addPointCloud<pcl::PointXYZRGB>(cloud,  id);
			std::cout << "Cloud added." << std::endl;
		}

		std::cout << "Setting rendering options..." << std::endl;
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
				0.2, 0.2, 0.2, id);
	}

	void drawAll()
	{
		std::cout << "Drawing background..." << std::endl;
		drawBackground();
		std::cout << "Drawing segments..." << std::endl;
		drawColoredSegments();
	}

	void clearViewer() {
		viewer->removeAllPointClouds();
	}

	void drawBackground()
	{
		addOrUpdateBackground(backgroundCloud, std::string("background"));
	}

	void drawColoredSegments()
	{
		for (unsigned int i = 0; i < coloredSegments.size(); i++)
		{
			std::stringstream cloudName;
			cloudName << "segment" << i;
			if (i == currentSegmentIdx)
			{
				addOrUpdateSelectedCloud<pcl::PointXYZRGB>(coloredSegments.at(i).makeShared(), cloudName.str());
			}
			else
			{
				addOrUpdateCloud<pcl::PointXYZRGB>(coloredSegments.at(i).makeShared(), cloudName.str());
			}
		}
	}

	bool wantsData() {
		return newDataWanted;
	}

	void gotData() {
		newDataWanted = false;
	}

	void updateBackgroundData(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
	{
		pcl::copyPointCloud(*cloud, *backgroundCloud);
	}

	void updateBackgroundData(pcl::PointCloud<pcl::PointXYZRGBA> cloud)
	{
		pcl::copyPointCloud(cloud, *backgroundCloud);
	}

	void updateSegmentData(pcl::PointCloud<pcl::PointXYZRGBA>::CloudVectorType newSegments)
	{
		coloredSegments.clear();
		// make a colored copy of the segments for drawing
		for (unsigned short i = 0; i < newSegments.size(); i++)
		{
			if (coloredSegments.size() <= i)
			{
				std::cout << "Coloring segment #" << i << std::endl;
				coloredSegments.push_back(new pcl::PointCloud<pcl::PointXYZRGB>());
			}

			// Add the extracted plane to the result point cloud
			uint8_t r, g, b;
			if ( i % 3 == 0 )
			{
				r = (i + 1) * 48 % 255;
				g = 0;
				b = (255 - i * 50) % 255;
			}
			else if ( i % 3 == 1 )
			{
				r = (255 - i * 20) % 255;;
				g = (i + 1) * 48 % 255;
				b = 0;
			}
			else
			{
				r = 0;
				g = (128 + i * 10) % 255;
				b = (i + 1) * 48 % 255;
			}
			pcl::PointXYZRGB xyzrgb_point = pcl::PointXYZRGB(r, g, b);

			for ( int point_number = 0; point_number < (int)newSegments.at(i).points.size(); point_number++ )
			{
				xyzrgb_point.x = newSegments.at(i).at(point_number).x;
				xyzrgb_point.y = newSegments.at(i).at(point_number).y;
				xyzrgb_point.z = newSegments.at(i).at(point_number).z;
				coloredSegments.at(i).push_back(xyzrgb_point);
			}

			std::stringstream segmentName;
			segmentName << "segment" << i;

			if (currentSegmentIdx < 0)
			{
				currentSegmentIdx = 0;
			}

		}
	}


};

}
