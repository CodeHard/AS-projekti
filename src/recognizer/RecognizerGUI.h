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

namespace askinect
{

class RecognizerGUI
{
private:
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	boost::ptr_vector<pcl::PointCloud<pcl::PointXYZRGB> > coloredSegments;
	boost::ptr_vector<pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> > colorHandlers;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr backgroundCloud;
	int currentSegmentIdx;
	bool newDataWanted;
	boost::signals2::signal<void (std::string)> saveObjectSignal;
	ModelData<pcl::PointXYZRGBA> newModel;
	bool gotNewModel;
public:

	RecognizerGUI() : viewer(new pcl::visualization::PCLVisualizer("Object recognizer")),
		coloredSegments(),
		colorHandlers(),
		backgroundCloud(new pcl::PointCloud<pcl::PointXYZRGB>),
		currentSegmentIdx(-1),
		newDataWanted(true),
		gotNewModel(false)
	{
		boost::function<void (const pcl::visualization::KeyboardEvent &)> f =
			boost::bind (&RecognizerGUI::keyboardCallback, this, _1);
		viewer->registerKeyboardCallback(f);



		std::string guidetext = "Keyboard controls:\n  Left/Right arrows = Select segment\n  S = Save model\n  R = Refresh data\n  Q = Quit";
		viewer->addText(guidetext, 250, 30, "guideText");
		viewer->addCoordinateSystem(0.3);
		viewer->setCameraPosition(0.0, 0.0, -3.0, 0.0, -1.0, 0.0, 0);
	}

	void addNewModel(const ModelData<pcl::PointXYZRGBA> &model)
	{
		newModel = model;
		gotData();
		gotNewModel = true;
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
			std::cout << "Recognizing this segment" << std::endl;
			saveObjectSignal(objectName);

		}
		if (event.getKeySym() == "r" && event.keyDown ())
		{
			std::cout << "Refreshing clouds." << std::endl;

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

	void update()
	{
		viewer->spinOnce();
		if (gotNewModel)
		{
			clearViewer();
			updateBackgroundData(newModel.rawCloud);
			updateSegmentData(newModel.segments);
			drawAll();
			drawAll();
			gotNewModel = false;
		}
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
		if ( !viewer->updatePointCloud<pcl::PointXYZRGB>(cloud, id))
		{
			viewer->addPointCloud<pcl::PointXYZRGB>(cloud,  id);
		}
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
				0.2, 0.2, 0.2, id);
	}

	void drawAll()
	{
		drawBackground();
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
