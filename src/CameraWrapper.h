#pragma once

#include <pcl/io/openni_grabber.h>
#include "ScannerGUI.h"

namespace askinect {

	class CameraWrapper {
	private:
		pcl::Grabber* interface;
		ScannerGUI& gui;
		//pcl::PointCloud<pcl::PointXYZRGB> data;
		//pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & data;
	public:
		CameraWrapper(ScannerGUI& gui) : gui(gui) {
			interface = new pcl::OpenNIGrabber();

		    // make callback function from member function
		    boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
		    boost::bind (&CameraWrapper::capture_cb_, this, _1);

			interface->registerCallback(f);
		}
		~CameraWrapper() {

		}
		void startCapturing() {
			interface->start();
		}
		void stopCapturing() {
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