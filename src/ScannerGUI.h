#ifndef CODEHARD_SCANNER_GUI_H
#define CODEHARD_SCANNER_GUI_H

#include <pcl/visualization/cloud_viewer.h>

class ScannerGUI {
private:
	pcl::visualization::CloudViewer viewer;
public:
	void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* none) {
		if (event.getKeySym() == "space" && event.keyDown()) {
			std::cout << "space pressed!" << std::endl;
		}
	}

	ScannerGUI() : viewer("openni viewer") {
		// TODO: fix this, but how?
		//viewer.registerKeyboardCallback(keyboardCallback);
	}

	~ScannerGUI() {
	}


	bool running() {
		return !viewer.wasStopped();
	}
	/*	\brief Visualizes point cloud and possibly handles keyboard input (?)
	 *
	 */
	void update(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {

		// visualize
		viewer.showCloud(cloud);
	}



};

#endif
