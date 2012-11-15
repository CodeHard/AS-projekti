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

#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

#include "ScannerGUI.h"
#include "../Calibrate.h"
#include "../Segment.h"
#include "../FileHandler.h"
#include "../ModelRecorder.h"
#include "../OpenNICamera.h"
#include "../DataTypes.h"

typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudType;
typedef pcl::PointXYZRGBA PointType;
typedef askinect::OpenNICamera CameraType;
typedef pcl::PointXYZRGB FilePointType;

// TODO: a common superclass for ObjectScanner and ObjectRecognizer. They have a lot incommon:
// recognizer needs almost all of scanner's member variables and some functions

namespace askinect
{

template <typename CameraT, typename PointT>
class ObjectScanner
{
private:
    FileHandler<FilePointType> fileHandler;
    ScannerGUI gui;
    ModelRecorder<CameraT, PointT> modelRecorder;
    PointCloudType objectCloud;
    unsigned int frameCounter;
public:
    ObjectScanner(std::string outputFolder) :
        fileHandler(outputFolder),
        gui(),
        modelRecorder(),
        frameCounter(0)
    {
    	boost::function<void (const typename pcl::PointCloud<PointT>::ConstPtr &)> f =
    	            boost::bind (&ObjectScanner::newBackground_cb_, this, _1);
    	modelRecorder.registerSingleCloudCallback(f);

    	boost::function<void (typename askinect::ModelData<PointT>)> f2 =
    	    	    	            boost::bind (&ObjectScanner::modelData_cb_, this, _1);
    	modelRecorder.registerModelDataCallback(f2);

    	boost::function<void (std::string)> f3 =
    	            boost::bind (&ObjectScanner::saveObject_cb_, this, _1);
    	gui.registerSaveObjectCallback(f3);


    }

    ScannerGUI &getGUI()
    {
        return gui;
    }
    FileHandler<PointT> &getFileHandler()
    {
        return fileHandler;
    }
    void init()
    {
        modelRecorder.start();
    }

    void saveObject_cb_(std::string objectName) {
    	std::cout << "OS: Saving object\n";
    	//convert to XYZRGB
    	auto currentCloud = gui.getCurrentCloud();
    	pcl::PointCloud<FilePointType> saveable;
    	pcl::copyPointCloud(currentCloud, saveable);
    	fileHandler.writePointCloudToFile(objectName, saveable);
    }

    //NOTE: this version of newdata callback is meant for recording data for SimulatorCamera, not actual object scanning
    void record_cb_ (const typename pcl::PointCloud<PointT>::ConstPtr & cloud) {
    	// show single cloud
    	//gui.drawRawData(cloud);
    	std::stringstream file_name;
		file_name << "frame";
		file_name.fill('0');
		file_name.width(4);
		file_name << frameCounter++ << ".pcd";
		std::cout << file_name.str() << std::endl;
		fileHandler.writePointCloudToFile(file_name.str(), cloud);
    }

    void newBackground_cb_ (const typename pcl::PointCloud<PointT>::ConstPtr & cloud) {
    	gui.updateBackgroundData(cloud);
    }

    void modelData_cb_ ( typename askinect::ModelData<PointT> model) {
		if (!model.segments.empty()) {
			if(gui.wantsData()) {
				gui.clearViewer();
				gui.updateBackgroundData(model.rawCloud);
				gui.updateSegmentData(model.segments);
				gui.drawAll();
				gui.drawAll();
				gui.gotData();
			}
		}
    }

    /* \brief This function is possibly not needed
     *
     */
    void run()
    {
        while (gui.running())
        {
            // main loop is empty, as cloud data is routed directly to ScannerGUI::update(), by boost signals
            boost::this_thread::sleep(boost::posix_time::milliseconds (5));

        	gui.update();

        }
    }
};

}


int main(int argc, char* argv[])
{
	std::cout << "argc = " << argc << ", argv = " << *argv << endl;
	std::string outputFolder = "/home/tommi/Desktop/AS-projekti/ObjectModels/";
	if(argc > 1)
		outputFolder = argv[1];

	std::cout << "Output folder: " << outputFolder << std::endl;

    askinect::ObjectScanner<CameraType, PointType> scanner(outputFolder);
    scanner.init();
    scanner.run();
    return 0;
}
