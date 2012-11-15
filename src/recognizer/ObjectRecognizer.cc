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

#include "RecognizerGUI.h"
#include "Recognize.h"
#include "../Calibrate.h"
#include "../Segment.h"
#include "../FileHandler.h"
#include "../ModelRecorder.h"
#include "../OpenNICamera.h"
#include "../DataTypes.h"
#include "../DatabaseReader.h"

//#include <pcl/recognition/impl/voxel_structure.hpp>
//#include <pcl/recognition/impl/obj_rec_ransac.hpp>

typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudType;
typedef pcl::PointXYZRGBA PointType;
typedef askinect::OpenNICamera CameraType;
typedef pcl::PointXYZRGB FilePointType;

// TODO: a common superclass for ObjectRecognizer and ObjectRecognizer. They have a lot incommon:
// recognizer needs almost all of recognizer's member variables and some functions

namespace askinect
{

template <typename CameraT, typename PointT>
class ObjectRecognizer
{
private:
	DatabaseReader<FilePointType> dbReader;
    RecognizerGUI gui;
    ModelRecorder<CameraT, PointT> modelRecorder;
    PointCloudType scene;
    PointCloudType objectCloud;
    unsigned int frameCounter;
public:
    ObjectRecognizer(std::string inputFolder) :
        //fileHandler(outputFolder),
    	dbReader(inputFolder),
        gui(),
        modelRecorder(),
        frameCounter(0)
    {
    	boost::function<void (const typename pcl::PointCloud<PointT>::ConstPtr &)> f =
    	            boost::bind (&ObjectRecognizer::newBackground_cb_, this, _1);
    	modelRecorder.registerSingleCloudCallback(f);

    	boost::function<void (typename askinect::ModelData<PointT>)> f2 =
    	    	    	            boost::bind (&ObjectRecognizer::modelData_cb_, this, _1);
    	modelRecorder.registerModelDataCallback(f2);

    	boost::function<void (std::string)> f3 =
    	            boost::bind (&ObjectRecognizer::recognize_cb_, this, _1);
    	gui.registerSaveObjectCallback(f3);


    }

    RecognizerGUI &getGUI()
    {
        return gui;
    }
    DatabaseReader<PointT> &getDBReader()
    {
        return dbReader;
    }
    void init()
    {
        modelRecorder.start();
        dbReader.loadFiles();
        std::cout << "files loaded" << std::endl;
    }

    void recognize_cb_(std::string turha) {
    	std::cout << "Loading files\n";
    	//boost::ptr_vector <pcl::PointCloud<FilePointType> >& refModels = dbReader.getModels();
    	auto refModels = dbReader.getModels();

    	// recognize from scene
    	//recognize(refModels, scene);

    	// recognize from pre-selected segment
    	recognize(refModels, gui.getCurrentCloud());




    	//fileHandler.writePointCloudToFile(objectName, gui.getCurrentCloud());
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
		//fileHandler.writePointCloudToFile(file_name.str(), cloud);
    }

    void newBackground_cb_ (const typename pcl::PointCloud<PointT>::ConstPtr & cloud) {
    	pcl::copyPointCloud(*cloud, scene);
    	//gui.updateBackgroundData(cloud);
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
            // main loop is empty, as cloud data is routed directly to RecognizerGUI::update(), by boost signals
            boost::this_thread::sleep(boost::posix_time::milliseconds (5));

        	gui.update();

        }
    }
};

}


int main(int argc, char* argv[])
{
	std::cout << "argc = " << argc << ", argv = " << *argv << endl;
	std::string inputFolder = "/home/tommi/Desktop/AS-projekti/ObjectModels/";
	if(argc > 1)
		inputFolder = argv[1];

	std::cout << "Output folder: " << inputFolder << std::endl;

    askinect::ObjectRecognizer<CameraType, PointType> recognizer(inputFolder);
    recognizer.init();
    recognizer.run();
    return 0;
}
