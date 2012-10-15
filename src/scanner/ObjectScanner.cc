#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

#include "../CameraWrapper.h"
#include "../Calibrator.h"
#include "../Segmentor.h"
#include "../FileHandler.h"
#include "../ScannerGUI.h"

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudType;


// TODO: a common superclass for ObjectScanner and ObjectRecognizer. They have a lot incommon:
// recognizer needs almost all of scanner's member variables and some functions

namespace askinect {

	class ObjectScanner {
	private:
		FileHandler fileHandler;
		Segmentor segmentor;
		Calibrator calibrator;
		ScannerGUI gui;
		CameraWrapper cameraWrapper;
		PointCloudType objectCloud;
	public:
		ObjectScanner() :
			fileHandler("../../ObjectModels/"),
			gui(),
			cameraWrapper(gui)
		{

		}
		ScannerGUI& getGUI() {
			return gui;
		}
		FileHandler& getFileHandler() {
			return fileHandler;
		}
		void init() {
			cameraWrapper.startCapturing();
		}

		void run() {
			while(gui.running()) {
				// main loop is empty, as CameraWrapper callback calls directly ScannerGUI::update()
				boost::this_thread::sleep(boost::posix_time::milliseconds (5));
			}

			// in the end, save the cloud to a file
			// TODO: save anytime with a keyboard command
			//const PointCloudType constCloud = const_cast<PointCloudType>(objectCloud);
			std::string objectName = "kahvikuppi";
			// TODO: get cloud data from somewhere to write it
			//fileHandler.writePointCloudToFile(objectName, objectCloud);
		}
	};

}


int main() {
	askinect::ObjectScanner scanner;
	scanner.init();
	scanner.run();
	return 0;
}
