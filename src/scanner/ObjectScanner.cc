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

#include "../CameraWrapper.h"
#include "../Calibrate.h"
#include "../Segment.h"
#include "../FileHandler.h"
#include "../ScannerGUI.h"

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudType;


// TODO: a common superclass for ObjectScanner and ObjectRecognizer. They have a lot incommon:
// recognizer needs almost all of scanner's member variables and some functions

namespace askinect
{

class ObjectScanner
{
private:
    FileHandler fileHandler;
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
    ScannerGUI &getGUI()
    {
        return gui;
    }
    FileHandler &getFileHandler()
    {
        return fileHandler;
    }
    void init()
    {
        cameraWrapper.startCapturing();
    }

    void run()
    {
        while (gui.running())
        {
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


int main()
{
    askinect::ObjectScanner scanner;
    scanner.init();
    scanner.run();
    return 0;
}
