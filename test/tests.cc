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

/*
* At the moment just some smoke testing.
*/

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <iostream>
#include "../src/OpenNICamera.h"
#include "../src/SimulatorCamera.h"
#include "../src/Calibrate.h"
#include "../src/Register.h"
#include "../src/Filter.h"
#include "../src/Segment.h"
#include "../src/CloudModel.h"
#include "../src/CloudDB.h"
#include "../src/Recognize.h"
#include "../src/RecognizeResults.h"
#include "../src/ModelRecorder.h"

void testCalibrate()
{
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    auto calibrated_cloud = askinect::calibrate(cloud);
}

void testRegister()
{
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    askinect::Register<pcl::PointXYZRGB> reg;
    auto adjusted_cloud = reg.registerNew(cloud);
}

void testFilter()
{
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    askinect::Filter<pcl::PointXYZRGB> filter;
    auto model = filter.updateModel(cloud);
}

void testSegment()
{
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    auto segments = askinect::segment(cloud);
}

void testCloudDB()
{
    askinect::CloudDB<pcl::PointXYZRGB> clouds;
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    clouds.addCloud("cloud", cloud);
    auto get_cloud = clouds.getCloud("cloud");
}

void testRecognize()
{
    askinect::CloudDB<pcl::PointXYZRGB> db;
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    auto results = askinect::recognize(db, cloud);
}

void testRecognizeResults()
{
    askinect::RecognizeResults results;
    results.addResults("test", 1);
    auto get_result = results.getResults("test");
}

void testSimulatorCamera()
{
    askinect::SimulatorCamera<pcl::PointXYZRGB> cam;
    cam.refreshRate = 100;
    cam.directory = "testing";
}

void testOpenNICamera()
{
    askinect::OpenNICamera cam;
}

void testModelRecorder()
{
    askinect::ModelRecorder<askinect::SimulatorCamera<pcl::PointXYZRGB>, pcl::PointXYZRGB> rec;
    rec.camera.refreshRate = 100;
    rec.camera.directory = "testing";
}

void runTest(void test(void), std::string name)
{
    std::cout << "Running test " << name << std::endl;
    test();
    std::cout << "Finished test " << name << std::endl;
}

int main()
{
    runTest(testCalibrate, "calibration");
    runTest(testRegister, "registration");
    runTest(testFilter, "filtering");
    runTest(testSegment, "segmentation");
    runTest(testCloudDB, "cloud database");
    runTest(testRecognize, "recognition");
    runTest(testRecognizeResults, "recognition results");
    runTest(testSimulatorCamera, "simulator camera");
    runTest(testModelRecorder, "model recorder");

    // aborts without camera
    //testOpenNICamera();

    // enables seeing output
    int i;
    std::cin >> i;

    return 0;
}