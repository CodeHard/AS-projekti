/*
* At the moment just some smoke testing.
*/

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
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

void testCalibrate() {
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  auto calibrated_cloud = askinect::calibrate(cloud);
}

void testRegister() {
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  askinect::Register<pcl::PointXYZRGB> reg;
  auto adjusted_cloud = reg.registerNew(cloud);
}

void testFilter() {
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  askinect::Filter<pcl::PointXYZRGB> filter;
  auto model = filter.updateModel(cloud); 
}

void testSegment() {
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  auto segments = askinect::segment(cloud);
}

void testCloudDB() {
  askinect::CloudDB<pcl::PointXYZRGB> clouds;
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  clouds.addCloud("cloud", cloud);
  auto get_cloud = clouds.getCloud("cloud");
}

void testRecognize() {
  askinect::CloudDB<pcl::PointXYZRGB> db;
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  auto results = askinect::recognize(db, cloud);
}

void testRecognizeResults() {
  askinect::RecognizeResults results;
  results.addResults("test", 1);
  auto get_result = results.getResults("test");
}

void testSimulatorCamera() {
  askinect::SimulatorCamera<pcl::PointXYZRGB> cam;
  cam.refreshRate = 100;
  cam.filename = "testing";
}

void testOpenNICamera() {
  askinect::OpenNICamera cam;
}

int main() {
  
  testCalibrate();
  testRegister();
  testFilter();
  testSegment();
  testCloudDB();
  testRecognize();
  testRecognizeResults();
  testSimulatorCamera();

  // aborts without camera
  //testOpenNICamera();

  return 0;
}