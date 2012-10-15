/*
* At the moment just some smoke testing.
*/

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "../src/OpenNICamera.h"
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
  auto calibrated_cloud = askinect::Calibrate(cloud);
}

void testRegister() {
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  askinect::Register<pcl::PointXYZRGB> reg;
  auto adjusted_cloud = reg.RegisterNew(cloud);
}

void testFilter() {
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  askinect::Filter<pcl::PointXYZRGB> filter;
  auto model = filter.UpdateModel(cloud); 
}

void testSegment() {
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  auto segments = askinect::Segment(cloud);
}

void testCloudDB() {
  askinect::CloudDB<pcl::PointXYZRGB> clouds;
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  clouds.AddCloud("cloud", cloud);
  auto get_cloud = clouds.GetCloud("cloud");
}

void testRecognize() {
  askinect::CloudDB<pcl::PointXYZRGB> db;
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  auto results = askinect::Recognize(db, cloud);
}

void testRecognizeResults() {
  askinect::RecognizeResults results;
  results.AddResults("test", 1);
  auto get_result = results.GetResults("test");
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

  // aborts without camera
  testOpenNICamera();

  return 0;
}