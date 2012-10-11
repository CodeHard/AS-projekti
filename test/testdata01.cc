#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

 int main ()
 {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("../../test/data/data01.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file ../../test/data/data01.pcd \n");
    return (-1);
  }

   pcl::visualization::CloudViewer v ("Testing");
   v.showCloud(cloud);
   int i = 0;
   while (!v.wasStopped())
   {}
   return 0;
 }