#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
namespace askinect {
	class FileHandler {
	private:
		std::string targetDirectory;
		pcl::PCDWriter writer;
		pcl::PCDReader reader;
	public:
		FileHandler(std::string targetDirectory) : targetDirectory(targetDirectory) {
			writer = pcl::PCDWriter();
			reader = pcl::PCDReader();
		}

		/*	Should this take a pointcloud or sensor_msgs::PointCloud2??
		 *
		 */
		bool writePointCloudToFile(std::string& filename, pcl::PointCloud<pcl::PointXYZRGB>& cloud) {

			// copy before writing, in case that reference contents change during the write
			const pcl::PointCloud<pcl::PointXYZRGB> constCloudCopy = cloud;
			const std::string completeFilename = targetDirectory + filename;
			writer.write(completeFilename, constCloudCopy);
			return true;
		}
		bool loadPointCloudFromFile(
				const std::string &filename, pcl::PointCloud<pcl::PointXYZRGB>& cloud) {
			if (reader.read(filename, cloud) == -1) {
				PCL_ERROR("Couldn't load file.\n");
				return false;
			}
			return true;
		}


	};
}
