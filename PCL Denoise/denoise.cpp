#include <iostream>
#include <string>
#define _ITERATOR_DEBUG_LEVEL 0
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

int
main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

	// Fill in the cloud data
	pcl::PCDReader reader;
	
	if (argc != 2) {
		std::cerr << "Invalid arguments";
		return 1;
	}
	//float siz;
	//std::cin >> siz;
	std::string filename = argv[1];
	// Replace the path below with the path where you saved your file
	reader.read(filename, *cloud); // Remember to download the file first!
	//std::cout << "read.";
	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(50);
	sor.setStddevMulThresh(1.0);
	sor.filter(*cloud_filtered);
	//std::cout << "procd.";
	pcl::PCDWriter writer;
	filename.append("_filtered.pcd");
	writer.write(filename, *cloud_filtered, false);

	return (0);
}