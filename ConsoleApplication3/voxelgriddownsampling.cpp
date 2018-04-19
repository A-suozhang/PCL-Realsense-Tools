#include <iostream>
#include <string>
#define _ITERATOR_DEBUG_LEVEL 0
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

int
main(int argc, char** argv)
{
	pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
	pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());

	// Fill in the cloud data
	pcl::PCDReader reader;

	if (argc != 2) {
		std::cerr << "Invalid arguments";
		return 1;
	}
	float siz;
	std::cin >> siz;
	std::string filename = argv[1];
	// Replace the path below with the path where you saved your file
	reader.read(filename, *cloud); // Remember to download the file first!

	std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
		<< " data points (" << pcl::getFieldsList(*cloud) << ")." << std::endl;

	// Create the filtering object
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(siz, siz, siz);
	sor.filter(*cloud_filtered);

	std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
		<< " data points (" << pcl::getFieldsList(*cloud_filtered) << ").";

	pcl::PCDWriter writer;
	filename.append("_downsampled.pcd");
	writer.write(filename, *cloud_filtered,
		Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false);

	return (0);
}