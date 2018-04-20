#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>   //最小二乘平滑处理类定义

int
main(int argc, char** argv)
{
	// Load input file into a PointCloud<T> with an appropriate type
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	// Load bun0.pcd -- should be available with the PCL archive in test 
	pcl::io::loadPCDFile("RealsenseUT-downsampled.pcd", *cloud);

	// Create a KD-Tree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

	// Output has the PointNormal type in order to store the normals calculated by MLS
	pcl::PointCloud<pcl::PointNormal> mls_points;

	// Init object (second point type is for the normals, even if unused)
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
	// 设置在最小二乘计算中需要进行法线估计,不需要可跳过
	mls.setComputeNormals(true);

	// Set parameters
	mls.setInputCloud(cloud);
	mls.setPolynomialFit(false);  //多项式拟合提高精度,可false 加快速度,或选择其他来控制平滑过程
	mls.setSearchMethod(tree);
	mls.setSearchRadius(0.3);

	// Reconstruct
	mls.process(mls_points);

	// Save output
	pcl::io::savePCDFile("Realsense-mls2.pcd", mls_points);
}