#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/common/common.h>
#include <cmath>
#include <pcl/common/transforms.h>

int main (int argc, char** argv)
{
	if (argc < 3){
		std::cout << "not enough arguments, need lidar, slam.\n";
		return (-1);
	}
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud1) == -1) {
    PCL_ERROR ("Couldn't read first file \n");
    return (-1);
  }
	
	std::cout << "Loaded " << cloud1->points.size()
            << " data points from " << argv[1] << std::endl;
 	
	if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[2], *cloud2) == -1) {
    PCL_ERROR ("Couldn't read second file \n");
    return (-1);
  }

	std::cout << "Loaded " << cloud2->points.size()
            << " data points from " << argv[2] << std::endl;

	pcl::PointXYZ min, max, min2, max2;
	float x, y, z, x2, y2, z2;
	
//	pcl::getMinMax3D(*cloud1, min, max);
//	x = std::abs(min.x) + std::abs(max.x);
//	y = std::abs(min.y) + std::abs(max.y);
//	z = std::abs(min.z) + std::abs(max.z);
//	std::cout << "original box boundaries x: " << x << " y: " << y << " z: " << z << std::endl;

	pcl::PointXYZ centroid;
	
	pcl::computeCentroid(*cloud1, centroid);
	Eigen::Matrix<float,4,1> center = Eigen::Matrix<float,4,1>();
	center (0,0) = centroid.x;
	center (1,0) = centroid.y;
	center (2,0) = centroid.z;

	std::ofstream cFile;
	cFile.open ("centroid.txt");
	cFile << center(0,0) << "\n" << center(1,0) << "\n" << center(2,0) << "\n" << cloud1->points.size() << "\n";
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr centered_cloud1 (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::demeanPointCloud(*cloud1, center, *centered_cloud1);

	pcl::computeCentroid(*cloud2, centroid);
	center (0,0) = centroid.x;
	center (1,0) = centroid.y;
	center (2,0) = centroid.z;

	cFile << center(0,0) << "\n" << center(1,0) << "\n" << center(2,0) << "\n" << cloud2->points.size() << "\n";
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr centered_cloud2 (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::demeanPointCloud(*cloud2, center, *centered_cloud2);
	
	pcl::getMinMax3D(*centered_cloud1, min, max);
	x = std::abs(min.x) + std::abs(max.x);
	y = std::abs(min.y) + std::abs(max.y);
	z = std::abs(min.z) + std::abs(max.z);
	std::cout << "centered box(1) boundaries x: " << x << " y: " << y << " z: " << z << std::endl;

	pcl::getMinMax3D(*centered_cloud2, min2, max2);
	x2 = std::abs(min2.x) + std::abs(max2.x);
	y2 = std::abs(min2.y) + std::abs(max2.y);
	z2 = std::abs(min2.z) + std::abs(max2.z);
	std::cout << "centered box(2) boundaries x: " << x2 << " y: " << y2 << " z: " << z2 << std::endl;
	 
	float maxDim1 = std::max(std::max(x, y), z);
	float maxDim2 = std::max(std::max(x2, y2), z2);
	float maxDim = std::max(maxDim1, maxDim2);
	cFile << maxDim;
	cFile.close();
	
	float scaleF = 1./maxDim;
	std::cout << "largest dimension: " << maxDim << " and scaling by: " << scaleF << std::endl;

	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
	transform_1 (0,0) = scaleF;
	transform_1 (1,1) = scaleF;
	transform_1 (2,2) = scaleF;
//	std::cout << transform_1 << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud1 (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::transformPointCloud (*centered_cloud1, *transformed_cloud1, transform_1);

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud2 (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::transformPointCloud (*centered_cloud2, *transformed_cloud2, transform_1);

	pcl::getMinMax3D(*transformed_cloud1, min, max);
	x = std::abs(min.x) + std::abs(max.x);
	y = std::abs(min.y) + std::abs(max.y);
	z = std::abs(min.z) + std::abs(max.z);
	std::cout << "scaled box(1) boundaries x: " << x << " y: " << y << " z: " << z << std::endl;
	
	pcl::getMinMax3D(*transformed_cloud2, min2, max2);
	x2 = std::abs(min2.x) + std::abs(max2.x);
	y2 = std::abs(min2.y) + std::abs(max2.y);
	z2 = std::abs(min2.z) + std::abs(max2.z);
	std::cout << "scaled box(2) boundaries x: " << x2 << " y: " << y2 << " z: " << z2 << std::endl;

	pcl::io::savePCDFileASCII("sc1.pcd", *transformed_cloud1);
	pcl::io::savePCDFileASCII("sc2.pcd", *transformed_cloud2);
	std::cerr << "Saved " << transformed_cloud1->points.size() << " data points to sc1.pcd." << std::endl;
	std::cerr << "Saved " << transformed_cloud2->points.size() << " data points to sc2.pcd." << std::endl;

//	for (size_t i = 0; i < cloud1->points.size (); ++i)
//    std::cout << "    " << cloud1->points[i].x
//              << " "    << cloud1->points[i].y
//              << " "    << cloud1->points[i].z << std::endl;

  return (0);
}
