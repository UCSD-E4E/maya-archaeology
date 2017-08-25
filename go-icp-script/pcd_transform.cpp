#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/common/common.h>
#include <cmath>
#include <pcl/common/transforms.h>

int main (int argc, char** argv)
{
	Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
	int i, j;

	std::ifstream mtrx("mat.txt");
	if (!mtrx) {
		std::cout << "Cannot open matrix file.\n";
		return (-1);
	}
	for (i = 0; i < 4; i++) {
		for (j = 0; j < 4; j++) {
			mtrx >> matrix(i, j);
		}
	}
	mtrx.close();

	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			std::cout << matrix(i, j) << " ";
		}
		std::cout << std::endl;
	}

	long double centroid_lidar[3], centroid_slam[3];
	long lidar_size, slam_size;
	float maxDim;
	std::ifstream centr("centroid.txt");
	if(!centr) {
		std::cout << "Cannot open centroid file.";
		return (-1);
	}
	for (i = 0; i < 3; i++) centr >> centroid_lidar[i];
	centr >> lidar_size;
	for (i = 0; i < 3; i++) centr >> centroid_slam[i];
	centr >> slam_size;
	centr >> maxDim;
	centr.close();

//	for(i = 0; i < 3; i++) std::cout << centroid_lidar[i] << " " << centroid_slam[i] << "\n";
//	std::cout << lidar_size << " " << slam_size << " " << maxDim << "\n";

//	Eigen::Matrix4f scaleUp = Eigen::Matrix4f::Identity();
//	scaleUp (0,0) = maxDim;
//	scaleUp (1,1) = maxDim;
//	scaleUp (2,2) = maxDim;
//
//	Eigen::Matrix4f scaleDown = Eigen::Matrix4f::Identity();
//	scaleDown (0,0) = 1./maxDim;
//	scaleDown (1,1) = 1./maxDim;
//	scaleDown (2,2) = 1./maxDim;
//
//	Eigen::Matrix4f centroid_s = Eigen::Matrix4f::Identity();
//	Eigen::Matrix4f centroid_l = Eigen::Matrix4f::Identity();
//	for(i = 0; i < 3; i++){
//		cent	roid_s (i, 3) = centroid_lidar[i];
//		cent	roid_l (i, 3) = 0-centroid_slam[i];
//	}     	
//        	
//	Eigen::Matrix4f fMatrix = centroid_l * scaleUp * matrix * scaleDown * centroid_s;
//	for(i 	= 0; i < 4; i++){
//		for(	j = 0; j < 4; j++){
//			st	d::cout << fMatrix(i,j) << " ";
//		}   	
//		std:  :cout << "\n";
//	}     	
          	
	pcl::PointCloud<pcl::PointXYZ>::Ptr slam_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ> ("sc2.pcd", *slam_cloud) == -1) {
		PCL_ERROR ("Couldn't read slam model file \n");
		return (-1);
	}       
	        	
	std::cout << "Loaded " << slam_cloud->width * slam_cloud->height
          	  << " data points from sc2.pcd (slam)" << std::endl;
	        	
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::transformPointCloud (*slam_cloud, *transformed_cloud, matrix);
          	
	std::cout << "Transformed and saving sc2-t.pcd\n";
	pcl::io::savePCDFileBinary("sc2-t.pcd", *transformed_cloud);

	return (0);
}
