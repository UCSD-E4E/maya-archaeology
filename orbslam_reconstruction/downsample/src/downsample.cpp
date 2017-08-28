#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ> ());

  pcl::io::loadPLYFile("good_result.ply", *cloud);

  // Create the filtering object
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.003785f, 0.003785f, 0.003785f); // the larger these numbers are, the less points the filtered cloud has
  sor.filter (*cloud_filtered);

  pcl::io::savePLYFile("downsampled_1.ply", *cloud_filtered);

  return (0);
}


