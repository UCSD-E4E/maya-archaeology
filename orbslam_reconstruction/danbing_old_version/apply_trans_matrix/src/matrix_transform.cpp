#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

// This is the main function
int
main (int argc, char** argv)
{
	// open key trajectory file (1497279653.261259535.txt)
  // open selected frame file (selected_frame.txt)
	if(argc < 2){
		std::cerr << "Usage: " << argv[0] << " Key trajectory file (.txt)" << std::endl;
		return 1;
	}

	std::string temp = std::string(argv[1]);
	std::ifstream infile_1(temp);
	std::cout << temp << std::endl;
	std::ifstream infile_2("selected_frame.txt");

  // Read one line from both key trajectory file and the selected fram file to get the transformation info and which point cloud to choose
	std::string line_1;
  std::string line_2;
	std::string point_cloud_path;

	//pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");
	pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

	int i = 0;
	while(std::getline(infile_1, line_1) && std::getline(infile_2, line_2) && i < 600) {
		line_2.insert(line_2.begin(), 6 - line_2.size(), '0');
		point_cloud_path = "point_clouds/" + line_2 + ".ply";
		std::cout << point_cloud_path << std::endl;
		
  	pcl::io::loadPLYFile (point_cloud_path, *source_cloud);

    std::vector<std::string>   parsed;
    std::stringstream  data(line_1);
    std::string line;
    while(std::getline(data,line,' '))
    {
        parsed.push_back(line);
    }

		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	 	transform.translation() << std::stof(parsed[1]), std::stof(parsed[2]), std::stof(parsed[3]);

		Eigen::Matrix3f mat3 = Eigen::Quaternionf(std::stof(parsed[7]), std::stof(parsed[4]), std::stof(parsed[5]), std::stof(parsed[6])).toRotationMatrix();
  	transform.rotate (mat3);
		
  	pcl::transformPointCloud (*source_cloud, *transformed_cloud, transform);
		*result_cloud  = *result_cloud + *transformed_cloud;
		//viewer.addPointCloud (transformed_cloud, line_2);
		i++;
	}

  pcl::io::savePLYFile("good_result.ply", *result_cloud);
  //viewer.addCoordinateSystem (1.0, "cloud", 0);
  //viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
  //viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_first_cloud");
  //viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud_578");
  //viewer.setPosition(800, 400); // Setting visualiser window position

  //while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    //viewer.spinOnce ();
  //}

  return 0;
}
