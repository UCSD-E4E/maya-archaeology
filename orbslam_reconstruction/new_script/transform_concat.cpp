#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/octree/octree.h>
#include <pcl/features/normal_3d_omp.h>

#include <Eigen/StdVector>

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Affine3f)


using namespace std;


//----------------------------------------------------------
template<typename PointT>
void computeNormals(
		typename pcl::PointCloud<PointT>::Ptr cloud,
		typename pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,
		float k = 10)
//----------------------------------------------------------
{
	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
	ne.setInputCloud (cloud);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
	ne.setSearchMethod (tree);

	// Use all neighbors in a sphere of radius <n> cm
	//ne.setRadiusSearch (radius);
	// Or use k nearest neighbors
	ne.setKSearch(k);

	// Compute the features
	ne.compute (*cloud_normals);
}


void printUsage(const char* progName)
{
	cout << "\n\nUsage: " << progName << " <Traj_File.txt> <pointclouds.txt> [options] \n\n"
		<< "Options:\n"
		<< "-n                                 Enable normal computation (may use more memory)\n"
		<< "-------------------------------------------\n"
		<< "-h  --help                         This help\n"
		<< "\n\n";
}


//----------------------------------------------------------
int main (int argc, char** argv)
//----------------------------------------------------------
{
	if(pcl::console::find_argument(argc, argv, "-h") >= 0
			|| pcl::console::find_argument(argc, argv, "--help") >= 0)
	{
		printUsage(argv[0]);
		return EXIT_SUCCESS;
	}
	
	vector<int> txt_filenames = pcl::console::parse_file_extension_argument(argc, argv, "txt");
	
	if(txt_filenames.size() < 2)
	{
		printUsage(argv[0]);
		return EXIT_FAILURE;
	}

	bool compute_normals = pcl::console::find_argument(argc, argv, "-n") >= 0;
	
	string traj_filename = string(argv[txt_filenames[0]]);
	string pc_filename   = string(argv[txt_filenames[1]]);


	// ---
	// Read input files
	// ---

	ifstream traj_file(traj_filename);
	ifstream pc_file(pc_filename);

	string traj_str;
	string pc_str;

	vector<string> paths;
	vector<Eigen::Affine3f> transforms;

	while(pc_file >> pc_str)
	{
		paths.push_back(pc_str);
	}

	while(traj_file >> traj_str)
	{
		float tx, ty, tz, qw, qx, qy, qz;
		traj_file >> tx >> ty >> tz >> qx >> qy >> qz >> qw;

		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		transform.translation() << tx, ty, tz;

		Eigen::Matrix3f mat3 = Eigen::Quaternionf(qw, qx, qy, qz).toRotationMatrix();
		transform.rotate (mat3);

		transforms.push_back(transform);
	}


	// ---
	// Transform / Concatenate
	// ---

	cout << "Transforming and concatenating clouds..." << endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

	int progress = 0;

#pragma omp parallel
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
		pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

#pragma omp for
		for (size_t i = 0; i < paths.size(); i++)
		{
			pcl::io::loadPLYFile (paths[i], *source_cloud);

			pcl::transformPointCloud (*source_cloud, *transformed_cloud, transforms[i]);

#pragma omp critical
			{
				*result_cloud += *transformed_cloud;

				//viewer.addPointCloud (transformed_cloud, line_2);

				progress++;
				cout << "\33[2K\r" << 100*progress/(float)paths.size() << " %" << flush;
			}
		}
	}
	cout << endl;


	// ---
  // Compute normals
	// ---

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
	
	if (compute_normals)
	{
		cout << "Computing normals..." << endl;

		pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
		computeNormals<pcl::PointXYZ>(result_cloud, normals, 10);
		
		pcl::concatenateFields(*result_cloud, *normals, *cloud_with_normals);
		pcl::PointCloud<pcl::PointXYZ>().swap(*result_cloud);
	}


	// ---
	// Save output
	// ---
	
	cout << "Saving output point cloud..." << endl;

	string output_filename = "orbslam_cloud.ply";

	if (compute_normals)
	{
		pcl::io::savePLYFile(output_filename, *cloud_with_normals);
	}
	else
	{
		pcl::io::savePLYFile(output_filename, *result_cloud);
	}


	/*
	// Set color based on light source
	cout << "Setting colors..." << endl;
#pragma omp parallel for
	for(unsigned int i = 0; i < outCloud.size(); i++)
	{
		Eigen::Vector3f v(points[i].x, points[i].y, points[i].z);
		Eigen::Vector3f n((*normals)[i].normal_x, (*normals)[i].normal_y, (*normals)[i].normal_z);

		Eigen::Vector3f vec = (lightSource - v).normalized();

		float weight = abs(vec.dot(n));

		int br = (int)(205 * weight) + 50;
		br = max (0, min (255, br));

		outCloud[i].x = v.x();
		outCloud[i].y = v.y();
		outCloud[i].z = v.z();
		outCloud[i].r = br;
		outCloud[i].g = br;
		outCloud[i].b = br;
	}
	*/





	return 0;
}
