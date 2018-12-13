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
#include <pcl/filters/voxel_grid.h>

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


//----------------------------------------------------------
template<typename PointT>
void voxelGrid (
		typename pcl::PointCloud<PointT>::ConstPtr input,
		typename pcl::PointCloud<PointT>::Ptr output,
		float leaf_x, float leaf_y, float leaf_z)
//----------------------------------------------------------
{
	pcl::VoxelGrid<PointT> grid;
	grid.setInputCloud (input);
	grid.setLeafSize (leaf_x, leaf_y, leaf_z);
	grid.filter (*output);
}


//----------------------------------------------------------
void printUsage(const char* progName)
//----------------------------------------------------------
{
	cout << "\n\nUsage: " << progName << " <Traj_File.txt> <pointclouds.txt> [options] \n\n"
		<< "Options:\n"
		<< "-n                                 Enable normal computation (may use more memory)\n"
		<< "-s <factor>                        Apply scaling factor to trajectory transforms\n"
		<< "-v <size>                          Voxel size for downsample\n"
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

	float scaling_factor = 1.0f;
	pcl::console::parse_argument(argc, argv, "-s", scaling_factor);
	
	float voxel_size = 0.05f;
	pcl::console::parse_argument(argc, argv, "-v", voxel_size);


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


	Eigen::Matrix4f tscale = Eigen::Matrix4f::Identity();
	tscale (0,0) = tscale (0,0) * scaling_factor;
	tscale (1,1) = tscale (1,1) * scaling_factor;
	tscale (2,2) = tscale (2,2) * scaling_factor;

	while(traj_file >> traj_str)
	{
		float tx, ty, tz, qw, qx, qy, qz;
		traj_file >> tx >> ty >> tz >> qx >> qy >> qz >> qw;

		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		transform.translation() << tx, ty, tz;

		Eigen::Matrix3f mat3 = Eigen::Quaternionf(qw, qx, qy, qz).toRotationMatrix();
		transform.rotate (mat3);

		transform = Eigen::Affine3f(tscale * transform.matrix());

		//transform.scale(scaling_factor);

		transforms.push_back(transform);
	}


	// ---
	// Transform / Concatenate / Downsample
	// ---

	cout << "Transforming and concatenating clouds..." << endl;

	// Breaking up transforming and concatenating as large reconstructions run out of memory.
	// Default number of clouds to transform and concatenate at once is 400.
	int progress = 0;
	int j = 0;
	int num_of_clouds = 400;
	int num_of_outer_loops = paths.size() / num_of_clouds;
	if (paths.size()%num_of_clouds == 0)
	{
		num_of_outer_loops--;
	}


	pcl::PointCloud<pcl::PointNormal>::Ptr downsampled_cloud (new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_temp (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr downsampled_normals (new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr downsampled_with_normals (new pcl::PointCloud<pcl::PointNormal>);



	while (j <= num_of_outer_loops)
	{
		int loop_size = (j+1)*num_of_clouds;

		if (paths.size() <= loop_size)
		{
			loop_size = paths.size();
		}

		pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

#pragma omp parallel
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
			pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
			pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

#pragma omp for
			for (size_t i = j*num_of_clouds; i < loop_size; i++)
			{
				pcl::io::loadPLYFile (paths[i], *source_cloud);

				pcl::transformPointCloud (*source_cloud, *transformed_cloud, transforms[i]);
				//pcl::transformPointCloud (*source_cloud, *temp_cloud, transforms[i]);
				//pcl::transformPointCloud (*temp_cloud, *transformed_cloud, tscale);


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
		// Downsample and compute normals on downsampled cloud
		// ---
		voxelGrid<pcl::PointXYZ>(result_cloud, downsampled_temp, voxel_size, voxel_size, voxel_size);
		computeNormals<pcl::PointXYZ>(downsampled_temp, downsampled_normals, 10);
		pcl::concatenateFields(*downsampled_temp, *downsampled_normals, *downsampled_with_normals);

		*downsampled_cloud += *downsampled_with_normals;




		// ---
		// Save output
		// ---

		cout << "Saving output point cloud..." << endl;


		string output_filename = "orbslam_cloud_" + std::to_string(j) + ".ply";

		if (compute_normals)
		{
			pcl::io::savePLYFile(output_filename, *cloud_with_normals, true);
		}
		else
		{
			pcl::io::savePLYFile(output_filename, *result_cloud, true);
		}

		


		j++;
	}



	pcl::io::savePLYFile("orbslam_cloud_downsampled.ply", *downsampled_cloud, true);


	// Save scaled trajectory in kitti format
	ofstream slam_traj_out("orbslam_traj.txt");

	for (auto& t :transforms)
	{
		slam_traj_out
			<< t.matrix().row(0)[0] << " " << t.matrix().row(0)[1] << " " << t.matrix().row(0)[2] << " " << t.matrix().row(0)[3] << " "
			<< t.matrix().row(1)[0] << " " << t.matrix().row(1)[1] << " " << t.matrix().row(1)[2] << " " << t.matrix().row(1)[3] << " "
			<< t.matrix().row(2)[0] << " " << t.matrix().row(2)[1] << " " << t.matrix().row(2)[2] << " " << t.matrix().row(2)[3]
			<< "\n";
	}


	return 0;
}
