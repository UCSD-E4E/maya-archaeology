#include <stdlib.h>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/common/transforms.h>
#include <pcl/PCLPointCloud2.h>



using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;


bool loadCloud (const std::string &filename, pcl::PCLPointCloud2 &cloud, bool isPly=false)
{
	TicToc tt;
	print_highlight ("Loading "); print_value ("%s ", filename.c_str ());

	tt.tic ();
	if (isPly)
	{
		if (loadPLYFile (filename, cloud) < 0)
			return (false);
	}
	else
	{
		if (loadPCDFile (filename, cloud) < 0)
			return (false);
	}
	print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
	print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList (cloud).c_str ());

	return (true);
}


	template<typename PointT>
void computeNormals(
		typename pcl::PointCloud<PointT>::Ptr cloud,
		typename pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,
		float k = 10)
{
	// Create the normal estimation class, and pass the input dataset to it
	NormalEstimationOMP<PointT, pcl::Normal> ne;
	ne.setInputCloud (cloud);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	typename search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
	ne.setSearchMethod (tree);

	// Use all neighbors in a sphere of radius 3cm
	//ne.setRadiusSearch (radius);
	// Or use k nearest neighbors
	ne.setKSearch(k);

	// Compute the features
	ne.compute (*cloud_normals);
}




void printUsage(const char* progName)
{
	cout << "\n\nUsage: " << progName << " <input.pcd/ply> <output.pcd/ply> [options] \n\n"
		<< "Options:\n"
		<< "-k                                 Number of nearest neighbors for normal estimation (default: 10)\n"
		<< "-------------------------------------------\n"
		<< "-h  --help                         This help\n"
		<< "\n\n";
}

int main (int argc, char** argv)
{
	if(console::find_argument(argc, argv, "-h") >= 0
			|| console::find_argument(argc, argv, "--help") >= 0)
	{
		printUsage(argv[0]);
		return EXIT_SUCCESS;
	}


	// Parse the command line arguments for .pcd files
	std::vector<int> pcd_file_indices;
	pcd_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
	std::vector<int> ply_file_indices;
	ply_file_indices = parse_file_extension_argument (argc, argv, ".ply");

	std::string input_filename;
	std::string output_filename;

	bool input_is_ply = true;
	bool output_is_ply = true;

	if (pcd_file_indices.size () + ply_file_indices.size () == 2)
	{
		int input_file_idx = 0;
		int output_file_idx = 0;
		input_is_ply = false;
		output_is_ply = false;
		if (pcd_file_indices.size () == 2)
		{
			input_file_idx = pcd_file_indices[0];
			output_file_idx = pcd_file_indices[1];
		}
		else if (ply_file_indices.size () == 2)
		{
			input_file_idx = ply_file_indices[0];
			output_file_idx = ply_file_indices[1];
			input_is_ply = true;
			output_is_ply = true;
		}
		else if (pcd_file_indices[0] < ply_file_indices[0])
		{
			input_file_idx = pcd_file_indices[0];
			output_file_idx = ply_file_indices[0];
			output_is_ply = true;
		}
		else
		{
			output_file_idx = pcd_file_indices[0];
			input_file_idx = ply_file_indices[0];
			input_is_ply = true;
		}

		input_filename = argv[input_file_idx];
		output_filename = argv[output_file_idx];

	}
	else
	{
		printUsage(argv[0]);
		return EXIT_FAILURE;
	}




	// Parse options from the command line
	int knn = 10;
	console::parse(argc, argv, "-k", knn);



	// Load the input file
	pcl::PCLPointCloud2::Ptr blob (new pcl::PCLPointCloud2);
	if (!loadCloud (input_filename.c_str (), *blob, input_is_ply)) 
	{ return EXIT_FAILURE; }

	bool has_normals = false;
	for (int i = 0; i < blob->fields.size(); i++)
	{
		if (blob->fields[i].name == "normal_x")
		{ has_normals = true; }
	}


	PointCloud<PointXYZ>::Ptr points_ptr(new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>& points = *points_ptr;
	PointCloud<Normal>::Ptr normals (new PointCloud<Normal>);

	fromPCLPointCloud2(*blob, points);

	if (!has_normals)
	{
		// Estimate normals
		cout << "Estimating normals..." << endl;

		computeNormals<PointXYZ>(points_ptr, normals, knn);
	}
	else
	{
		fromPCLPointCloud2(*blob, *normals);
	}


	Eigen::Vector3f lightSource(0,0,0);

	PointCloud<PointXYZRGBNormal>::Ptr outCloud_ptr(new PointCloud<PointXYZRGBNormal>(points.size(), 1));
	PointCloud<PointXYZRGBNormal>& outCloud = *outCloud_ptr;

	copyPointCloud(points, outCloud);

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
		outCloud[i].normal_x = n.x();
		outCloud[i].normal_y = n.y();
		outCloud[i].normal_z = n.z();
		outCloud[i].r = br;
		outCloud[i].g = br;
		outCloud[i].b = br;
	}


	// Save point cloud
	cout << "Saving processed points..." << endl;

	if (output_is_ply)
	{ pcl::io::savePLYFile(output_filename.c_str(), outCloud, true); }
	else { pcl::io::savePCDFile(output_filename.c_str(), outCloud, true); }

	cout << "Points saved to \"" << output_filename << "\"" << endl;


	return EXIT_SUCCESS;
}
