#include <stdlib.h>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>



using namespace std;
using namespace pcl;


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
	cout << "\n\nUsage: " << progName << " <input.pcd> <output.pcd> [options] \n\n"
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

	vector<int> pcd_filenames = console::parse_file_extension_argument(argc, argv, "pcd");

	if(pcd_filenames.size() < 2)
	{
		printUsage(argv[0]);
		return EXIT_FAILURE;
	}

	const char* filepath    = argv[pcd_filenames[0]];
	const char* outfilepath = argv[pcd_filenames[1]];

	int knn = 10;
	console::parse(argc, argv, "-k", knn);


	typedef PointXYZRGB  PointOutT;




	PointCloud<PointXYZ>::Ptr points_ptr(new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>& points = *points_ptr;
	pcl::io::loadPCDFile<PointXYZ>(filepath, points);

	cout << "Read " << points.size() << " points." << endl;

	// Estimate normals
	cout << "Estimating normals..." << endl;
	PointCloud<Normal>::Ptr normals (new PointCloud<Normal>);
	computeNormals<PointXYZ>(points_ptr, normals, knn);


	Eigen::Vector3f lightSource(0,0,0);

	PointCloud<PointOutT>::Ptr outCloud_ptr(new PointCloud<PointOutT>(points.size(), 1));
	PointCloud<PointOutT>& outCloud = *outCloud_ptr;
	
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


	cout << "Saving processed points..." << endl;
	pcl::io::savePCDFile(outfilepath, outCloud, true);
	cout << "Points saved to \"" << outfilepath << "\"" << endl;


	return EXIT_SUCCESS;
}
