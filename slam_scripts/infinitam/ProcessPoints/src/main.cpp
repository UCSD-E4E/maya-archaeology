#include <stdlib.h>
#include <iostream>

#include "point_XYZRGBI.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>



using namespace std;
using namespace pcl;

struct float3
{
	float x,y,z;
};




	template<typename PointT>
void computeNormals(
		typename pcl::PointCloud<PointT>::Ptr cloud,
		typename pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,
		float radius = 0.03)
{
	// Create the normal estimation class, and pass the input dataset to it
	NormalEstimationOMP<PointT, pcl::Normal> ne;
	ne.setInputCloud (cloud);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	typename search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
	ne.setSearchMethod (tree);

	// Use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch (radius);

	// Compute the features
	ne.compute (*cloud_normals);
}


	template<typename PointT, typename PointOutT>
inline void averageColor(PointOutT& p_out, const PointT& p1, const PointT& p2)
{
	p_out.r = (int(p1.r) + int(p2.r)) / 2;
	p_out.g = (int(p1.g) + int(p2.g)) / 2;
	p_out.b = (int(p1.b) + int(p2.b)) / 2;
}

	template<>
inline void averageColor<PointXYZI, PointXYZRGB>(PointXYZRGB& p_out, const PointXYZI& p1, const PointXYZI& p2)
{}


	template<typename PointT, typename PointOutT>
void getCloudFromTSDF(
		typename pcl::PointCloud<PointT>::Ptr points_ptr,
		pcl::PointCloud<PointOutT>& outCloud,
		float3 cell_size,
		float resolution = 2.f)
{
	pcl::PointCloud<PointT>& points = *points_ptr;

	pcl::octree::OctreePointCloudSearch<PointT> octree (resolution);
	octree.setInputCloud (points_ptr);
	octree.addPointsFromInputCloud();
	std::vector<float> k_sqr_distances;


	// Progress bar
	int barSize = 50;
	cout << "\n|";
	for(int i = 0; i < barSize+1; i++)
	{ cout << "-"; }
	cout << "|\n|" << flush;

	int percent = points.size() / barSize;

	// Main loop for all points
#pragma omp parallel for
	for(size_t i = 0; i < points.size(); i++)
	{
		const float x = points[i].x;
		const float y = points[i].y;
		const float z = points[i].z;
		const float F = points[i].intensity;

		float3 V;
		V.x = (x + 0.5f) * cell_size.x;
		V.y = (y + 0.5f) * cell_size.y;
		V.z = (z + 0.5f) * cell_size.z;


		{
			//process dx
			PointT next_search;
			next_search.x = x + 1;
			next_search.y = y;
			next_search.z = z;
			std::vector <int> k_indices;

			octree.nearestKSearch(next_search, 1, k_indices, k_sqr_distances);

			PointT &next_pt = points[k_indices[0]];

			if (std::abs((x + 1.f) - next_pt.x) < 0.0001f)
			{
				const float Fn = next_pt.intensity;

				if ((F > 0 && Fn < 0) || (F < 0 && Fn > 0))
				{
					PointOutT p;
					p.y = V.y;
					p.z = V.z;

					float Vnx = V.x + cell_size.x;

					float d_inv = 1.f / (fabs(F) + fabs(Fn));
					p.x = (V.x * fabs(Fn) + Vnx * fabs(F)) * d_inv;

					averageColor(p, points[i], next_pt);

#pragma omp critical
					{
						outCloud.push_back(p);
					}
				}
			}
		}

		{
			//process dy
			PointT next_search;
			next_search.x = x;
			next_search.y = y + 1;
			next_search.z = z;
			std::vector <int> k_indices;

			octree.nearestKSearch(next_search, 1, k_indices, k_sqr_distances);

			PointT &next_pt = points[k_indices[0]];

			if (std::abs((y + 1.f) - next_pt.y) < 0.0001f)
			{
				const float Fn = next_pt.intensity;

				if ((F > 0 && Fn < 0) || (F < 0 && Fn > 0))
				{
					PointOutT p;
					p.x = V.x;
					p.z = V.z;

					float Vny = V.y + cell_size.y;

					float d_inv = 1.f / (fabs (F) + fabs (Fn));
					p.y = (V.y * fabs (Fn) + Vny * fabs (F)) * d_inv;

					averageColor(p, points[i], next_pt);

#pragma omp critical
					{
						outCloud.push_back(p);
					}
				}
			}
		}

		{
			//process dz
			PointT next_search;
			next_search.x = x;
			next_search.y = y;
			next_search.z = z + 1;
			std::vector <int> k_indices;

			octree.nearestKSearch(next_search, 1, k_indices, k_sqr_distances);

			PointT &next_pt = points[k_indices[0]];

			if (std::abs((z + 1.f) - next_pt.z) < 0.0001f)
			{
				const float Fn = next_pt.intensity;

				if ((F > 0 && Fn < 0) || (F < 0 && Fn > 0))
				{
					PointOutT p;
					p.x = V.x;
					p.y = V.y;

					float Vnz = V.z + cell_size.z;

					float d_inv = 1.f / (fabs (F) + fabs (Fn));
					p.z = (V.z * fabs (Fn) + Vnz * fabs (F)) * d_inv;

					averageColor(p, points[i], next_pt);

#pragma omp critical
					{
						outCloud.push_back(p);
					}
				}
			}
		}

		if(i % percent == 0)
		{
			cout << "*" << flush;
		}

	}// for all points

	cout << "|\n" << endl;
}



void printUsage(const char* progName)
{
	cout << "\n\nUsage: " << progName << " <input.pcd> <output.pcd> [options] \n\n"
		<< "Options:\n"
		<< "-------------------------------------------\n"
		<< "-v  --voxels     <num>             Num voxels on one side of the volume (default: 256)\n"
		<< "-s  --size       <meters>          Size of one side of the volume in meters (default: 3.0)\n"
		//<< "-nc --no-colors                    Load the points without colors and compute colors from normals and artificial light source\n"
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

	float volume_size = 3.f;
	int   num_voxels = 256;

	//bool hasColors = (console::find_argument(argc, argv, "--no-colors") < 0);
	console::parse(argc, argv, "-v",       num_voxels);
	console::parse(argc, argv, "--voxels", num_voxels);
	console::parse(argc, argv, "-s",       volume_size);
	console::parse(argc, argv, "--size",   volume_size);

	float cell_size_f = volume_size / num_voxels;

	float3 cell_size;
	cell_size.x = cell_size_f;
	cell_size.y = cell_size_f;
	cell_size.z = cell_size_f;

	cout << "Volume size = " << volume_size << " ; Num voxels = " << num_voxels << endl;
	cout << "Cell size = " << cell_size.x << endl;


	bool hasColors = false;

	// Read header of file
	pcl::PCLPointCloud2 cloud_header;
	pcl::PCDReader reader;
	reader.read (filepath, cloud_header);
	for(auto& f: cloud_header.fields)
	{
		if(f.name == "rgb"){ hasColors = true; }
	}

	cout << "Reading points in \"" << filepath << "\" " << (hasColors? "with": "without") << " colors." << endl;

	typedef PointXYZRGB  PointOutT;



	PointCloud<PointOutT>::Ptr outCloud_ptr(new PointCloud<PointOutT>());
	PointCloud<PointOutT>& outCloud = *outCloud_ptr;

	if(hasColors)
	{
		PointCloud<PointXYZRGBI>::Ptr points_ptr(new PointCloud<PointXYZRGBI>);
		PointCloud<PointXYZRGBI>& points = *points_ptr;
		pcl::io::loadPCDFile<PointXYZRGBI>(filepath, points);

		cout << "Read " << points.size() << " points." << endl;
		cout << "Processing points..." << endl;

		getCloudFromTSDF<PointXYZRGBI, PointOutT>(points_ptr, outCloud, cell_size);
	}
	else
	{
		PointCloud<PointXYZI>::Ptr points_ptr(new PointCloud<PointXYZI>);
		PointCloud<PointXYZI>& points = *points_ptr;
		pcl::io::loadPCDFile<PointXYZI>(filepath, points);

		cout << "Read " << points.size() << " points." << endl;
		cout << "Processing points..." << endl;

		getCloudFromTSDF<PointXYZI, PointOutT>(points_ptr, outCloud, cell_size);
	}




	if(!hasColors)
	{
		// Estimate normals
		cout << "Estimating normals..." << endl;
		PointCloud<Normal>::Ptr normals (new PointCloud<Normal>);
		computeNormals<PointOutT>(outCloud_ptr, normals, cell_size.x * 3);


		Eigen::Vector3f lightSource(0,0,0);

		// Set color based on light source
		cout << "Setting colors..." << endl;
#pragma omp parallel for
		for(unsigned int i = 0; i < outCloud.size(); i++)
		{
			Eigen::Vector3f v(outCloud[i].x, outCloud[i].y, outCloud[i].z);
			Eigen::Vector3f n((*normals)[i].normal_x, (*normals)[i].normal_y, (*normals)[i].normal_z);

			Eigen::Vector3f vec = (lightSource - v).normalized();

			float weight = abs(vec.dot(n));

			int br = (int)(205 * weight) + 50;
			br = max (0, min (255, br));

			outCloud[i].r = br;
			outCloud[i].g = br;
			outCloud[i].b = br;
		}
	}


	//Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	//transform.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX()));

	//pcl::transformPointCloud(outCloud, outCloud, transform);


	cout << "Saving processed points..." << endl;
	pcl::io::savePCDFile(outfilepath, outCloud, true);
	cout << "Points saved to \"" << outfilepath << "\"" << endl;


	return EXIT_SUCCESS;
}
