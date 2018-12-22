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
#include <pcl/registration/icp.h>
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/normal_3d_omp.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <chrono>
#include <thread>
#include <mutex>

#include <Eigen/StdVector>

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Affine3f)


using namespace std;
using namespace pcl;


const int SIZE_X = 640;
const int SIZE_Y = 480;


boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
pcl::PointCloud<pcl::PointNormal>::Ptr slice_cloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr slam_cloud;
pcl::PointCloud<pcl::PointNormal>::Ptr aligned_slam_cloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_traj_cloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr trajectory_cloud;

mutex pointcloud_mutex;

PointXYZ depth_map[SIZE_X][SIZE_Y];
int depth_map_idx[SIZE_X][SIZE_Y];


//----------------------------------------------------------
template<typename PointInT, typename PointOutT>
void computeNormals(
		typename pcl::PointCloud<PointInT>::Ptr cloud,
		typename pcl::PointCloud<PointOutT>::Ptr cloud_normals,
		float k = 10)
//----------------------------------------------------------
{
	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimationOMP<PointInT, PointOutT> ne;
	ne.setInputCloud (cloud);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	typename pcl::search::KdTree<PointInT>::Ptr tree (new pcl::search::KdTree<PointInT> ());
	ne.setSearchMethod (tree);

	// Use all neighbors in a sphere of radius <n> cm
	//ne.setRadiusSearch (radius);
	// Or use k nearest neighbors
	ne.setKSearch(k);

	// Compute the features
	ne.compute (*cloud_normals);
}


//----------------------------------------------------------
void clearDepthMap()
//----------------------------------------------------------
{
	for (int i = 0; i < SIZE_X; i++)
	{
		for (int j = 0; j < SIZE_Y; j++)
		{
			depth_map[i][j] = PointXYZ();
			depth_map[i][j].z = 999999999;
			depth_map_idx[i][j] = -1;
		}
	}
}

//----------------------------------------------------------
template<typename PointT>
void voxelGrid (
		typename pcl::PointCloud<PointT>::ConstPtr input,
		typename pcl::PointCloud<PointT>::Ptr output,
		float leaf_x, float leaf_y, float leaf_z)
//----------------------------------------------------------
{
  VoxelGrid<PointT> grid;
  grid.setInputCloud (input);
  grid.setLeafSize (leaf_x, leaf_y, leaf_z);
  grid.filter (*output);
}

//----------------------------------------------------------
void showViewer()
//----------------------------------------------------------
{
	viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_color(trajectory_cloud, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(slam_cloud, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> single_color2(aligned_slam_cloud, 0, 0, 255);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color3(aligned_traj_cloud, 0, 255, 255);
	viewer->addPointCloud<PointNormal> (slice_cloud, "Cloud 1");
	viewer->addPointCloud<PointXYZ> (trajectory_cloud, red_color, "Cloud 2");
	//viewer->addPointCloud<PointXYZ> (slam_cloud, single_color, "slam_cloud");
	viewer->addPointCloud<PointNormal> (aligned_slam_cloud, single_color2, "aligned_cloud");
	viewer->addPointCloud<PointXYZ> (aligned_traj_cloud, single_color3, "aligned_traj_cloud");
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();

	double psize = 1;
	while (!viewer->wasStopped ())
	{
		viewer->getPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, psize, "Cloud 1");
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
		lock_guard<mutex> guard(pointcloud_mutex);
		viewer->removePointCloud("Cloud 1");
		viewer->addPointCloud<PointNormal> (slice_cloud, "Cloud 1");
		//viewer->removePointCloud("slam_cloud");
		//viewer->addPointCloud<PointXYZ> (slam_cloud, single_color, "slam_cloud");
		viewer->removePointCloud("aligned_cloud");
		viewer->addPointCloud<PointNormal> (aligned_slam_cloud, single_color2, "aligned_cloud");
		viewer->removePointCloud("aligned_traj_cloud");
		viewer->addPointCloud<PointXYZ> (aligned_traj_cloud, single_color3, "aligned_traj_cloud");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, psize, "Cloud 1");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, psize, "Cloud 2");
		//viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, psize, "slam_cloud");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, psize, "aligned_cloud");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, psize, "aligned_traj_cloud");
		viewer->spinOnce (100);
	}
}


//----------------------------------------------------------
void readTUMTrajectory(string& path, vector<Eigen::Affine3f>& transforms, Eigen::Affine3f& global_transform)
//----------------------------------------------------------
{
	ifstream traj_file(path);
	
	string traj_str;

	while(traj_file >> traj_str)
	{
		float tx, ty, tz, qw, qx, qy, qz;
		traj_file >> tx >> ty >> tz >> qx >> qy >> qz >> qw;

		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		transform.translation() << tx, ty, tz;

		Eigen::Matrix3f mat3 = Eigen::Quaternionf(qw, qx, qy, qz).toRotationMatrix();
		transform.rotate (mat3);

		transform = global_transform * transform;

		transforms.push_back(transform);
	}
}


//----------------------------------------------------------
void readKittiTrajectory(string& path, vector<Eigen::Affine3f>& transforms, Eigen::Affine3f& global_transform)
//----------------------------------------------------------
{
	ifstream traj_file(path);

	float values[12];

	while(traj_file >> values[0])
	{
		for (int i = 1; i < 12; i++)
		{
			traj_file >> values[i];
		}

		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		transform.matrix() << 
			values[0], values[1], values[2], values[3],
			values[4], values[5], values[6], values[7],
			values[8], values[9], values[10], values[11],
			0, 0, 0, 1;

		transform = global_transform * transform;

		transforms.push_back(transform);
	}
}


//----------------------------------------------------------
void printUsage(const char* progName)
//----------------------------------------------------------
{
	cout << "\n\nUsage: " << progName << " <Traj_File.txt> <tranform.txt> <pointclouds.txt> <lidar.ply> [options] \n\n"
		<< "Options:\n"
		<< "-------------------------------------------\n"
		<< "-h  --help                         This help\n"
		<< "\n\n";
}


//----------------------------------------------------------
void saveTrajectories(vector<Eigen::Affine3f>& transforms, vector<Eigen::Affine3f>& registered_transforms, vector<Eigen::Affine3f>& transforms_aligned_to_registered)
//----------------------------------------------------------
{
	ofstream traj_out("gt_traj.txt");

	for (auto& t :registered_transforms)
	{
		traj_out
			<< t.matrix().row(0)[0] << " " << t.matrix().row(0)[1] << " " << t.matrix().row(0)[2] << " " << t.matrix().row(0)[3] << " "
			<< t.matrix().row(1)[0] << " " << t.matrix().row(1)[1] << " " << t.matrix().row(1)[2] << " " << t.matrix().row(1)[3] << " "
			<< t.matrix().row(2)[0] << " " << t.matrix().row(2)[1] << " " << t.matrix().row(2)[2] << " " << t.matrix().row(2)[3]
			<< "\n";
	}
	
	ofstream slam_traj_out("slam_traj_not_aligned.txt");

	for (auto& t :transforms)
	{
		slam_traj_out
			<< t.matrix().row(0)[0] << " " << t.matrix().row(0)[1] << " " << t.matrix().row(0)[2] << " " << t.matrix().row(0)[3] << " "
			<< t.matrix().row(1)[0] << " " << t.matrix().row(1)[1] << " " << t.matrix().row(1)[2] << " " << t.matrix().row(1)[3] << " "
			<< t.matrix().row(2)[0] << " " << t.matrix().row(2)[1] << " " << t.matrix().row(2)[2] << " " << t.matrix().row(2)[3]
			<< "\n";
	}
	
	ofstream slam_traj_out2("slam_traj.txt");

	for (auto& t :transforms_aligned_to_registered)
	{
		slam_traj_out2
			<< t.matrix().row(0)[0] << " " << t.matrix().row(0)[1] << " " << t.matrix().row(0)[2] << " " << t.matrix().row(0)[3] << " "
			<< t.matrix().row(1)[0] << " " << t.matrix().row(1)[1] << " " << t.matrix().row(1)[2] << " " << t.matrix().row(1)[3] << " "
			<< t.matrix().row(2)[0] << " " << t.matrix().row(2)[1] << " " << t.matrix().row(2)[2] << " " << t.matrix().row(2)[3]
			<< "\n";
	}
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
	vector<int> ply_filenames = pcl::console::parse_file_extension_argument(argc, argv, "ply");

	if(txt_filenames.size() < 3 || ply_filenames.size() < 1)
	{
		printUsage(argv[0]);
		return EXIT_FAILURE;
	}

	bool display = pcl::console::find_argument(argc, argv, "-d") >= 0;
	bool tum_format = pcl::console::find_argument(argc, argv, "-tum") >= 0;

	string traj_filename = string(argv[txt_filenames[0]]); // SLAM trajectory (KITTI or TUM format (with -tum option))
	string transform_filename = string(argv[txt_filenames[1]]); // Global transformation matrix from SLAM point cloud to reference point cloud
	string pc_filename   = string(argv[txt_filenames[2]]); // Filenames of processed SLAM sensor data
	string ply_filename   = string(argv[ply_filenames[0]]); // Filename of reference point cloud (in PLY format)


	// ---
	// Read input files
	// ---

	ifstream trans_file(transform_filename);
	ifstream pc_file(pc_filename);

	Eigen::Affine3f global_transform;
	float value;
	for (int i = 0; i < 16; i++)
	{
		trans_file >> value;
		global_transform(i/4,i%4) = value;
	}



	string pc_str;

	vector<string> paths;
	vector<Eigen::Affine3f> transforms;

	while(pc_file >> pc_str)
	{
		paths.push_back(pc_str);
	}

	
	if(tum_format) { readTUMTrajectory(traj_filename, transforms, global_transform); }
	else { readKittiTrajectory(traj_filename, transforms, global_transform); }


	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::PointCloud<pcl::Normal>::Ptr source_normals (new pcl::PointCloud<pcl::Normal> ());
	pcl::PointCloud<pcl::PointNormal>::Ptr source_with_normals (new pcl::PointCloud<pcl::PointNormal> ());
	pcl::io::loadPLYFile<pcl::PointXYZ> (ply_filename, *source_cloud);

	computeNormals<PointXYZ, Normal>(source_cloud, source_normals);
	concatenateFields(*source_cloud, *source_normals, *source_with_normals);

	trajectory_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ> ());
	for (auto& t : transforms)
	{
		PointXYZ p;
		p.x = t.translation()[0];
		p.y = t.translation()[1];
		p.z = t.translation()[2];
		trajectory_cloud->push_back(p);
	}




	slice_cloud = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal> ());
	slam_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ> ());
	aligned_slam_cloud = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal> ());
	aligned_traj_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ> ());
	aligned_traj_cloud->resize(transforms.size());

	// Parameters
	float fx, fy, cx, cy;
	fx = 525.0;
	fy = 525.0;
	cx = 319.5;
	cy = 239.5;
	float max_dist = 3.0f;

	// Parameters actually used
	float margin_dist = 0.1f;
	float voxel_size = 0.01f;
	

	// Start viewer thread
	thread show_thread;
	
	if (display)
	{
		show_thread = thread(showViewer);
	}


	// -------
	// Register each SLAM cloud with the reference model
	// -------

	int num_parallel = 1;
	int save_temp = 10;


	vector<Eigen::Affine3f> registered_transforms(transforms.size());
	vector<Eigen::Affine3f> transforms_aligned_to_registered(transforms.size());

	//this_thread::sleep_for(chrono::seconds(1));

	for(int j = 0; j < transforms.size(); j+=num_parallel)
	{
		cout << j+1 << " / " << transforms.size() << endl;

		//this_thread::sleep_for(chrono::milliseconds(200));

#pragma omp parallel
		{
			//pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
			pcl::PointCloud<pcl::PointXYZ>::Ptr orig_slam_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
			pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_slam_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
			pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_slam_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
			pcl::PointCloud<pcl::Normal>::Ptr slam_normals (new pcl::PointCloud<pcl::Normal> ());
			pcl::PointCloud<pcl::PointNormal>::Ptr slam_cloud_normals (new pcl::PointCloud<pcl::PointNormal> ());
			pcl::PointCloud<pcl::PointNormal>::Ptr lidar_slice_cloud (new pcl::PointCloud<pcl::PointNormal> ());
			pcl::PointCloud<pcl::PointNormal> final_cloud;

#pragma omp for
			for (int k = 0; k < num_parallel; k++)
			{

				// The starting transform:
				// We start with the SLAM transform
				auto adjusted_transform = transforms[j+k];

				if (j > 0)
				{
					// After the first iteration, we use the difference between SLAM transforms (tr[j-1].inv() * tr[j])
					// and apply it to the previously registered pose.
					//adjusted_transform = registered_transforms[j-1] * (transforms[j-1].inverse() * transforms[j+k]);

					// Or we simply use the last registered pose if we don't trust the SLAM trajectory at all
					adjusted_transform = registered_transforms[j-1];
				}
				
				// Load SLAM sensor data, and apply the transform
				pcl::io::loadPLYFile (paths[j+k], *orig_slam_cloud);

				pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ> ());
				range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 3.0)));
				pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
				condrem.setCondition (range_cond);
				condrem.setInputCloud (orig_slam_cloud);
				condrem.filter(*filtered_slam_cloud);

				pcl::transformPointCloud (*filtered_slam_cloud, *transformed_slam_cloud, adjusted_transform);
				computeNormals<PointXYZ, Normal>(transformed_slam_cloud, slam_normals);
				concatenateFields(*transformed_slam_cloud, *slam_normals, *slam_cloud_normals);
				
				// Find bounding box
				PointXYZ slam_min, slam_max;
				getMinMax3D(*transformed_slam_cloud, slam_min, slam_max);

				// Crop the reference cloud using the bounding box from SLAM data (plus margin)
				pcl::CropBox<PointNormal> cropFilter;
				cropFilter.setInputCloud (source_with_normals);
				cropFilter.setMin(Eigen::Vector4f(slam_min.x-margin_dist, slam_min.y-margin_dist, slam_min.z-margin_dist, 1.0f) );
				cropFilter.setMax(Eigen::Vector4f(slam_max.x+margin_dist, slam_max.y+margin_dist, slam_max.z+margin_dist, 1.0f) );
   				
				cropFilter.filter (*lidar_slice_cloud);

				
				
				//
				// The piece of code below is a previous attempt at projecting the reference cloud into the camera frame
				// using a depth map to limit the number of points.
				// It works okay, but a bounding box is simpler and tends to crop a more relevant portion of the cloud.
				//

				//pcl::transformPointCloud(*source_cloud, *transformed_cloud, transforms[j].inverse());

				//clearDepthMap();

				//for (int i = 0; i < transformed_cloud->size(); i++)
				//{
				//	auto& p = (*transformed_cloud)[i];
				//	if (p.z < 0 || p.z > max_dist) { continue; }
				//	int px, py;
				//	px = (p.x * fx) / p.z + cx;
				//	py = ((p.y * fy) / p.z + cy);
				//	if (px >= 0 && px < SIZE_X && py >= 0 && py < SIZE_Y)
				//	{
				//		if (depth_map[px][py].z > p.z)
				//		{
				//			depth_map[px][py] = p;
				//			depth_map_idx[px][py] = i;
				//		}
				//	}
				//}
				//
				//{
				//	lock_guard<mutex> guard(pointcloud_mutex);

				//	slice_cloud->clear();
				//	
				//	for (int x = 0; x < SIZE_X; x++)
				//	{
				//		for (int y = 0; y < SIZE_Y; y++)
				//		{
				//			int idx = depth_map_idx[x][y];
				//			if (idx >= 0)
				//			{
				//				slice_cloud->push_back((*source_cloud)[idx]);
				//			}
				//		}
				//	}
				//	
				//	pcl::transformPointCloud (*orig_slam_cloud, *slam_cloud, transforms[j]);
				//}

	

				// ICP from the SLAM cloud to the reference cloud
				//pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
				pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp;
				icp.setInputSource(slam_cloud_normals);
				icp.setInputTarget(lidar_slice_cloud);
				icp.setMaximumIterations (50);
				icp.setTransformationEpsilon (1e-8);
				
				icp.align(final_cloud);
				
				cout << "Converged: " << icp.hasConverged() << " Score: " << icp.getFitnessScore() << endl;

				// Compute the registered transform
				auto new_transform = Eigen::Affine3f(icp.getFinalTransformation()) * adjusted_transform;
				PointXYZ p;
				p.x = new_transform.translation()[0];
				p.y = new_transform.translation()[1];
				p.z = new_transform.translation()[2];

				registered_transforms[j+k] = new_transform;

				
				Eigen::Affine3f tf = transforms[j+k] * (transforms[0].inverse() * registered_transforms[0]);
				transforms_aligned_to_registered[j+k] = tf;


				// Display cropped reference cloud, SLAM cloud, aligned SLAM cloud, and aligned trajectory position
				if (display)
				{
					lock_guard<mutex> guard(pointcloud_mutex);
					copyPointCloud(final_cloud, *aligned_slam_cloud);
					copyPointCloud(*transformed_slam_cloud, *slam_cloud);
					copyPointCloud(*lidar_slice_cloud, *slice_cloud);
					(*aligned_traj_cloud)[j+k] = p; // keep this inside of mutex
				}
				else
				{
					(*aligned_traj_cloud)[j+k] = p;
				}


			}
		} // parallel

		if (save_temp < 0)
		{
			saveTrajectories(transforms, registered_transforms, transforms_aligned_to_registered);
			save_temp = 10;
		}
		save_temp--; // todo
	}





	if (display)
	{
		show_thread.join();
	}



	// ---
	// Save output
	// ---

	cout << "Saving output trajectories..." << endl;

	// Save point cloud of x,y,z positions
	pcl::io::savePLYFile("slam_traj.ply", *trajectory_cloud, true);
	pcl::io::savePLYFile("gt_traj.ply", *aligned_traj_cloud, true);

	// Save both slam and ground truth transforms in KITTI format
	saveTrajectories(transforms, registered_transforms, transforms_aligned_to_registered);



	// Merge the SLAM sensor data using aligned transforms
	cout << "Computing verification cloud" << endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr orig_slam_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_slam_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::PointCloud<pcl::PointXYZ>::Ptr verification_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::PointCloud<pcl::PointXYZ>::Ptr large_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
	for(int j = 0; j < registered_transforms.size(); j++)
	{	
		pcl::io::loadPLYFile (paths[j], *orig_slam_cloud);
		pcl::transformPointCloud (*orig_slam_cloud, *transformed_slam_cloud, registered_transforms[j]);
		*large_cloud += *transformed_slam_cloud;

		if ((j > 0 && j % 30 == 0) || j == registered_transforms.size()-1)
		{
			voxelGrid<PointXYZ>(large_cloud, downsampled_cloud, voxel_size, voxel_size, voxel_size);
			*verification_cloud += *downsampled_cloud;
			large_cloud->clear();
		}
	}


	pcl::io::savePLYFile("verif.ply", *verification_cloud, true);


	return 0;
}
