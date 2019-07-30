#include <functional>
#include <string>

#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <tf/transform_listener.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "depth_map.hpp"


class PointCloudFusion
{
protected:
	typedef pcl::PointCloud<pcl::PointXYZ> point_cloud_t_;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> CloudSyncPolicy;

	// The fused point cloud itself
	point_cloud_t_            fused_cloud_;

	// Listener for tf frames
	tf::TransformListener     tf_listener_;

	// The name of the base frame
	std::string               base_frame_id_;

	image_transport::ImageTransport it_;

	ros::Publisher            pub_;
	image_transport::CameraPublisher depth_pub_;
	image_transport::CameraPublisher color_pub_;


	message_filters::Subscriber<sensor_msgs::PointCloud2> sub1_;
	message_filters::Subscriber<sensor_msgs::PointCloud2> sub2_;
	message_filters::Subscriber<sensor_msgs::CameraInfo>  info_sub_;
	message_filters::Synchronizer<CloudSyncPolicy> img_synchronizer_;

	tangodata::Intrinsics depth_intr_;
	tangodata::DepthMapCreator depth_creator_;
	std::vector<uint16_t> depth_map_;
	cv::Mat cv_map_;
	cv_bridge::CvImagePtr cv_ptr_;
	sensor_msgs::CameraInfoPtr depth_info_;

	cv::Mat cv_color_;
	cv_bridge::CvImagePtr cv_color_ptr_;




	// publish the fused cloud
	void publish_() const
	{
		// temporary PointCloud2 intermediary
		pcl::PCLPointCloud2 tmp_pc;

		// Convert fused from PCL native type to ROS
		pcl::toPCLPointCloud2(fused_cloud_, tmp_pc);
		sensor_msgs::PointCloud2 published_pc;
		pcl_conversions::fromPCL(tmp_pc, published_pc);

		published_pc.header.frame_id = base_frame_id_;

		// Publish the data
		pub_.publish(published_pc);
	}


public:

	PointCloudFusion(
			ros::NodeHandle& nh,
			const std::string& base_frame_id,
			const ros::Publisher& pub) :
		tf_listener_(ros::Duration(9999)),
		pub_(pub),
		it_(nh),
		img_synchronizer_(CloudSyncPolicy(10), sub1_, sub2_),
		cv_ptr_(new cv_bridge::CvImage),
		cv_color_ptr_(new cv_bridge::CvImage)
	{
		set_base_frame_id(base_frame_id);
		//nh.subscribe<sensor_msgs::CameraInfo>("/depth1_info", 1, &PointCloudFusion::set_cam1_info, this);
		sub1_.subscribe(nh, "points1", 1);
		sub2_.subscribe(nh, "points2", 1);
		img_synchronizer_.registerCallback(boost::bind(&PointCloudFusion::fuse_two_clouds, this, _1, _2));
		depth_creator_.max_depth = 0;
		depth_pub_ = it_.advertiseCamera("depth_fused/image", 1);
		color_pub_ = it_.advertiseCamera("color_fused/image", 1);
	}

	~PointCloudFusion() { }


	const std::string base_frame_id() const { return base_frame_id_; }


	// update base frame id - this will reset the fused point cloud
	void set_base_frame_id(const std::string& base_frame_id)
	{
		// clear current fused point cloud on a base frame change
		fused_cloud_.clear();

		// record new frame
		base_frame_id_ = base_frame_id;
	}

	void set_cam1_info(const sensor_msgs::CameraInfo& cam_info)
	{
		depth_intr_ = tangodata::Intrinsics(cam_info.width, cam_info.height, cam_info.K[0], cam_info.K[4], cam_info.K[2], cam_info.K[5]);
		depth_intr_.width *= 2.5;
		depth_intr_.cx *= 2;
		depth_intr_.fx *= 1;

		depth_info_.reset(new sensor_msgs::CameraInfo(cam_info));
		depth_info_->height = depth_intr_.height;
		depth_info_->width  = depth_intr_.width;
		depth_info_->K[0]   = depth_intr_.fx;
		depth_info_->K[4]   = depth_intr_.fy;
		depth_info_->K[2]   = depth_intr_.cx;
		depth_info_->K[5]   = depth_intr_.cy;

		depth_map_.resize(depth_intr_.height * depth_intr_.width);
		cv_map_ = cv::Mat(depth_intr_.height, depth_intr_.width, CV_16UC1, depth_map_.data());
        
		cv_ptr_->encoding = "16UC1";
        cv_ptr_->header.frame_id = "depth_fused";
        cv_ptr_->image = cv_map_;

		cv_color_ = cv::Mat(depth_intr_.height, depth_intr_.width, CV_8UC3, cv::Scalar(255,255,255));
		cv_color_ptr_->encoding = "8UC3";
        cv_color_ptr_->header.frame_id = "color_fused";
        cv_color_ptr_->image = cv_color_;
	}

	void fuse_two_clouds(
			const sensor_msgs::PointCloud2ConstPtr& ros_pc1,
			const sensor_msgs::PointCloud2ConstPtr& ros_pc2)
	{
		//if (depth_intr_.width <= 0) { return; }

		// Clear previous cloud
		fused_cloud_.clear();


		// transform the point cloud into base_frame_id
		sensor_msgs::PointCloud2 trans_ros_pc1;
		if(!pcl_ros::transformPointCloud(base_frame_id_, *ros_pc1, trans_ros_pc1, tf_listener_))
		{
			// Failed to transform
			ROS_WARN("Dropping input point cloud");
			return;
		}
		
		// transform the point cloud into base_frame_id
		sensor_msgs::PointCloud2 trans_ros_pc2;
		if(!pcl_ros::transformPointCloud(base_frame_id_, *ros_pc2, trans_ros_pc2, tf_listener_))
		{
			// Failed to transform
			ROS_WARN("Dropping input point cloud");
			return;
		}


		// Convert
		pcl::PCLPointCloud2 tmp_pc;
		point_cloud_t_ input;
		pcl_conversions::toPCL(trans_ros_pc1, tmp_pc);
		pcl::fromPCLPointCloud2(tmp_pc, input);
		fused_cloud_ += input;

		pcl_conversions::toPCL(trans_ros_pc2, tmp_pc);
		pcl::fromPCLPointCloud2(tmp_pc, input);
		fused_cloud_ += input;

		// Publish fused cloud
		publish_();

		// Project points to depth map
		depth_creator_.pointsToDepthMap(fused_cloud_, depth_intr_, depth_map_, tangodata::DEPTH_INTERP_BILINEAR);


        ros::Time time = ros::Time::now();

		// Publish fused depth map
        cv_ptr_->header.stamp = time;
        depth_pub_.publish(cv_ptr_->toImageMsg(), depth_info_);

		// Publish fake color image
		cv_color_ptr_->header.stamp = time;
        color_pub_.publish(cv_color_ptr_->toImageMsg(), depth_info_);
	}


	// callback when a new point cloud is available
	void add_cloud(const sensor_msgs::PointCloud2& ros_pc)
	{
		// temporary PointCloud2 intermediary
		pcl::PCLPointCloud2 tmp_pc;

		// transform the point cloud into base_frame_id
		sensor_msgs::PointCloud2 trans_ros_pc;
		if(!pcl_ros::transformPointCloud(base_frame_id_, ros_pc, trans_ros_pc, tf_listener_))
		{
			// Failed to transform
			ROS_WARN("Dropping input point cloud");
			return;
		}

		// Convert ROS point cloud to PCL point cloud
		// See http://wiki.ros.org/hydro/Migration for the source of this magic.
		pcl_conversions::toPCL(trans_ros_pc, tmp_pc);

		// Convert point cloud to PCL native point cloud
		point_cloud_t_ input;
		pcl::fromPCLPointCloud2(tmp_pc, input);

		// Fuse
		fused_cloud_ += input;

		// Publish fused cloud
		publish_();
	}
};



int main (int argc, char** argv)
{
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> CloudSyncPolicy;
	
	// Initialize ROS
	ros::init (argc, argv, "pcl_fuse");
	ros::NodeHandle nh;

	// Create a publisher for the fused data and create a PointCloudFusion object to do it.
	PointCloudFusion fusion(nh, "/camera3_frame", nh.advertise<sensor_msgs::PointCloud2>("/fused_points", 1));

	ROS_INFO("Waiting for camera info...");
	sensor_msgs::CameraInfo camInfo = *ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/depth1_info", nh);
	fusion.set_cam1_info(camInfo);
	ROS_INFO("Got camera info");

	// Spin
	ros::spin ();
}

