# maya-archaeology
This repository contains various pipelines for slam, registration, error comparison algorithms for maya tunnel scanning.  For more information about the project, refer to the [project wiki](https://github.com/UCSD-E4E/maya-archaeology/wiki).

# Folder structure

* [`old_experiments`](old_experiments): The original set of experiments to register point clouds and obtain error metrics. Later experiments have mostly used CloudCompare for registration and error metric.
* [`orbslam_reconstruction`](orbslam_reconstruction): Contains code and scripts to reconstruct ORB-SLAM2 point clouds from a trajectory file and a set of ROS bag files.
* [`slam_scripts`](slam_scripts): The scripts and ROS launch files used to run our experiments.
* [`trajectory_icp`](trajectory_icp): Code and scripts to reconstruct a trajectory from ROS bag files and a LiDAR point cloud.
