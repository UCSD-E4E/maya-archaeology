# maya-archaeology
This repository contains various pipelines for slam, registration, error comparison algorithms for maya tunnel scanning.  For more information about the project, refer to the [project wiki](https://github.com/UCSD-E4E/maya-archaeology/wiki).

## Go-ICP Script
[Go-ICP](http://jlyang.org/go-icp/) is a registration algorithm for 3d point clouds with global optimization. However, it needs nornalized models in txt file format as input. This container with script does pre-processing to normalize (values in \[-1,1\]) any ply files passed as input, generate the registration matrix (GoICP), register the reading model to the reference model, and calculate the error between the two models using [PCL](http://pointclouds.org/).

(Generated Example) Error Heatmap between Two Head Scan 3d Point Clouds  
![error heatmap gif](https://github.com/UCSD-E4E/maya-archaeology/blob/master/images/error-p2pl.gif)

### To Setup Initially and Run:
Follow the instructions [here](https://hub.docker.com/r/wasd/go-icp-docker/) to setup a docker image for the dependencies

### Details about Running
Inside the docker container (/root/maya-archaeology/scripts/)  
Edit the config.txt to adjust the parameters for registration and then run
```
python go-icp.py reference.ply reading.ply
```
where for example reference.ply would be the Ground Truth (lidar model) and reading.ply would be a Test (slam model). The script should output point to point error (RMSE of distance between closest points) and point to plane error (RMSE of distance between a point and its closest point's plane)

### Viewing Results
Model Files will be located in 
```go-icp-script/models/```  
reference.ply (original reference model)  
mat.txt (transformation matrix file)  
reading-t.ply (transformed reading model)  
error-p2p.pcd (error model of point to point error)  
error-p2pl.pcd (error model of point to plane error)  

Make sure to copy the results in go-icp-script/models/reading (where reading is the name of the reading.ply file) to the $MODEL_DIR (/models in container) that was volume mounted to save the results.

If X forwarding is enabled when starting the docker container, after running the registration and generating a models folder inside go-icp-script folder, error pcd files in the go-icp-script/models folder can be viewed with pcl_viewer (gif above)
```
pcl_viewer error-p2pl.pcd
```
Otherwise, they can be converted to ply by running, for example:
```
pcl1.8_pcd2ply error-p2pl.pcd error-p2pl.ply
```
Maintained by [Waseem Khan](https://github.com/waseemkhan96/)

## orbslam_reconstruction

ORBSLAM2 only saves (key) trajectory of the camera for user. These scripts are intended to generate 3D models in the form of point cloud based on the trajectory file and corresponding video recorded in ROS bag file.

The folder works as an empty template. 

Before running the scripts, put the trajectory file(.txt) and bag file(.bag) into the folder. Build the two .cpp applications by

```
cd apply_trans_matrix
mkdir build
cd build
cmake ..
make
```

```
cd downsample
mkdir build
cd build
cmake ..
make
```

Then run
```
python run.py <.bag file> <.txt trajectory file>
```

The result will be stored in downsampled_1.ply.

### Notes: 

These scripts depend on pcl library.

You can change the downsample rate by modifying the parameters of setLeafSize() function in downsample.cpp

These specific scripts work only for Kinect version 1. To make it work for different cameras, you need to change generate_pointcloud_kinect1.py file. Substitute focalLength, centerX, centerY, scalingFactor with your camera specs. 

You also need to modify get_time_stamp_and_depth_images.py to choose the right rostopic.  
Maintained by [Danbing Zhu](https://github.com/DanbingZhu)



