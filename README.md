# maya-archaeology
various pipelines for slam, registration, error comparison algorithms for maya tunnel scanning

## go-icp script
	install GoICP
	install PCL
	
To install:
```
cd go-icp-script
mkdir build
cd build
cmake ..
make
```

To run (config.txt - adjust parameters for registration)
```
cd go-icp-script/scripts
python go-icp.py lidar.ply slam.ply
```
where lidar.ply and slam.ply are the reference and reading models, respectively

Model Files will be located in 
```go-icp-script/models/```
	-> lidar.ply (original reference file)
	-> mat.txt (transformation matrix file)
	-> slam-t.ply (transformed slam file)
	-> error-p2p.pcd (error file of point to point error)
	-> error-p2pl.pcd (error file of point to plane error)

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

Note: These specific scripts work only for Kinect version 1. To make it for different cameras, you need to change generate_pointcloud_kinect1.py file. Substitute focalLength, centerX, centerY, scalingFactor with your camera specs. You also need to modify get_time_stamp_and_depth_images.py to choose the right rostopic.



