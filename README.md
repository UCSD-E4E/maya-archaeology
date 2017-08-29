# maya-archaeology
various pipelines for slam, registration, error comparison algorithms for maya tunnel scanning

## go-icp script
To setup initially and run:
Follow the instructions [here](https://github.com/waseemkhan96/go-icp-docker) to setup a docker image for the dependencies

To run 
After inside docker container (/root/maya-archaeology/scripts/)
Edit the config.txt to adjust the parameters for registration
```
python go-icp.py lidar.ply slam.ply
```
where lidar.ply and slam.ply are the reference and reading models, respectively

Model Files will be located in 
```go-icp-script/models/```
lidar.ply (original reference file)
mat.txt (transformation matrix file)
slam-t.ply (transformed slam file)
error-p2p.pcd (error file of point to point error)
error-p2pl.pcd (error file of point to plane error)

Make sure to copy the results to the $MODEL_DIR that was volume mounted.
If X forwarding is enabled when starting the docker container, the error pcd files can be viewed with pcl_viewer
```
docker run -it --rm \
  -v $MODEL_DIR:/models/ \
  -w=/root/maya-archaeology/go-icp-script/models \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  wasd/go-icp-docker
pcl_viewer error-p2pl.pcd
```
Otherwise, they can be converted to ply by running, for example:
```
pcl1.8_pcd2ply error-p2pl.pcd error-p2pl.ply
```

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



