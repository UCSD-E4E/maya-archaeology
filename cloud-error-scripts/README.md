# Cloud error scripts

## `compute_cloud_error.cpp`

This is part of the PCL library but modified so that:
- We can input a list of cutoff distances (if the distance between point p1 on the reading point cloud and point p2 on the reference point cloud is greater than the cutoff distance, then do not include this distance in the final calculation for the RMSE)
- Outputs min and max distance between all points on the reading point cloud and the corresponding closest points on the reference point cloud


## `rmse_curve.py`

__Note: the file moved to "super4pcs-script/general".__

Calculates RMSE for a reading point cloud and reference point cloud for a specified number of cutoff distances between the min and max distances between all points on the reading model and their correspondiing points on the reference model. Uses the modified `compute_cloud_error.cpp`.

### Example Usage

The following command finds 10 evenly spaced cutoff distances (default setting), and calculates the corresponding RMSEs for `aligned-reading.pcd` and `lidar-g-1m-4.pcd` using the generated cutoff distances.

```
$ python3 rmse_curve.py aligned-reading.pcd lidar-g-1m-4.pcd -p
```
