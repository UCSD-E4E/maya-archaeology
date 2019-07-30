#!/usr/bin/env bash
# Script to run ROVIOLI from a live data source (e.g., Euroc bag file with rosbag play).
# Usage: ./run.sh <output save folder> <ncamera.yaml> <imu_maplab.yaml> <imu_rovio.yaml> [<additional rovioli flags>]

LOCALIZATION_MAP_OUTPUT=$1
NCAMERA_CALIBRATION=$2
IMU_PARAMETERS_MAPLAB=$3
IMU_PARAMETERS_ROVIO=$4
REST=${@:5}

rosrun rovioli rovioli \
  --alsologtostderr=1 \
  --v=2 \
  --ncamera_calibration=$NCAMERA_CALIBRATION  \
  --imu_parameters_maplab=$IMU_PARAMETERS_MAPLAB \
  --imu_parameters_rovio=$IMU_PARAMETERS_ROVIO \
  --datasource_type="rostopic" \
  --save_map_folder="$LOCALIZATION_MAP_OUTPUT" \
  --map_builder_save_image_as_resources=false \
  --optimize_map_to_localization_map=false \
  --feature_tracker_visualize_feature_tracks \
  $REST
