#!/usr/bin/env bash
# Script to run ROVIOLI from a Euroc live data source (e.g., Euroc bag file with rosbag play).
# Usage: tutorial_euroc <output save folder> [<additional rovioli flags>]

LOCALIZATION_MAP_OUTPUT=$1
NCAMERA_CALIBRATION="ncamera.yaml"
IMU_PARAMETERS_MAPLAB="imu-zr300-maplab.yaml"
IMU_PARAMETERS_ROVIO="imu-zr300-rovio.yaml"
REST=$@

rosrun rovioli rovioli \
  --alsologtostderr=1 \
  --v=2 \
  --ncamera_calibration=$NCAMERA_CALIBRATION  \
  --imu_parameters_maplab=$IMU_PARAMETERS_MAPLAB \
  --imu_parameters_rovio=$IMU_PARAMETERS_ROVIO \
  --datasource_type="rostopic" \
  --save_map_folder="$LOCALIZATION_MAP_OUTPUT" \
  --map_builder_save_image_as_resources=false \
	--rovioli_visualize_map=true \
	--rovioli_run_map_builder=true \
	--optimize_map_to_localization_map=false $REST
