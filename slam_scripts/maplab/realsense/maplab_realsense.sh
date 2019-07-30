#!/bin/bash

DATE=`date +%Y_%m_%d_%H_%M_%S`

if [ -z "$ROS_HOME" ]; then
	ROS_HOME=$HOME/.ros
fi

out_dir="$ROS_HOME/maplab_t265_$DATE"

roslaunch rs_d435_and_t265.launch map_output:=${out_dir} "$@"

echo "Map saved to ${out_dir}"

