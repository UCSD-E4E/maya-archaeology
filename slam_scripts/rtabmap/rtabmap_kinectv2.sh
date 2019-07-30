#!/bin/bash

dblink="/home/e4e/.ros/rtabmap.db"

DATE=`date +%Y_%m_%d_%H_%M_%S`

if [ -z "$ROS_HOME" ]; then
	ROS_HOME=$HOME/.ros
fi

dbfilename="$ROS_HOME/rtabmap_$DATE.db"

if [ -f $dblink	]; then
    rm $dblink
fi

ln -fs ${dbfilename} ${dblink}

roslaunch rtabmap_kinectv2.launch "$@"

rm $dblink

echo "Database saved to $dbfilename"

