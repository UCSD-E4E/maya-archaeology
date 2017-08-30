#!/bin/bash

DATE=`date +%Y_%m_%d_%H_%M_%S`

ln -fs "/home/e4e/.ros/rtabmap_$DATE.db" "/home/e4e/.ros/rtabmap.db"

roslaunch rtabmap_kinectv1.launch "$@"

rm ~/.ros/rtabmap.db

