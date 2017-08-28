#!/usr/bin/env python

clear
rostime="timestamp.txt"
for file in selected_depth_image/*.png
do
	source=${file:21:6}
	let index=10#$source+1
	echo "Converting depth image $source"
	time_stamp=$(sed -n "${index}p" "$rostime")
	python generate_pointcloud_kinect1.py $file "point_clouds/$source.ply"
done

