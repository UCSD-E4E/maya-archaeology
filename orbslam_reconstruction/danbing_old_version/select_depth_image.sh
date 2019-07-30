#!/bin/bash

clear
file="selected_frame.txt"
while read -r line
do
	idx="$line"
	idx=$(printf %06d $idx).png
	path="depth_image/depth$idx"
	echo "Moving $path"
	cp "$path" "selected_depth_image/$idx"
done < "$file"
