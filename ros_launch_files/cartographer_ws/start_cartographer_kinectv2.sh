#!/bin/bash

source /home/e4e/cartographer_ws/devel_isolated/setup.sh --extend

roslaunch cartographer_ros kinectv2.launch "$@"

