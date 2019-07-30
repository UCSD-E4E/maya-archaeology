#!/bin/bash

source /home/e4e/cartographer_ws/install_isolated/setup.bash

roslaunch cartographer_ros realsense.launch "$@"

