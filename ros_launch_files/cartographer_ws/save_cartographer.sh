#!/bin/bash

DATE=`date +%Y_%m_%d_%H_%M_%S`

source /home/e4e/cartographer_ws/install_isolated/setup.bash
rosservice call /finish_trajectory "trajectory_id: 0"
rosservice call /write_assets "cartog_$DATE"

