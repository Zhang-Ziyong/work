#!/bin/bash
source install/setup_rely.bash

if [ -f /userdata/share/robot/fastrtps_config.xml ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE=/userdata/share/robot/fastrtps_config.xml
else
    DIR_T="$( cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    export FASTRTPS_DEFAULT_PROFILES_FILE=${DIR_T}/install/cvte_lidar_slam/fastrtps_config.xml
fi

ros2 run cvte_lidar_slam lidar_slam_mission_manager --ros-args --params-file /oem/robot_params/install/robot_params/slam_2d/2DSLAM.yaml

