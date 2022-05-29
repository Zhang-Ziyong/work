#!/bin/bash
source install/setup_rely.bash

if [ -f /userdata/share/robot/fastrtps_config.xml ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE=/userdata/share/robot/fastrtps_config.xml
else
    DIR_T="$( cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    export FASTRTPS_DEFAULT_PROFILES_FILE=${DIR_T}/install/slam2d_ros2/fastrtps_config.xml
fi

ros2 run slam2d_ros2 slam2d --ros-args --params-file /oem/robot_params/install/robot_params/slam2d/slam2d_ros2_params_mission_clean.yaml

