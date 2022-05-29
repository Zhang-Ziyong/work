#!/bin/bash
source install/setup_rely.bash

if [ -f /userdata/share/robot/fastrtps_config.xml ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE=/userdata/share/robot/fastrtps_config.xml
else
    DIR_T="$( cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    export FASTRTPS_DEFAULT_PROFILES_FILE=${DIR_T}/install/navigation_ros2/fastrtps_config.xml
fi

taskset -a 0x30 ros2 run navigation_ros2 navigation_ros2_mission_manager --ros-args --params-file /oem/robot_params/install/robot_params/navigation/navigation_param_kinodynamic_clean.yaml

 
