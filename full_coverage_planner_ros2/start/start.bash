#!/bin/bash
source install/setup_rely.bash
DIR_T="$( cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export FASTRTPS_DEFAULT_PROFILES_FILE=${DIR_T}/install/full_coverage_planner_ros2/fastrtps_config.xml
ros2 run full_coverage_planner_ros2 full_coverage_planner_multi_area_ccpp_mission_manager --ros-args --params-file ./install/full_coverage_planner_ros2/params/coverage_planner.yaml
