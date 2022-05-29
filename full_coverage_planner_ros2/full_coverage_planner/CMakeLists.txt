cmake_minimum_required(VERSION 3.5)
project(full_coverage_planner_ros2)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-std=c++17)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC -shared)
endif()

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED filesystem)
find_package(rclcpp REQUIRED)
find_package(mission_manager_msgs REQUIRED)
find_package(full_coverage_planner REQUIRED)

#############
# LIBRARIES #
#############

add_executable(full_coverage_planner_multi_area_ccpp_mission_manager
  ros_adapter/ros2_multi_ccpp_mission_mananger.cc
)

ament_target_dependencies(full_coverage_planner_multi_area_ccpp_mission_manager
  "rclcpp"
  "mission_manager_msgs"
  "full_coverage_planner"
  "Boost"
)

target_link_libraries(full_coverage_planner_multi_area_ccpp_mission_manager
  -lglog -ljsoncpp
)


#############
## Install ##
#############

install(FILES fastrtps_config.xml
  DESTINATION ./)

install(
  TARGETS 
  full_coverage_planner_multi_area_ccpp_mission_manager
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY params/
  DESTINATION params/)

ament_package()
