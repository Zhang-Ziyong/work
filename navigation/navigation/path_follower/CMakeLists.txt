cmake_minimum_required(VERSION 3.5)

project(path_follower)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED COMPONENTS)
find_package(ament_cmake_ros REQUIRED)
find_package(pose2d REQUIRED)
find_package(planner_utils REQUIRED)
find_package(costmap_base REQUIRED)
find_package(data_manager REQUIRED)
find_package(costmap_mediator REQUIRED)
find_package(planner_utils REQUIRED)
find_package(local_controller REQUIRED)
find_package(robot_trajectory REQUIRED)
find_package(local_planner REQUIRED)
find_package(speed_decision REQUIRED)
find_package(perception REQUIRED)
find_package(pnc_map REQUIRED)
find_package(OpenCV REQUIRED)
find_package(algorithm_packets REQUIRED)

include_directories(
 include
 ${GLOG_INCLUDE_DIRS}
 ${pose2d_INCLUDE_DIRS}
 ${planner_utils_INCLUDE_DIRS}
 ${costmap_base_INCLUDE_DIRS}
 ${costmap_mediator_INCLUDE_DIRS}
 ${planner_utils_INCLUDE_DIRS}
 ${data_manager_INCLUDE_DIRS}
 ${local_controller_INCLUDE_DIRS}
 ${robot_trajectory_INCLUDE_DIRS}
 ${local_planner_INCLUDE_DIRS}
 ${perception_INCLUDE_DIRS}
 ${speed_decision_INCLUDE_DIRS}
 ${algorithm_packets_INCLUDE_DIRS}
)

add_library(path_follower
 src/path_follower.cpp
#  src/speed_controller.cpp
 src/path_manager.cpp
#  src/turn_lighter.cpp
 
 src/supervisor/angle_topath_supervisor.cpp
 src/supervisor/dist_topath_supervisor.cpp
 src/supervisor/obstacle_avoider_supervisor.cpp
 src/supervisor/supervisor_chain.cpp
 src/supervisor/final_goal_supervisor.cpp
 src/supervisor/speed_level_supervisor.cpp
 src/supervisor/over_time_supervisor.cpp
 src/supervisor/marker_map_supervisor.cpp
 src/supervisor/pass_pit_supervisor.cpp
 
#  src/supervisor/car_avoider_supervisor.cpp
#  src/car_avoiding/car_avoiding_util.cpp
#  src/car_avoiding/point_sampler.cpp
#  src/car_avoiding/point_validity_checker.cpp
#  src/car_avoiding/visualizer.cpp

 src/planner_decision/planner_decision.cpp 

#  src/recovery/path_recovery.cpp
 src/recovery/front_path_recovery.cpp 
 src/recovery/rear_path_recovery.cpp 
 src/recovery/round_path_recovery.cpp 

 src/state/pit_state.cpp
 src/state/local_state.cpp
 src/state/pause_state.cpp
 src/state/refer_state.cpp
 src/state/free_state.cpp
 src/state/done_state.cpp
 src/state/rotate_state.cpp
 src/state/recover_state.cpp
 src/state/elevator_done_state.cpp 
#  src/state/avoidcar_state.cpp
 src/state/state_utils.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  "costmap_mediator"
  "data_manager"
  "OpenCV"
  "planner_utils"
  "speed_decision"
  "perception"
  "planner_utils"
  "pnc_map"
)

target_link_libraries(${PROJECT_NAME}
  -lglog
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(false)
  find_package(GTest REQUIRED)
  include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

  # add_executable(test_path_recovery 
  #   src/recovery/path_recovery.cpp
  #   test/test_path_recovery.cpp
  # )

  # target_link_libraries(test_path_recovery -lglog -lgtest -lpthread)

  # ament_target_dependencies(test_path_recovery
  #   "costmap_mediator"
  #   "data_manager"
  # )

  install(TARGETS test_path_recovery
  DESTINATION lib/${PROJECT_NAME})

endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(data_manager)
ament_package()

