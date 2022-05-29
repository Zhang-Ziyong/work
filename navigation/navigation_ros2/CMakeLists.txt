cmake_minimum_required(VERSION 3.5)
project(navigation_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(robot_trajectory_msgs REQUIRED)
find_package(robot_trajectory REQUIRED)
find_package(costmap_ros2 REQUIRED)
find_package(pose2d REQUIRED)
find_package(pnc_map REQUIRED)
find_package(Protobuf REQUIRED)
find_package(logic_controller REQUIRED)
find_package(local_controller REQUIRED)
find_package(local_planner REQUIRED)
find_package(global_planner REQUIRED)
find_package(path_follower REQUIRED)
find_package(planner_utils REQUIRED)
find_package(costmap_mediator REQUIRED)
find_package(costmap_base REQUIRED)
find_package(costmap_builder REQUIRED)
find_package(costmap_layer REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED QUIET COMPONENTS common)
find_package(mission_manager_msgs REQUIRED)
find_package(log REQUIRED)
find_package(speed_decision REQUIRED)
find_package(clean_decision REQUIRED)
find_package(perception REQUIRED)
#find_package(osqp REQUIRED)
find_package(Git REQUIRED)
find_package(SDL REQUIRED)
find_package(SDL_image REQUIRED)
find_package(public_parameters REQUIRED)

if(GIT_FOUND)
execute_process(
    COMMAND ${GIT_EXECUTABLE} log -1 --pretty=format:%h
    OUTPUT_VARIABLE COMMIT_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)
execute_process(
    COMMAND ${GIT_EXECUTABLE} symbolic-ref --short -q HEAD
    OUTPUT_VARIABLE BRANCH_NAME
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)
endif()

string(TIMESTAMP TIME_VERSION "%Y-%m-%d %H:%M:%S")
message("SW version is ${BRANCH_NAME}:${COMMIT_HASH} ${TIME_VERSION}")

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/common/utils/SwVersion.hpp.ini
  ${CMAKE_CURRENT_SOURCE_DIR}/include/SwVersion.hpp
  @ONLY
)

include_directories(include
  ${pose2d_INCLUDE_DIRS}
  ${costmap_ros2_INCLUDE_DIRS}
  ${logic_controller_INCLUDE_DIRS}
  ${local_planner_INCLUDE_DIRS}
  ${global_planner_INCLUDE_DIRS}
  ${local_controller_INCLUDE_DIRS}
  ${path_follower_INCLUDE_DIRS}
  ${planner_utils_INCLUDE_DIRS}
  ${costmap_mediator_INCLUDE_DIRS}
  ${costmap_base_INCLUDE_DIRS}
  ${costmap_builder_INCLUDE_DIRS}
  ${costmap_layer_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${robot_trajectory_msgs_INCLUDE_DIRS}
  ${robot_trajectory_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${PCL_COMMON_INCLUDE_DIRS}
  ${log_INCLUDE_DIRS}
  ${speed_decision_INCLUDE_DIRS}
  ${perception_INCLUDE_DIRS}
  ${clean_decision_INCLUDE_DIRS}
  ${SDL_INCLUDE_DIRS}
  ${SDL_IMAGE_INCLUDE_DIRS}
)



add_library(navigation_lib
  src/arch_adapter/navigation_arch_adapter_ros2.cpp
  src/arch_adapter/navigation_params_ros2.cpp
  src/arch_adapter/zmq_server.cpp
  src/path_tools/path_optimize.cpp
  #src/arch_adapter/path_optimize.cpp
  #src/rfsm/navigation_rfsm.cpp
  src/data_handle/range_sensor.cpp
  src/data_handle/range_sensor_ros2.cpp
  )

ament_target_dependencies(navigation_lib
  "rclcpp"
  "geometry_msgs"
  "visualization_msgs"
  "nav_msgs"
  "costmap_mediator"
  "costmap_base"
  "costmap_builder"
  "costmap_layer"
  "sensor_msgs"
  "yaml_cpp_vendor"
  "costmap_ros2"
  "pcl_conversions"
  "mission_manager_msgs"
  "log"
  "speed_decision"
  "perception"
  "clean_decision"
  "pnc_map"
  "public_parameters"
)

target_link_libraries(navigation_lib
  ${costmap_ros2_LIBRARIES}
  ${logic_controller_LIBRARIES}
  ${local_controller_LIBRARIES}
  ${local_planner_LIBRARIES}
  ${global_planner_LIBRARIES}
  ${path_follower_LIBRARIES}
  ${planner_utils_LIBRARIES}
  ${robot_trajectory_msgs_LIBRARIES}
  ${robot_trajectory_LIBRARIES}
  ${costmap_mediator_LIBRARIES}
  ${costmap_base_LIBRARIES}
  ${costmap_layer_LIBRARIES}
  ${costmap_builder_LIBRARIES}
  ${yaml_cpp_vendor_LIBRARIES}
  ${PCL_LIBRARIES}
  ${log_LIBRARIES}
  ${speed_decision_LIBRARIES}
  ${perception_LIBRARIES}
  ${clean_LIBRARIES}
  ${SDL_LIBRARY}
  ${SDL_IMAGE_LIBRARIES}
  #osqp::osqp
  -lglog
  -lnlopt
  -ljsoncpp
  -lzmq
  )

add_executable(navigation_ros2
  src/main.cpp
)

target_link_libraries(navigation_ros2
  navigation_lib)

add_executable(navigation_ros2_mission_manager
  src/mission_manager_main.cpp
)

target_link_libraries(navigation_ros2_mission_manager
  navigation_lib)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(TARGETS navigation_ros2 navigation_ros2_mission_manager
  DESTINATION lib/${PROJECT_NAME})

  install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES fastrtps_config.xml
  DESTINATION ./)

install(
  DIRECTORY params/
  DESTINATION params/)

#if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
#  # the following line skips the linter which checks for copyrights
#  # remove the line when a copyright and license is present in all source files
#  set(ament_cmake_copyright_FOUND TRUE)
#  # the following line skips cpplint (only works in a git repo)
#  # remove the line when this package is a git repo
#  set(ament_cmake_cpplint_FOUND TRUE)
#  ament_lint_auto_find_test_dependencies()
#
#  find_package(GTest REQUIRED)
#  include_directories( include ${GTEST_INCLUDE_DIR})
#
#  # test for optimize
#  add_executable(test_optimize test/test_path_optimize.cpp)
#  target_link_libraries(test_optimize navigation_lib -lglog -lgtest -lpthread)
#
#  install(TARGETS test_optimize  
#  DESTINATION lib/${PROJECT_NAME})
#endif()

ament_package()
