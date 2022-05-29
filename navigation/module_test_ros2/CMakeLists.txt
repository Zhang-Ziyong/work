cmake_minimum_required(VERSION 3.5)
project(module_test_ros2)

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

add_compile_options(-std=c++11)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(costmap_ros2 REQUIRED)
find_package(costmap_base REQUIRED)
find_package(costmap_layer REQUIRED)
find_package(costmap_builder REQUIRED)
find_package(costmap_mediator REQUIRED)
find_package(pose2d REQUIRED)
find_package(perception REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(speed_decision REQUIRED)
# find_package(cvte_sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(planner_utils REQUIRED)

include_directories(
 perception
 ${GLOG_INCLUDE_DIRS}
 ${costmap_base_INCLUDE_DIRS}
 ${costmap_layer_INCLUDE_DIRS}
 ${costmap_builder_INCLUDE_DIRS}
 ${perception_INCLUDE_DIRS}
 ${sensor_msgs_INCLUDE_DIRS}
 ${speed_decision_INCLUDE_DIRS}
#  ${cvte_sensor_msgs_INCLUDE_DIRS}
 ${geometry_msgs_INCLUDE_DIRS}
 ${planner_utils_INCLUDE_DIRS}
)

add_library(perception_utility perception/perception_utility.cpp)
target_link_libraries(perception_utility ${costmap_base_LIBRARIES} ${sensor_msgs_LIBRARIES})
ament_target_dependencies(perception_utility 
                            "rclcpp" 
                            "costmap_ros2" 
                            "nav_msgs" 
                            "costmap_base" 
                            "costmap_layer"
                            "costmap_builder"
                            "perception" 
                            "costmap_mediator" 
                            "pose2d")

add_executable(narrow_recognition_node perception/narrow_recognition_node.cpp)
target_link_libraries(narrow_recognition_node 
    -lglog 
    ${costmap_base_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    ${perception_LIBRARIES}
    perception_utility
)
ament_target_dependencies(narrow_recognition_node 
                          "rclcpp" 
                          "costmap_ros2" 
                          "nav_msgs" 
                          "costmap_base" 
                          "costmap_layer"
                          "costmap_builder"
                          "perception" 
                          "costmap_mediator" 
                          "pose2d")

add_executable(strip_marker_node marker/strip_marker_node.cpp)
target_link_libraries(strip_marker_node -lglog -ljsoncpp perception_utility ${speed_decision_LIBRARIES})
ament_target_dependencies(strip_marker_node
                          "rclcpp"  
                          "nav_msgs"
                          "speed_decision"
                          "costmap_base"
                          "planner_utils")

# add_executable(collision_switch_node sim_sensor/collision_switch_node.cpp)
# ament_target_dependencies(collision_switch_node "rclcpp" "geometry_msgs" "cvte_sensor_msgs")

install(TARGETS narrow_recognition_node strip_marker_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  narrow_recognition_node strip_marker_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY perception/
  DESTINATION include/
)

ament_export_include_directories(perception)
ament_export_libraries(perception_utility)

ament_package()
