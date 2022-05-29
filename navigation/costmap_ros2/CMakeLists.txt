cmake_minimum_required(VERSION 3.5)
project(costmap_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED COMPONENTS)
find_package(ament_cmake_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(lidar_perception_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3)

find_package(pose2d REQUIRED)
find_package(data_manager REQUIRED)
find_package(costmap_mediator REQUIRED)
find_package(costmap_base REQUIRED)
find_package(costmap_builder REQUIRED)
find_package(costmap_layer REQUIRED)
find_package(public_parameters REQUIRED)
# find_package(cvte_sensor_msgs REQUIRED)

include_directories(include
  ${pose2d_INCLUDE_DIRS}
  ${costmap_mediator_INCLUDE_DIRS}
  ${costmap_base_INCLUDE_DIRS}
  ${costmap_builder_INCLUDE_DIRS}
  ${costmap_layer_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}

)

add_library(${PROJECT_NAME}
  src/common/costmap_params_ros2.cpp
  src/manager_ros2/data_manager_ros2.cpp
  src/manager_ros2/map_manager_ros2.cpp
  src/visualizer_ros2/costmap_visualizer_ros2.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "costmap_mediator"
  "costmap_base"
  "costmap_builder"
  "costmap_layer"
  # "cvte_sensor_msgs"
  "sensor_msgs"
  "tf2_ros"
  "std_msgs"
  "lidar_perception_msgs"
  "visualization_msgs"
  "data_manager"
  "public_parameters"
)

target_link_libraries(${PROJECT_NAME}
  -lglog
)

add_executable(${PROJECT_NAME}_main
  src/main.cpp
)

target_link_libraries(${PROJECT_NAME}_main
  ${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(TARGETS ${PROJECT_NAME}_main
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY params/
  DESTINATION params/
)

if(false)
# Setup testing
  enable_testing()
  find_package(GTest REQUIRED)
  include_directories(${GTEST_INCLUDE_DIR})

  # test for layered_costmap
  add_executable(test_layered_costmap test/test_layered_costmap.cpp)
  target_link_libraries(test_layered_costmap ${PROJECT_NAME} -lgtest -lpthread)

  # test for message_filter
  add_executable(test_message_filter test/test_message_filter.cpp)
  target_link_libraries(test_message_filter ${PROJECT_NAME} -lgtest -lpthread)

  install(TARGETS test_layered_costmap test_message_filter
  DESTINATION lib/${PROJECT_NAME})
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(lidar_perception_msgs)

ament_package()
