cmake_minimum_required(VERSION 3.5)

project(costmap_layer)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_C_FLAGS_RELEASE "-O3")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

find_package(ament_cmake_auto REQUIRED COMPONENTS)
find_package(ament_cmake_ros REQUIRED)
find_package(costmap_base REQUIRED)
find_package(costmap_mediator REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(data_manager REQUIRED)
find_package(rclcpp REQUIRED)


include_directories(
 include
 ${costmap_mediator_INCLUDE_DIRS}
 ${costmap_base_INCLUDE_DIRS}
 ${EIGEN3_INCLUDE_DIR}
 ${data_manager_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/layer_creator.cpp
  src/costmap_layer.cpp
  src/static_layer.cpp
  src/inflation_layer.cpp
  src/obstacle_layer.cpp
  src/range_sensor_layer.cpp
  src/voxel_layer.cpp
  src/probability_voxel_layer.cpp
  src/negative_obstacles_layer.cpp
  src/collision_layer.cpp
  src/range_sensor_layer2.cpp
)

target_link_libraries(${PROJECT_NAME}
-lglog
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  costmap_base
  costmap_mediator
  data_manager
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

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(costmap_base)
ament_export_dependencies(costmap_mediator)
ament_export_dependencies(data_manager)

if(false)
# Setup testing
    enable_testing()
    find_package(GTest REQUIRED)
    include_directories(${GTEST_INCLUDE_DIR})

    # test for inflation_layer
    add_executable(test_inflation_layer test/test_inflation_layer.cpp)
    target_link_libraries(test_inflation_layer  ${PROJECT_NAME} -lgtest -lpthread)

    # test for static_layer
    add_executable(test_static_layer test/test_static_layer.cpp)
    target_link_libraries(test_static_layer 
      ${PROJECT_NAME} -lgtest gtest_main -pthread)

    # test for obstacle_layer
    add_executable(test_obstacle_layer test/test_obstacle_layer.cpp)
    target_link_libraries(test_obstacle_layer
      ${PROJECT_NAME} -lgtest gtest_main -pthread)

    # test for voxel_layer
    add_executable(test_voxel_layer test/test_voxel_layer.cpp)
    target_link_libraries(test_voxel_layer
      ${PROJECT_NAME} -lgtest gtest_main -pthread)

    # test for range_sensor_layer
    add_executable(test_range_sensor_layer test/test_range_sensor_layer.cpp)
    target_link_libraries(test_range_sensor_layer
        ${PROJECT_NAME} -lgtest gtest_main -pthread)

    install(TARGETS test_static_layer test_range_sensor_layer test_inflation_layer
      test_obstacle_layer test_voxel_layer
      DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()

