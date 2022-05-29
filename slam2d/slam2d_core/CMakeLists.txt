cmake_minimum_required(VERSION 3.5)
project(slam2d_core)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

if(NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE Release)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED COMPONENTS)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS iostreams)
find_package(Ceres REQUIRED COMPONENTS SuiteSparse)
find_package(yaml_cpp_vendor REQUIRED)
find_package(SDL REQUIRED)
find_package(SDL_image REQUIRED)
find_package(Protobuf 3.0.0 REQUIRED)
find_package(PCL REQUIRED QUIET)


include(FindPkgConfig)
PKG_SEARCH_MODULE(CAIRO REQUIRED cairo>=1.12.16)


file(GLOB_RECURSE ALL_PROTOS "*.proto")

set(ALL_PROTO_SRCS)
set(ALL_PROTO_HDRS)

foreach(ABS_FIL ${ALL_PROTOS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${ABS_FIL})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  get_filename_component(FIL_WE ${REL_FIL} NAME_WE)

  list(APPEND ALL_PROTO_SRCS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc")
  list(APPEND ALL_PROTO_HDRS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h")
  add_custom_command(
    OUTPUT "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc"
           "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h"
    #COMMAND  qemu-aarch64-static ${PROTOBUF_PROTOC_EXECUTABLE}
    COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
    ARGS --cpp_out  ${PROJECT_BINARY_DIR} 
      -I ${PROJECT_SOURCE_DIR} 
      ${ABS_FIL}
    DEPENDS ${ABS_FIL}
    COMMENT "Running C++ protocol buffer compiler on ${ABS_FIL}"
    VERBATIM
  )
endforeach()
set_source_files_properties(${ALL_PROTO_SRCS} ${ALL_PROTO_HDRS} PROPERTIES GENERATED TRUE)

include_directories(
  include
  ${sensor_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  ${CAIRO_INCLUDE_DIRS}
  ${SDL_INCLUDE_DIRS}
  ${SDL_IMAGE_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_library(slam2d_core
  src/common/time.cpp
  src/common/task.cpp
  src/common/image.cpp
  src/common/grid_map.cpp
  src/common/thread_pool.cpp
  src/common/point_cloud.cpp
  src/common/proto_stream.cpp
  src/common/rigid_transform.cpp
  src/common/probability_grid.cpp
  src/common/ray_to_pixel_mask.cpp
  src/common/probability_values.cpp
  src/common/fixed_ratio_sampler.cpp
  src/common/state_serialization.cpp
  src/common/ceres_solver_options.cpp
  src/common/value_conversion_tables.cpp
  src/common/proto_stream_deserializer.cpp
  src/common/probability_grid_range_data_inserter.cpp
  src/frontend/submap_2d.cpp
  src/frontend/voxel_filter.cpp
  src/frontend/motion_filter.cpp
  src/frontend/submap_painter.cpp
  src/frontend/pose_extrapolator.cpp
  src/frontend/local_trajectory_builder.cpp
  src/scan_matcher/laser_odometry_fast_loo.cpp
  src/scan_matcher/ceres_scan_matcher_2d.cpp
  src/scan_matcher/correlative_scan_matcher_2d.cpp
  src/scan_matcher/occupied_space_cost_function_2d.cpp
  src/scan_matcher/fast_correlative_scan_matcher_2d.cpp
  src/scan_matcher/real_time_correlative_scan_matcher_2d.cpp
  src/scan_matcher/global_scan_matcher_2d.cpp
  src/backend/histogram.cpp
  src/backend/pose_graph.cpp
  src/backend/constraint_builder.cpp
  src/backend/optimization_problem.cpp
  src/backend/spa_cost_function_2d.cpp
  src/backend/global_trajectory_builder.cpp
  src/amcl/amcl.cpp
  src/amcl/laser_model.cpp
  src/amcl/motion_model.cpp
  src/amcl/occupancy_grid.cpp
  src/amcl/particle_filter.cpp
  src/msf/multi_sensors_fusion.cpp
  src/state_machine/state_machine.cpp
  src/state_machine/slam_state_machine.cpp
  src/slam_system/mapping.cpp
  src/slam_system/slam_system.cpp
  src/slam_system/localization.cpp
  src/occupancy_map/depth_camera_calibrate.cpp
  src/occupancy_map/depth_camera_occ_map.cpp
  ${ALL_PROTO_SRCS}
  ${ALL_PROTO_HDRS}
)

ament_target_dependencies(slam2d_core
  "rclcpp"
  "sensor_msgs"
  "nav_msgs"
  "EIGEN3"
  "geometry_msgs"
)

target_link_libraries(slam2d_core
  -lglog
  ${EIGEN3_LIBRARIES}
  ${CERES_LIBRARIES}
  ${CAIRO_LIBRARIES}
  ${SDL_LIBRARY}
  ${SDL_IMAGE_LIBRARIES}
  ${yaml_cpp_vendor_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PROTOBUF_LIBRARY}
  ${PCL_LIBRARIES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

foreach(HDR ${ALL_PROTO_HDRS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_BINARY_DIR} ${HDR})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  install(
    FILES ${HDR}
    DESTINATION include/${DIR}
  )
endforeach()

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


ament_export_dependencies(yaml_cpp_vendor)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
