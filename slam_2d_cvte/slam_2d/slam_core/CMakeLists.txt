
# cmake_minimum_required(VERSION 3.5)
project(cvte_lidar_slam)

# # Default to C99
# if(NOT CMAKE_C_STANDARD)
#   set(CMAKE_C_STANDARD 99)
# endif()

# # Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic )
# endif()

set(CMAKE_BUILD_TYPE Release)

# set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
message("Start build slamcore lib")

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(chassis_interfaces REQUIRED)

include_directories(
  slam_core
  ${PROJECT_SOURCE_DIR}
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Ceres_INCLUDE_DIRS}
)

link_directories(
	${OpenCV_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
  ${Ceres_LIBRARY_DIRS}
)



add_library(slam_lib SHARED
            common/config/system_config.cpp
            common/data_struct/keyframe.cpp
            common/data_struct/our_cloud_downsample.cpp
            common/gps/Geocentric.cpp 
            common/gps/LocalCartesian.cpp 
            common/gps/Math.cpp 
            frontend/feature_extractor.cpp
            loopend/loop_track.cpp
            map/map_manager.cpp
            lidar_odom/lidar_odom.cpp
            map_track/map_track.cpp
            map_track/pose_solver.cpp
            msf/multi_sensors_fusion.cpp
            occupancy_map/new_occupancy_map.cpp
            occupancy_map/depth_camera_calibrate.cpp
            occupancy_map/depth_camera_occ_map.cpp
            occupancy_map/local_occupancy_map.cpp
            system/system.cpp
            state_machine/slam_state_machine.cpp 
            state_machine/stateMachine.cpp 
            common/registration/loam_registration.cpp
            )

ament_target_dependencies(slam_lib
  chassis_interfaces
  OpenCV
)
#############
###library###
#############

target_link_libraries(slam_lib
   ${Boost_SYSTEM_LIBRARY}
   ${PCL_LIBRARIES}
   ${OPENCV_LIBRARIES}
   ${CERES_LIBRARIES}
   -lglog
   -ljsoncpp
)



#############
## Install ##
#############

install(
  TARGETS  slam_lib 
  DESTINATION lib)


#############
##  test   ##
#############

# if(BUILD_TESTING)

#   add_executable(test_state_machine state_machine/test_state_machine.cpp 
#                                     state_machine/slam_state_machine.cpp 
#                                     state_machine/stateMachine.cpp)
#   target_link_libraries(test_state_machine -lglog)

# endif()