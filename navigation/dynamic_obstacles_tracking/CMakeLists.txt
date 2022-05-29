cmake_minimum_required(VERSION 3.5)
project(dynamic_obstacles_tracking)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED COMPONENTS)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pose2d REQUIRED)
find_package(log REQUIRED)

find_package(lidar_perception_msgs REQUIRED)

find_package( PCL 1.7 REQUIRED )
if( PCL_FOUND )
  # [C/C++]>[General]>[Additional Include Directories]
  include_directories( ${PCL_INCLUDE_DIRS} )

  # [C/C++]>[Preprocessor]>[Preprocessor Definitions]
  add_definitions( ${PCL_DEFINITIONS} )
  
  # [Linker]>[General]>[Additional Library Directories]
  link_directories( ${PCL_LIBRARY_DIRS} )
endif()

function(checkPackage package customMessage)
    set(varName "${package}_FOUND")
    if (NOT "${${varName}}")
        string(REPLACE "_" "-" aptPackage ${package})
        if("${customMessage}" STREQUAL "")
            message(FATAL_ERROR "\n\n ${package} is missing, please try to install it with:\n sudo apt-get install ros-$(rosversion -d)-${aptPackage}\n\n")
        else()
            message(FATAL_ERROR "\n\n ${customMessage} \n\n")
        endif()
    endif()
endfunction(checkPackage)

# INCLUDES and LIBS
include_directories(
    include
)

## add lidar project here
add_executable(tracking_node
    src/filters/mean_filter.cpp
    src/object_builders/min_box_object_builder.cpp
    src/matchers/observation_matcher.cpp
    src/clusters/cluster.cpp
    src/classifier/classifier.cpp
    src/trackers/object_tracker.cpp
    src/trackers/multi_object_tracker.cpp
    src/tracking_worker.cpp
#    src/ros_common.cpp
    src/tracking_node.cpp
)

target_link_libraries(tracking_node
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
  -lglog
  -lrt
)

ament_target_dependencies(tracking_node
  "rclcpp"
  "rcutils"
  "sensor_msgs"
  "std_msgs"
  "visualization_msgs"
  "pose2d"
  "lidar_perception_msgs"
  "log"
)

install(TARGETS tracking_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY params/
  DESTINATION params/
)

if(BUILD_TESTING)
# Setup testing
  enable_testing()
  find_package(GTest REQUIRED)
  include_directories(${GTEST_INCLUDE_DIR})

  # test for tracking
  add_executable(test_tracking test/test_tracking.cpp)
  target_link_libraries(test_tracking -lgtest -lpthread -lglog)

  install(TARGETS test_tracking
  DESTINATION lib/${PROJECT_NAME})
endif()

ament_export_include_directories(${PCL_INCLUDE_DIRS})
ament_export_libraries(${PCL_LIBRARIES})

ament_package()