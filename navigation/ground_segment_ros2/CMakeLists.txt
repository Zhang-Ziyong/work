cmake_minimum_required(VERSION 3.5)
project(ground_segment_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
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
# Find Packages
find_package(Boost COMPONENTS system REQUIRED)
find_package(PCL  REQUIRED )
find_package(log REQUIRED)

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
#add_executable(groundfit_node
#    src/ground_node.cpp
#    src/point_cloud_filter.cpp
#    src/recon_cluster.cpp
#)
#
#ament_target_dependencies(groundfit_node
#  "rclcpp"
#  "rcutils"
#  "sensor_msgs"
#  "std_msgs"
#  "visualization_msgs"
#)
#
#target_link_libraries(groundfit_node
#  ${Boost_SYSTEM_LIBRARY}
#  ${PCL_LIBRARIES}
#  -lglog
#  -lrt
#)

## add lidar project here
add_executable(ground_segment_node
  src/segment_node.cpp
  src/voxel_grid_filter.cpp
)

ament_target_dependencies(ground_segment_node
  "rclcpp"
  "rcutils"
  "sensor_msgs"
  "std_msgs"
  "log"
)

target_link_libraries(ground_segment_node
  ${PCL_LIBRARIES}
  -lglog
  -lrt
)

install(TARGETS 
  #groundfit_node 
  ground_segment_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY params/
  DESTINATION params/)

ament_package()
