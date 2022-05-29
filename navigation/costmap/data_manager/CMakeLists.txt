cmake_minimum_required(VERSION 3.5)
project(data_manager)

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

find_package(costmap_base REQUIRED)
find_package(costmap_mediator REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pose2d REQUIRED)

# INCLUDES and LIBS
include_directories(
  include
)

add_library(${PROJECT_NAME}
  src/data_register.cpp
  src/processor_utils.cpp
  src/perception_map.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  costmap_base
  costmap_mediator
  OpenCV
  pose2d
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

ament_export_include_directories(include)

ament_export_libraries(${PROJECT_NAME})

ament_export_dependencies(costmap_base)
ament_export_dependencies(pose2d)
ament_export_dependencies(OpenCV)
ament_export_dependencies(costmap_mediator)
ament_package()