cmake_minimum_required(VERSION 3.5)

project(planner_utils)

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


include_directories(
 include
 ${GLOG_INCLUDE_DIRS}
 ${pose2d_INCLUDE_DIRS}
)


add_library(planner_utils
  src/path.cpp
  src/navigation_mediator.cpp 
  src/robot_info.cpp 
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

ament_package()

