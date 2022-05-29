cmake_minimum_required(VERSION 3.5)
project(perception)

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

find_package(ament_cmake REQUIRED)
find_package(costmap_base REQUIRED)

include_directories(
 include
 ${GLOG_INCLUDE_DIRS}
 ${costmap_base_INCLUDE_DIRS}
)

add_library(narrow_recognition src/narrow_recognition.cpp)
target_link_libraries(narrow_recognition 
    -lglog 
    ${costmap_2d_LIBRARIES}
)

install(TARGETS
  narrow_recognition
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(narrow_recognition)

ament_package()
