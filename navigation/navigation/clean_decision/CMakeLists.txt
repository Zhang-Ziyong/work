cmake_minimum_required(VERSION 3.5)
project(clean_decision)

# Default to C99
# if(NOT CMAKE_C_STANDARD)
#   set(CMAKE_C_STANDARD 99)
# endif()

# # Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
# endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3)
find_package(rclcpp REQUIRED)
find_package(log REQUIRED)
find_package(SDL REQUIRED)
find_package(SDL_image REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(full_coverage_planner REQUIRED)
find_package(planner_utils REQUIRED)
find_package(pose2d REQUIRED)

include_directories(
include
config
${SDL_INCLUDE_DIRS}
${SDL_IMAGE_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
src/data_base/obstacle_map.cpp 
src/clean_decision.cpp 
)

target_link_libraries(${PROJECT_NAME}
-lglog 
jsoncpp   
${SDL_LIBRARY}
${SDL_IMAGE_LIBRARIES}
${yaml_cpp_vendor_LIBRARIES})

ament_target_dependencies(${PROJECT_NAME}
"rclcpp"
"log"
"full_coverage_planner"
"planner_utils"
"pose2d"
)

# add_executable(map_test test/map_test.cpp)

# target_link_libraries(map_test 
# ${PROJECT_NAME})

install(
    DIRECTORY include/ config/
    DESTINATION include/
  )
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# install(TARGETS map_test
#   DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include ${full_coverage_planner_INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(yaml_cpp_vendor SDL SDL_image)

ament_package()
