cmake_minimum_required(VERSION 3.5)

project(costmap_mediator)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

find_package(ament_cmake_auto REQUIRED COMPONENTS)
find_package(costmap_base REQUIRED)

include_directories(
 include
)

add_library(${PROJECT_NAME}
  src/costmap_mediator.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  costmap_base
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
if(false)
  find_package(GTest REQUIRED)
  include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

  add_executable(test_costmap_mediator test/test_costmap_mediator.cpp)
  target_link_libraries(test_costmap_mediator ${PROJECT_NAME} -lgtest -lpthread -lglog)

  install(TARGETS test_costmap_mediator
  DESTINATION lib/${PROJECT_NAME})
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_export_dependencies(costmap_base)


ament_package()

