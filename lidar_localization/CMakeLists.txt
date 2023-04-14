cmake_minimum_required(VERSION 3.5)
project(lidar_localization)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fpermissive)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(localization_interfaces REQUIRED)
find_package(localization_common REQUIRED)
find_package(scan_context REQUIRED)

include_directories(include)

set(dependencies
  rclcpp
  localization_interfaces
  localization_common
  scan_context
)

# matching_node
add_executable(matching_node
  src/matching.cpp
  src/matching_node.cpp
  src/matching_main.cpp)

ament_target_dependencies(matching_node ${dependencies})

# install executable
install(TARGETS
    matching_node
  DESTINATION lib/${PROJECT_NAME}
)

# install directories
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
