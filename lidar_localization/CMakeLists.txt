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

# lidar_localization_node
add_executable(lidar_localization_node
  src/lidar_localization.cpp
  src/lidar_localization_node.cpp
  src/lidar_localization_main.cpp)

ament_target_dependencies(lidar_localization_node ${dependencies})

# install executable
install(TARGETS
    lidar_localization_node
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
