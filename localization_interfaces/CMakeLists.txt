cmake_minimum_required(VERSION 3.5)
project(localization_interfaces)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GnssData.msg"
  "msg/LidarFrame.msg"
  "msg/LidarFrames.msg"
  "msg/LoopCandidate.msg"
  "srv/OptimizeMap.srv"
  "srv/SaveMap.srv"
  "srv/SaveOdometry.srv"
  "srv/SaveScanContext.srv"
DEPENDENCIES
  builtin_interfaces
  geometry_msgs
  sensor_msgs
  nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
