cmake_minimum_required(VERSION 3.5)
project(localization_common)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -Wall -Wextra -Wpedantic -fpermissive)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(localization_interfaces REQUIRED)
find_package(ndt_omp_ros2 REQUIRED)
#
find_package(yaml-cpp REQUIRED)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)

include_directories(include)

# libaray
set(dependencies
  rclcpp
  pcl_ros
  tf2_ros
  tf2_eigen
  geometry_msgs
  sensor_msgs
  nav_msgs
  localization_interfaces
  ndt_omp_ros2
)
add_library(${PROJECT_NAME} SHARED
  # sensor_data
  src/sensor_data/gnss_data.cpp
  src/sensor_data/key_frame.cpp
  src/sensor_data/loop_pose.cpp
  src/sensor_data/pose_data.cpp
  src/sensor_data/velocity_data.cpp
  # publisher
  src/publisher/cloud_publisher.cpp
  src/publisher/imu_publisher.cpp
  src/publisher/key_frame_publisher.cpp
  src/publisher/key_frames_publisher.cpp
  src/publisher/loop_pose_publisher.cpp
  src/publisher/odometry_publisher.cpp
  src/publisher/pos_vel_publisher.cpp
  # subscriber
  src/subscriber/cloud_subscriber.cpp
  src/subscriber/gnss_subscriber.cpp
  src/subscriber/imu_subscriber.cpp
  src/subscriber/key_frame_subscriber.cpp
  src/subscriber/key_frames_subscriber.cpp
  src/subscriber/loop_pose_subscriber.cpp
  src/subscriber/odometry_subscriber.cpp
  src/subscriber/pos_vel_subscriber.cpp
  src/subscriber/velocity_subscriber.cpp
  # cloud_filter
  src/cloud_filter/box_filter.cpp
  src/cloud_filter/no_filter.cpp
  src/cloud_filter/voxel_filter.cpp
  src/cloud_filter/cloud_filter_factory.cpp
  # registration
  src/registration/icp_registration.cpp
  src/registration/icp_svd_registration.cpp
  src/registration/ndt_registration.cpp
  src/registration/ndt_omp_registration.cpp
  src/registration/registration_factory.cpp
  # distortion adjust
  src/distortion_adjust.cpp
  # utils
  src/tf_utils.cpp
  src/data_synchronization.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME}
  ${GeographicLib_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

# hello_kitti_node
add_executable(hello_kitti_node
  src/hello_kitti_main.cpp)
target_link_libraries(hello_kitti_node ${PROJECT_NAME})

# kitti_preprocess_node
add_executable(kitti_preprocess_node
  src/kitti_preprocess_node.cpp
  src/kitti_preprocess_main.cpp)

target_link_libraries(kitti_preprocess_node ${PROJECT_NAME})

# Install include directories
install(DIRECTORY include/
  DESTINATION include
)

# Install libraries
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install executables
install(TARGETS
    hello_kitti_node
    kitti_preprocess_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
