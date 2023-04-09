cmake_minimum_required(VERSION 3.5)
project(lidar_mapping)

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

find_package(g2o REQUIRED
  COMPONENTS
  g2o::core
  g2o::types_slam3d
  g2o::solver_cholmod
  g2o::solver_csparse
  g2o::solver_pcg
)
find_package(Sophus REQUIRED)
# find_package(backward_ros REQUIRED)

include_directories(include)

set(dependencies
  rclcpp
  localization_interfaces
  localization_common
  scan_context
)

# lidar_mapping libaray
add_library(${PROJECT_NAME} SHARED
  src/graph_optimizer/g2o_graph_optimizer.cpp
  src/map_generator.cpp
  src/front_end.cpp
  src/back_end.cpp
  src/loop_closure.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME}
  g2o::core
  g2o::types_slam3d
  g2o::solver_cholmod
  g2o::solver_csparse
  g2o::solver_pcg
  fmt
)

# front_end_node
add_executable(mapping_front_end_node
  src/front_end_node.cpp
  src/front_end_main.cpp)
target_link_libraries(mapping_front_end_node ${PROJECT_NAME})

# back_end_node
add_executable(mapping_back_end_node
  src/back_end_node.cpp
  src/back_end_main.cpp)
target_link_libraries(mapping_back_end_node ${PROJECT_NAME})

# loop_closure_node
add_executable(mapping_loop_closure_node
  src/loop_closure_node.cpp
  src/loop_closure_main.cpp)
target_link_libraries(mapping_loop_closure_node ${PROJECT_NAME})

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

# install
install(TARGETS
    mapping_front_end_node
    mapping_back_end_node
    mapping_loop_closure_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

# Install  directories
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
