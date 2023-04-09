cmake_minimum_required(VERSION 3.5)
project(scan_context)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fpermissive)
endif()

find_package(ament_cmake REQUIRED)
find_package(Protobuf REQUIRED)
find_package(nanoflann REQUIRED)
find_package(localization_common REQUIRED)
# find_package(backward_ros REQUIRED)

include_directories(include)
# to find *.bp.h files
include_directories(${CMAKE_CURRENT_BINARY_DIR})

# generate proto
protobuf_generate_cpp(PROTO_SRCS PROTO_HEADERS
  proto/key_frames.proto
  proto/ring_keys.proto
  proto/scan_contexts.proto)

# libaray
set(dependencies
  localization_common
  nanoflann
)

add_library(${PROJECT_NAME} SHARED
  ${PROTO_HEADERS}
  ${PROTO_SRCS}
  src/scan_context_manager.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME}
  ${PROTOBUF_LIBRARIES}
)

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

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
