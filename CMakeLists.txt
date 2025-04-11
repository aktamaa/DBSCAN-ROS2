cmake_minimum_required(VERSION 3.8)
project(dbscan_ros)
set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Add your executable
add_executable(dbscan_node
  src/dbscan_node.cpp
  src/dbscan.cpp
)

# Include directories
target_include_directories(dbscan_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

# Link PCL libraries
target_link_libraries(dbscan_node ${PCL_LIBRARIES})

# Dependencies for ROS2 and PCL
ament_target_dependencies(dbscan_node
  rclcpp
  sensor_msgs
  pcl_conversions
)

# Install target
install(TARGETS
  dbscan_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

