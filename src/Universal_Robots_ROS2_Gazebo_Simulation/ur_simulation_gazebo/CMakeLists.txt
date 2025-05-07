cmake_minimum_required(VERSION 3.5)
project(ur_simulation_gazebo)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED COMPONENTS common visualization)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(VTK REQUIRED)

# Include directories
include_directories(
  ${PCL_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
  include
)

# Link directories
link_directories(
  ${PCL_LIBRARY_DIRS}
  ${VTK_LIBRARY_DIRS}
)

# Add definitions
add_definitions(
  ${PCL_DEFINITIONS}
  ${VTK_DEFINITIONS}
)

# Build test_pcl executable
add_executable(test_pcl src/test_pcl.cpp)
target_link_libraries(test_pcl
  ${PCL_LIBRARIES}
  ${VTK_LIBRARIES}
)
ament_target_dependencies(test_pcl
  rclcpp
  sensor_msgs
  pcl_conversions
)

# Install targets
install(TARGETS test_pcl
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()