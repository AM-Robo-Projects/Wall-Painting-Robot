cmake_minimum_required(VERSION 3.5)
project(ur_simulation_gazebo)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED COMPONENTS common visualization filters segmentation kdtree search features)
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
add_executable(voxelizing src/voxelizing.cpp)
add_executable(planner_seg src/planner_seg.cpp)
add_executable(clustering src/clustering.cpp)

target_link_libraries(test_pcl
  ${PCL_LIBRARIES}
  ${VTK_LIBRARIES}
)
ament_target_dependencies(test_pcl
  rclcpp
  sensor_msgs
  pcl_conversions
)

target_link_libraries(voxelizing
  ${PCL_LIBRARIES}
  ${VTK_LIBRARIES}
  pcl_common
  pcl_filters 
)

ament_target_dependencies(voxelizing
  rclcpp
  sensor_msgs
  pcl_conversions
)

target_link_libraries(planner_seg
  ${PCL_LIBRARIES}
  ${VTK_LIBRARIES}
  pcl_common
  pcl_filters 
  pcl_segmentation
  pcl_kdtree
  pcl_search  
)

target_link_libraries(clustering
  ${PCL_LIBRARIES}
  ${VTK_LIBRARIES}
  pcl_common
  pcl_filters 
  pcl_segmentation
  pcl_kdtree
  pcl_search  
  pcl_features
)

ament_target_dependencies(clustering
  rclcpp
  sensor_msgs
  pcl_conversions
  geometry_msgs
  std_msgs
  
)

# Install targets
install(TARGETS test_pcl
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS voxelizing 
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS planner_seg
  DESTINATION lib/${PROJECT_NAME}
)


install(TARGETS clustering
  DESTINATION lib/${PROJECT_NAME}
)
# Install directories
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake)
  find_package(ament_cmake_pytest REQUIRED)
  add_launch_test(test/test_gazebo.py
    TIMEOUT
      180
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()