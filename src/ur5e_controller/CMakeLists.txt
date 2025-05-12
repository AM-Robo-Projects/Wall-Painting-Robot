cmake_minimum_required(VERSION 3.8)
project(ur5e_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Install Python scripts with executable permission
install(PROGRAMS
  scripts/auto_setup_ur.py
  scripts/collision_environment.py
  scripts/livox_converter.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY 
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files and config
install(DIRECTORY 
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install requirements.txt file
install(FILES
  requirements.txt
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
