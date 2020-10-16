cmake_minimum_required(VERSION 3.5.0)
project(gazebo_ros2_control)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(urdf REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(joint_limits_interface REQUIRED)
find_package(transmission_interface REQUIRED)
find_package(controller_manager REQUIRED)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

include_directories(include)

# Libraries
add_library(${PROJECT_NAME} SHARED src/gazebo_ros2_control_plugin.cpp)
ament_target_dependencies(${PROJECT_NAME}
  controller_manager
  gazebo_dev
  hardware_interface
  pluginlib
  rclcpp
  transmission_interface
  urdf
  yaml_cpp_vendor
)

add_library(default_robot_hw_sim SHARED
        include/gazebo_ros2_control/default_robot_hw_sim.h
        include/gazebo_ros2_control/robot_hw_sim.h
        src/default_robot_hw_sim.cpp)
ament_target_dependencies(default_robot_hw_sim
  gazebo_dev
  rclcpp
  joint_limits_interface
  transmission_interface
  hardware_interface
  angles
)

## Install
install(TARGETS default_robot_hw_sim ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_libraries(${PROJECT_NAME} default_robot_hw_sim)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(gazebo_ros2_control robot_hw_sim_plugins.xml)

ament_package()
