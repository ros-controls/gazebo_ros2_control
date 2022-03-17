cmake_minimum_required(VERSION 3.5.0)
project(gazebo_ros2_control)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(controller_manager REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wconversion -Wno-sign-conversion -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

include_directories(include)
link_directories(
  ${gazebo_dev_LIBRARY_DIRS}
)

# Libraries
add_library(${PROJECT_NAME} SHARED
  src/gazebo_ros2_control_plugin.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  angles
  controller_manager
  gazebo_dev
  gazebo_ros
  hardware_interface
  pluginlib
  rclcpp
  urdf
  yaml_cpp_vendor
)

add_library(gazebo_hardware_plugins SHARED
  src/gazebo_system.cpp
)
ament_target_dependencies(gazebo_hardware_plugins
  angles
  gazebo_dev
  hardware_interface
  rclcpp
)

## Install
install(TARGETS
  ${PROJECT_NAME}
  gazebo_hardware_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(
  include
)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_libraries(
  ${PROJECT_NAME}
  gazebo_hardware_plugins
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(gazebo_ros2_control gazebo_hardware_plugins.xml)

ament_package()
