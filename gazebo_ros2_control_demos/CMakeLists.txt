cmake_minimum_required(VERSION 3.5.0)
project(gazebo_ros2_control_demos)

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

install(DIRECTORY
  launch
  config
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

add_executable(example_position examples/example_position.cpp)
ament_target_dependencies(example_position
  rclcpp
  rclcpp_action
  control_msgs
)

add_executable(example_velocity examples/example_velocity.cpp)
ament_target_dependencies(example_velocity
  rclcpp
  rclcpp_action
  control_msgs
)

## Install
install(TARGETS example_position
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
