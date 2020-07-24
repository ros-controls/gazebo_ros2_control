# gazebo_ros2_control

This repository contains the contents for testing gazebo_ros2_control

It is running Gazebo and some other ROS 2 nodes.

## Video + Pictures

![](img/gazebo_ros2_control_position_pid.gif)

## Running

### Modifying or building your own

```bash
cd Docker
docker build -t gazebo_ros2_control_docker .
```

### To run the demo

To run the demo we are going to use [rocker](https://github.com/osrf/rocker/) which is a tool to run docker images with customized local support injected for things like nvidia support. And user id specific files for cleaner mounting file permissions. You can install this tool with the following [instructions](https://github.com/osrf/rocker/#installation).

The following command will launch Gazebo:

```bash
rocker --x11 --nvidia --name gazebo_ros2_control_demo gazebo_ros2_control:latest
```

The following commands allow to move the cart in the rail:

```bash
docker exec gazebo_ros2_control_demo bash
ros2 run gazebo_ros2_control_demos example_position
```
