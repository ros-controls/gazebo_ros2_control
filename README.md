# gazebo_ros2_control

This is a ROS 2 package for integrating the `ros2_control` controller architecture with the [Gazebo Classic](https://classic.gazebosim.org/) simulator.

> Gazebo Classic goes end-of-life in January of 2025. We strongly recommend all users migrate from Gazebo Classic (numbered releases) to modern Gazebo (formerly known as Ignition 3, lettered releases) before January 2025.
>
> Furthermore, Gazebo Classic is not released to Ubuntu Noble. As a consequence, gazebo_ros2_control won't be released for Jazzy and Rolling anymore.

To use ros2_control with newer versions of Gazebo take a look at [gz_ros2_control](https://github.com/ros-controls/gz_ros2_control).

This package provides a Gazebo plugin which instantiates a `ros2_control` controller manager and connects it to a Gazebo model.

## Documentation
See the [documentation file](doc/index.rst) or [control.ros.org](https://control.ros.org/master/doc/gazebo_ros2_control/doc/index.html).

## Build status

ROS 2 Distro | Branch | Build status | Documentation
:----------: | :----: | :----------: | :-----------:
**Rolling** | [`master`](https://github.com/ros-controls/gazebo_ros2_control/tree/master) | n/a | [Documentation](https://control.ros.org/master/doc/gazebo_ros2_control/doc/index.html)
**Jazzy** | [`master`](https://github.com/ros-controls/gazebo_ros2_control/tree/master) | n/a | [Documentation](https://control.ros.org/jazzy/doc/gazebo_ros2_control/doc/index.html)
**Iron** | [`iron`](https://github.com/ros-controls/gazebo_ros2_control/tree/iron) | [![Gazebo ros2 control CI](https://github.com/ros-controls/gazebo_ros2_control/actions/workflows/ci-iron.yaml/badge.svg?branch=iron)](https://github.com/ros-controls/gazebo_ros2_control/actions/workflows/ci-iron.yaml) | [Documentation](https://control.ros.org/iron/doc/gazebo_ros2_control/doc/index.html)
**Humble** | [`humble`](https://github.com/ros-controls/gazebo_ros2_control/tree/humble) | [![Gazebo ros2 control CI](https://github.com/ros-controls/gazebo_ros2_control/actions/workflows/ci-humble.yaml/badge.svg?branch=humble)](https://github.com/ros-controls/gazebo_ros2_control/actions/workflows/ci-humble.yaml) | [Documentation](https://control.ros.org/humble/doc/gazebo_ros2_control/doc/index.html)
