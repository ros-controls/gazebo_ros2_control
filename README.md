# gazebo_ros2_control


This is a ROS 2 package for integrating the `ros2_control` controller architecture with the [Gazebo](http://gazebosim.org/) simulator.

This package provides a Gazebo plugin which instantiates a `ros2_control` controller manager and connects it to a Gazebo model.

# Usage

This repository contains the contents for testing gazebo_ros2_control

It is running Gazebo and some other ROS 2 nodes.

## Video + Pictures

![](img/gazebo_ros2_control_position.gif)

## Running

### Modifying or building your own

```bash
cd Docker
docker build -t gazebo_ros2_control .
```

### To run the demo

#### Using Docker

Docker allows us to run the demo without GUI if we don't configure it properly. The following command runs the demo without GUI:

```bash
docker run -it --rm --name gazebo_ros2_control_demo --net host gazebo_ros2_control ros2 launch gazebo_ros2_control_demos cart_example_position.launch.py gui:=false
```

The in your local machine you can run the Gazebo client:

```bash
gzclient
```

#### Using Rocker

To run the demo with GUI we are going to use [rocker](https://github.com/osrf/rocker/) which is a tool to run docker
images with customized local support injected for things like nvidia support. And user id specific files for cleaner
mounting file permissions. You can install this tool with the following [instructions](https://github.com/osrf/rocker/#installation).

The following command will launch Gazebo:

```bash
rocker --x11 --nvidia --name gazebo_ros2_control_demo gazebo_ros2_control:latest
```

The following commands allow to move the cart in the rail:

```bash
docker exec -it gazebo_ros2_control_demo bash
source /home/ros2_ws/install/setup.bash
ros2 run gazebo_ros2_control_demos example_position
```


## Add transmission elements to a URDF

To use `ros2_control` with your robot, you need to add some additional elements to your URDF.
The `<transmission>` element is used to link actuators to joints, see the `<transmission>` spec for exact XML format.

For the purposes of `gazebo_ros2_control` in its current implementation, the only important information
in these transmission tags are:

 - `<joint name="">` the name must correspond to a joint else where in your URDF
 - `<type>` the type of transmission. Currently only `transmission_interface/SimpleTransmission` is implemented.
 - `<hardwareInterface>` within the `<actuator>` and `<joint>` tags, this tells the `gazebo_ros2_control` plugin what hardware interface to load (position, velocity or effort interfaces).

## Add the gazebo_ros2_control plugin

In addition to the transmission tags, a Gazebo plugin needs to be added to your URDF that
actually parses the transmission tags and loads the appropriate hardware interfaces and
controller manager. By default the `gazebo_ros2_control` plugin is very simple, though it is also
extensible via an additional plugin architecture to allow power users to create their own custom
robot hardware interfaces between `ros2_control` and Gazebo.

```xml
<gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
      <robot_sim_type>gazebo_ros2_control/GazeboSystem</robot_sim_type>
      <robot_param>robot_description</robot_param>
      <robot_param_node>robot_state_publisher</robot_param_node>
      <parameters>$(find gazebo_ros2_control_demos)/config/cartpole_controller.yaml</parameters>
      <e_stop_topic>False</e_stop_topic>
      <control_period>0.01</control_period>
    </plugin>
</gazebo>
```

The `gazebo_ros2_control` `<plugin>` tag also has the following optional child elements:

 - `<control_period>`: The period of the controller update (in seconds), defaults to Gazebo's period
 - `<robot_param>`: The location of the `robot_description` (URDF) on the parameter server, defaults to `robot_description`
 - `<robot_param_node>`: Name of the node where the `robot_param` is located, defauls to `robot_state_publisher`
 - `<robot_sim_type>`: The pluginlib name of a custom robot ros2_control system plugin to be used, defaults to `gazebo_ros2_control/GazeboSystem`
 - `<parameters>`: YAML file with the configuration of the controllers
 - `<e_stop_topic>`: Topic to publish the emergency stop

#### Default gazebo_ros2_control Behavior

By default, without a `<robot_sim_type>` tag, `gazebo_ros2_control` will attempt to get all of the information it needs to interface with a ros2_control-based controller out of the URDF. This is sufficient for most cases, and good for at least getting started.

The default behavior provides the following ros2_control interfaces:

 - hardware_interface::JointStateInterface
 - hardware_interface::EffortJointInterface
 - hardware_interface::VelocityJointInterface

#### Advanced: custom gazebo_ros2_control Simulation Plugins

The `gazebo_ros2_control` Gazebo plugin also provides a pluginlib-based interface to implement custom interfaces between Gazebo and `ros2_control` for simulating more complex mechanisms (nonlinear springs, linkages, etc).

These plugins must inherit `gazebo_ros2_control::GazeboSystemInterface` which implements a simulated `ros2_control`
`hardware_interface::SystemInterface`. SystemInterface provides API-level access to read and command joint properties.

The respective GazeboSystemInterface sub-class is specified in a URDF model and is loaded when the
robot model is loaded. For example, the following XML will load the default plugin
(same behavior as when using no `<robot_sim_type>` tag):

```xml
<gazebo>
  <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
    <robot_sim_type>gazebo_ros2_control/GazeboSystem</robot_sim_type>
  </plugin>
</gazebo>
```

#### Set up controllers

Use the tag `<parameters>` inside `<plugin>` to set the YAML file with the controller configuration.

```xml
<gazebo>
  <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
    <parameters>$(find gazebo_ros2_control_demos)/config/cartpole_controller.yaml</parameters>
  </plugin>
<gazebo>
```

This controller publishes the state of all resources registered to a
`hardware_interface::StateInterface` to a topic of type `sensor_msgs/msg/JointState`.
The following is a basic configuration of the controller.

```yaml
joint_state_controller:
  ros__parameters:
    type: joint_state_controller/JointStateController
```

This controller creates an action called `/cart_pole_controller/follow_joint_trajectory` of type `control_msgs::action::FollowJointTrajectory`.

```yaml
cart_pole_controller:
  ros__parameters:
    type: joint_trajectory_controller/JointTrajectoryController
    joints:
       - slider_to_cart
    write_op_modes:
       - slider_to_cart
    state_publish_rate: 25
    action_monitor_rate: 20
    constraints:
      stopped_velocity_tolerance: 0.05
      goal_time: 5
```
#### Executing the examples

There are some examples in the `gazebo_ros2_control_demos` package. These examples allow to launch a cart in a 30 meter rail.

![](img/cart.gif)

You can run some of the configuration running the following commands:

```bash
ros2 launch gazebo_ros2_control_demos cart_example_position.launch.py
ros2 launch gazebo_ros2_control_demos cart_example_velocity.launch.py
ros2 launch gazebo_ros2_control_demos cart_example_effort.launch.py
```

Send example commands:

When the Gazebo world is launched you can run some of the following commads to move the cart.

```bash
ros2 run gazebo_ros2_control_demos example_position
ros2 run gazebo_ros2_control_demos example_velocity
ros2 run gazebo_ros2_control_demos example_effort
```

#### Gazebo + Moveit2 + ROS 2

This example works with [ROS 2 Foxy](https://index.ros.org/doc/ros2/Installation/Foxy/).
You should install Moveit2 from sources, the instructions are available in this [link](https://moveit.ros.org/install-moveit2/source/).

The repository with all the required packages are in the [gazebo_ros_demos](https://github.com/ros-simulation/gazebo_ros_demos/tree/ahcorde/port/ros2).

```bash
ros2 launch rrbot_moveit_demo_nodes rrbot_demo.launch.py
```

![](img/moveit2.gif)
