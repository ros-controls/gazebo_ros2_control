# gazebo_ros2_control

This is a ROS 2 package for integrating the `ros2_control` controller architecture with the [Gazebo](http://gazebosim.org/) simulator.

This package provides a Gazebo plugin which instantiates a `ros2_control` controller manager and connects it to a Gazebo model.

# Usage

## Add transmission elements to a URDF

To use `ros2_control` with your robot, you need to add some additional elements to your URDF. The ``<transmission>`` element is used to link actuators to joints, see the `<transmission>` spec for exact XML format.

For the purposes of `gazebo_ros2_control` in its current implementation, the only important information in these transmission tags are:

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
      <robotSimType>gazebo_ros2_control/DefaultRobotHWSim</robotSimType>
      <robotParamNode>robot_state_publisher</robotParamNode>
      <legacyModeNS>True</legacyModeNS>
      <parameters>$(find gazebo_ros2_control_demos)/config/cartpole_controller.yaml</parameters>
    </plugin>
</gazebo>
```

The `gazebo_ros2_control` `<plugin>` tag also has the following optional child elements:

 - `<robotNamespace>`: The ROS namespace to be used for this instance of the plugin, defaults to robot name in URDF/SDF
 - `<controlPeriod>`: The period of the controller update (in seconds), defaults to Gazebo's period
 - `<robotParam>`: The location of the robot_description (URDF) on the parameter server, defaults to 'robot_description'
 - `<robotParam>`: Name of the node where the robotParam is located, defauls to `robot_state_publisher`
 - `<robotSimType>`: The pluginlib name of a custom robot sim interface to be used, defaults to 'DefaultRobotHWSim'
 - `<parameters>`: YAML file with the configuration of the controllers

#### Default gazebo_ros2_control Behavior

By default, without a `<robotSimType>` tag, `gazebo_ros2_control` will attempt to get all of the information it needs to interface with a ros2_control-based controller out of the URDF. This is sufficient for most cases, and good for at least getting started.

The default behavior provides the following ros2_control interfaces:

 - hardware_interface::JointStateInterface
 - hardware_interface::EffortJointInterface
 - hardware_interface::VelocityJointInterface

#### Advanced: custom gazebo_ros2_control Simulation Plugins

The `gazebo_ros2_control` Gazebo plugin also provides a pluginlib-based interface to implement custom interfaces between Gazebo and `ros2_control` for simulating more complex mechanisms (nonlinear springs, linkages, etc).

These plugins must inherit `gazebo_ros2_control::RobotHWSim` which implements a simulated `ros2_control` `hardware_interface::RobotHW`. RobotHWSim provides API-level access to read and command joint properties in the Gazebo simulator.

The respective RobotHWSim sub-class is specified in a URDF model and is loaded when the robot model is loaded. For example, the following XML will load the default plugin (same behavior as when using no `<robotSimType>` tag):

```xml
<gazebo>
  <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
    <robotNamespace></robotNamespace>
    <robotSimType>gazebo_ros2_control/DefaultRobotHWSim</robotSimType>
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
`hardware_interface::JointStateInterface` to a topic of type `sensor_msgs/JointState`. The following is a basic configuration of the controller.

```yaml
joint_state_controller:
  ros__parameters:
    type: joint_state_controller/JointStateController
    publish_rate: 50
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

#### Setting PID gains

To set the PID gains for a specific joint you need to define them inside `<plugin><ros></plugin></ros>`. Using the generic way of defining parameters with `gazebo_ros`. The name of the parameter correspond the name of the joint followed by a dot and the name of the parameter: `p`, `i`, `d`, `i_clamp_max`, `i_clamp_min` and `antiwindup`.

```xml
<gazebo>
  <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
    <ros>
      <namespace>/</namespace>
      <parameter name="slider_to_cart.p" type="double">50.0</parameter>
      <parameter name="slider_to_cart.i" type="double">10.0</parameter>
      <parameter name="slider_to_cart.d" type="double">15.0</parameter>
      <parameter name="slider_to_cart.i_clamp_max" type="double">3.0</parameter>
      <parameter name="slider_to_cart.i_clamp_min" type="double">-3.0</parameter>
      <parameter name="slider_to_cart.antiwindup" type="bool">false</parameter>
    </ros>
    ...
  </plugins>
</gazebo>
```

#### Executing the examples

There are some examples in the `gazebo_ros2_control_demos` package. These examples allow to launch a cart in a 30 meter rail.

![](img/cart.gif)

You can run some of the configuration running the following commands:

```bash
ros2 launch gazebo_ros2_control_demos cart_example_position_pid.launch.py
ros2 launch gazebo_ros2_control_demos cart_example_position.launch.py
ros2 launch gazebo_ros2_control_demos cart_example_velocity.launch.py
```

Send example commands:

When the Gazebo world is launched you can run some of the following commads to move the cart.

```bash
ros2 run gazebo_ros2_control_demos example_position
ros2 run gazebo_ros2_control_demos example_velocity
```

To get or modify the values of the PID controller you can run the following commands:

```bash
ros2 param get /gazebo_ros2_control slider_to_cart.p
ros2 param get /gazebo_ros2_control slider_to_cart.i
ros2 param get /gazebo_ros2_control slider_to_cart.d
ros2 param get /gazebo_ros2_control slider_to_cart.i_clamp_max
ros2 param get /gazebo_ros2_control slider_to_cart.i_clamp_min
```

```bash
ros2 param set /gazebo_ros2_control slider_to_cart.p 50.0
ros2 param set /gazebo_ros2_control slider_to_cart.i 10.0
ros2 param set /gazebo_ros2_control slider_to_cart.d 15.0
ros2 param set /gazebo_ros2_control slider_to_cart.i_clamp_max 3.0
ros2 param set /gazebo_ros2_control slider_to_cart.i_clamp_min -3.0
```

#### Dependencies

 - gazebo_ros_pkgs
 - realtime_tools
 - ros2_control
 - ros2_controllers
 - xacro

Step to compile `gazebo_ros2_control`

```bash
mkdir -p ~/ros2_control_ws/src
cd ~/ros2_control_ws/src
git clone https://github.com/ros-simulation/gazebo_ros_pkgs -b ros2
git clone https://github.com/ros-controls/realtime_tools -b ros2_devel
git clone https://github.com/ros-controls/ros2_control
git clone https://github.com/ros-controls/ros2_controllers
git clone https://github.com/ddengster/ros2_control/ -b coffeebot_deps ros2_control_ddengster
touch ros2_control_ddengster/COLCON_IGNORE
cp -r ros2_control_ddengster/joint_limits_interface ros2_control_ddengster/transmission_interface  ros2_control
git clone https://github.com/ros/xacro -b dashing-devel
cd ~/ros2_control_ws
colcon build --merge-install
```
