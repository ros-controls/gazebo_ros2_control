FROM osrf/ros:foxy-desktop

# Make sure everything is up to date before building from source
RUN apt-get update \
  && apt-get upgrade -y \
  && apt-get clean

RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    lsb-release \
    python3-colcon-ros \
    && apt-get clean

RUN mkdir -p /home/ros2_ws/src \
    && cd /home/ros2_ws/src/ \
    && git clone https://github.com/ros-simulation/gazebo_ros2_control -b ahcorde/update/ros2_control \
    && git clone https://github.com/ros-simulation/gazebo_ros2_control -b ahcorde/initial/demos gazebo_ros2_control_demos \
    && touch gazebo_ros2_control_demos/gazebo_ros2_control/COLCON_IGNORE \
    && git clone https://github.com/ros-controls/realtime_tools -b ros2_devel \
    && git clone https://github.com/ros-controls/ros2_control \
    && git clone https://github.com/ros-controls/ros2_controllers \
    && git clone https://github.com/ddengster/ros2_control/ -b coffeebot_deps ros2_control_ddengster \
    && touch ros2_control_ddengster/COLCON_IGNORE \
    && cp -r ros2_control_ddengster/joint_limits_interface ros2_control_ddengster/transmission_interface ros2_control \
    && git clone https://github.com/ros/xacro -b dashing-devel \
    && sed -i '/rclcpp/d' ros2_control/joint_limits_interface/CMakeLists.txt \
    && sed -i '/rclcpp/d' ros2_control/joint_limits_interface/package.xml \
    && rosdep update \
    && rosdep install --from-paths ./ -i -y --rosdistro foxy \
      --skip-keys=controller_interface \
      --skip-keys=controller_manager \
      --skip-keys=controller_parameter_server \
      --skip-keys=hardware_interface \
      --skip-keys=joint_limits_interface \
      --skip-keys=joint_state_controller \
      --skip-keys=joint_trajectory_controller \
      --skip-keys=realtime_tools \
      --skip-keys=transmission_interface \
      --skip-keys=xacro

RUN cd /home/ros2_ws/ \
  && . /opt/ros/foxy/setup.sh \
  && export CMAKE_PREFIX_PATH=$AMENT_PREFIX_PATH:$CMAKE_PREFIX_PATH \
  && colcon build

COPY entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

CMD ros2 launch gazebo_ros2_control_demos cart_example_position.launch.py
