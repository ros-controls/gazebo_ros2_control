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
    && git clone https://github.com/ros-simulation/gazebo_ros2_control \
    && git clone https://github.com/ros-controls/ros2_control \
    && git clone https://github.com/ros-controls/ros2_controllers \
    && rosdep update \
    && rosdep install --from-paths ./ -i -y --rosdistro foxy \
      --ignore-src

RUN cd /home/ros2_ws/ \
  && . /opt/ros/foxy/setup.sh \
  && export CMAKE_PREFIX_PATH=$AMENT_PREFIX_PATH:$CMAKE_PREFIX_PATH \
  && colcon build

COPY entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

CMD ros2 launch gazebo_ros2_control_demos cart_example_position.launch.py
