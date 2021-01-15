FROM osrf/ros:foxy-desktop

# Make sure everything is up to date before building from source
RUN apt-get update \
  && apt-get upgrade -y \
  && apt-get clean

RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    lsb-release \
    wget \
    python3-colcon-ros \
    && apt-get clean

RUN mkdir -p /home/ros2_ws/src \
    && cd /home/ros2_ws/src/ \
    # && wget https://raw.githubusercontent.com/ros-planning/moveit2/main/moveit2.repos \
    # && vcs import < moveit2.repos \
    && git clone https://github.com/ros-planning/moveit2 \
    && git clone https://github.com/ros-planning/moveit_msgs -b ros2 \
    && git clone https://github.com/ros-planning/moveit_resources -b ros2 \
    && git clone https://github.com/ros-planning/geometric_shapes -b ros2 \
    && git clone https://github.com/ros-planning/srdfdom -b ros2 \
    && git clone https://github.com/ros-planning/warehouse_ros -b ros2 \
    && git clone https://github.com/ros-planning/warehouse_ros_mongo -b ros2 \
    && git clone https://github.com/ros-simulation/gazebo_ros2_control -b ahcorde/update/foxy \
    && git clone https://github.com/ros-controls/ros2_control \
    && git clone https://github.com/ros-controls/ros2_controllers \
    && git clone https://github.com/ros-simulation/gazebo_ros_demos -b ahcorde/port/ros2 \
    && rosdep update \
    && rosdep install --from-paths ./ -i -y --rosdistro foxy \
      --ignore-src

RUN cd /home/ros2_ws/ \
  && . /opt/ros/foxy/setup.sh \
  && colcon build --merge-install

COPY entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

CMD ros2 launch rrbot_moveit_demo_nodes rrbot_demo.launch.py
