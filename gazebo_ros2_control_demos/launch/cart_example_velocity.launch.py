# Copyright 2020 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command

from launch_ros.actions import Node


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
             )

    gazebo_ros2_control_demos_path = os.path.join(
        get_package_share_directory('gazebo_ros2_control_demos'))

    xacro_file = os.path.join(gazebo_ros2_control_demos_path, 'urdf', 'test_cart_velocity.xacro.urdf')
    config_file = os.path.join(gazebo_ros2_control_demos_path, 'config', 'cartpole_controller.yaml')

    robot_desc = Command('xacro %s' % xacro_file)
    params = {'robot_description': robot_desc}

    context = LaunchContext()
    command = Command('xacro %s -o /tmp/test_cart_velocity.urdf' % xacro_file)
    command.perform(context)

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # GAZEBO_MODEL_PATH has to be correctly set for Gazebo to be able to find the model
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-file', '/tmp/test_cart_velocity.urdf', '-entity', 'cartpole'],
                        output='screen')

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
    ])
