# Copyright 2024 Open Source Robotics Foundation, Inc.
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
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
             )

    gazebo_ros2_control_demos_path = os.path.join(
        get_package_share_directory('gazebo_ros2_control_demos'))

    xacro_file = os.path.join(gazebo_ros2_control_demos_path,
                              'urdf',
                              'test_diff_drive_namespaced.xacro.urdf')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    node_robot_state_publisher_r1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='r1',
        output='screen',
        parameters=[params]
    )

    node_robot_state_publisher_r2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='r2',
        output='screen',
        parameters=[params]
    )

    spawn_entity_r1 = Node(
      package='gazebo_ros', executable='spawn_entity.py',
      arguments=[
                '-topic', '/r1/robot_description',
                '-robot_namespace', 'r1',
                '-entity', 'diffbot1'
                ],
      output='screen'
    )

    spawn_entity_r2 = Node(
      package='gazebo_ros', executable='spawn_entity.py',
      arguments=[
                '-topic', '/r2/robot_description',
                '-robot_namespace', 'r2',
                '-entity', 'diffbot1'
                ],
      output='screen'
    )

    load_joint_state_broadcaster_r1 = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster',
             '-c', '/r1/controller_manager'],
        output='screen'
    )

    load_diff_drive_base_controller_r1 = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_drive_base_controller',
             '-c', '/r1/controller_manager'],
        output='screen'
    )

    load_joint_state_broadcaster_r2 = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster',
             '-c', '/r2/controller_manager'],
        output='screen'
    )

    load_diff_drive_base_controller_r2 = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_drive_base_controller',
             '-c', '/r2/controller_manager'],
        output='screen'
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                  target_action=spawn_entity_r1,
                  on_exit=[
                            load_joint_state_broadcaster_r1,
                            load_diff_drive_base_controller_r1
                          ],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                  target_action=spawn_entity_r2,
                  on_exit=[
                            load_joint_state_broadcaster_r2,
                            load_diff_drive_base_controller_r2
                          ],
            )
        ),
        gazebo,
        node_robot_state_publisher_r1,
        node_robot_state_publisher_r2,
        spawn_entity_r1,
        spawn_entity_r2,
    ])
