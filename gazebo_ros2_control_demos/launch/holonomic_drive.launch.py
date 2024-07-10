import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    package_name='gazebo_ros2_control_demos' 

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory(package_name))
    xacro_file = os.path.join(pkg_path, 'urdf', 'test_holonomic_drive.xacro.urdf')
    robot_description_config = xacro.process_file(xacro_file)

    # Robot State Publisher
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': True}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Gazebo
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
             )

    # Spawn Robot in Gazebo
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'bot'],
                        output='screen')    

    # Holonomic Drive Conroller
    mec_cont_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mec_cont"],
    )
    
    # Joint State Broadcaster
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )
            
    # RVIZ
    rviz = Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        )       

    # Launch them all!
    return LaunchDescription([               
        node_robot_state_publisher,
        gazebo,
        spawn_entity,        
        mec_cont_spawner,
        joint_broad_spawner,       
        rviz
    ])

