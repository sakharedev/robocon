from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
import os
from ament_index_python import get_package_share_directory
from launch.conditions import IfCondition

def generate_launch_description():
    # Get the path to the URDF and RViz config files
    urdf_path = os.path.join(
        get_package_share_directory('robot_two'),
        'urdf',
        'mec_rob.xacro'
    )
    rviz_config_path = os.path.join(
        get_package_share_directory('robot_two'),
        'launch',
        'urdf.rviz'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=urdf_path,
            description='Path to the robot URDF file'
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Flag to enable joint_state_publisher_gui'
        ),
        DeclareLaunchArgument(
            'rvizconfig',
            default_value=rviz_config_path,
            description='Path to the RViz config file'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', LaunchConfiguration('model')])
            }]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(LaunchConfiguration('gui'))
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rvizconfig')],
            parameters=[{
                'use_sim_time': True
            }]
        ),
    ])

