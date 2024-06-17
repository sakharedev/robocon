from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the URDF file
    urdf_file_path = os.path.join(
        get_package_share_directory('robot_two'),
        'urdf',
        'mec_rob.xacro'
    )

    # Get the path to the Gazebo ROS empty world launch file
    gazebo_ros_launch_dir = os.path.join(
        get_package_share_directory('robot_two'),
        'worlds'
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_file_path])
            }]
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_urdf',
            output='screen',
            arguments=[
                '-param', 'robot_description',
                '-urdf',
                '-model', 'mec_rob',
                '-z', '0.055'
            ]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_launch_dir, 'robocon_final_field.sdf')
            ),
            launch_arguments={
                'paused': 'true',
                'use_sim_time': 'true',
                'gui': 'true',
                'headless': 'false',
                'debug': 'false'
            }.items()
        )
    ])
