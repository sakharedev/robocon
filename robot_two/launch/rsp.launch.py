import launch
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    # robot State publisher

    urdf = os.path.join(
        get_package_share_directory('robot_two'),
        'urdf',
        'mec_rob.xacro'
    )

    xacro_node = Node(
        package='xacro',
        executable='xacro',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[os.path.join(get_package_share_directory('robot_two'), 'urdf', 'robot.xacro')]
    )

    

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[urdf]
    )

    # Joint State Publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return launch.LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        xacro_node
    ])