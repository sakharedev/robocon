import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
from launch_ros.actions import Node
import os

from launch.actions import IncludeLaunchDescription

from ament_index_python import get_package_share_directory


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='robot_two').find('robot_two')
    world_path=os.path.join(pkg_share, 'worlds/robocon_final_field.sdf')
    use_sim_time = LaunchConfiguration('use_sim_time')

    ignition_gazebo = IncludeLaunchDescription(os.path.join(
        get_package_share_directory('ros_ign_gazebo'), 'launch', 'ign_gazebo.launch.py'),
        launch_arguments={'ign_args': 'r -v 4 ' + world_path}.items()
    )

    # spawn a robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'tortoisebot', '-topic', 'robot_description'],
        parameters= [{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # include rsp node
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[os.path.join(pkg_share, 'urdf/mec_rob.xacro')]
    ) 

    return launch.LaunchDescription([

        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='False',
                                description='Flag to enable use_sim_time'),


        spawn_entity,
        rsp_node,
        ignition_gazebo
    ])
    