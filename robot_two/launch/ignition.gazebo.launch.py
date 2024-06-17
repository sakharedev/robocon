# import launch
# from launch.substitutions import Command, LaunchConfiguration
# import launch_ros
# from launch_ros.actions import Node
# import os

# from launch.actions import IncludeLaunchDescription

# from ament_index_python import get_package_share_directory


# def generate_launch_description():
#     pkg_share = launch_ros.substitutions.FindPackageShare(package='robot_two').find('robot_two')
#     world_path=os.path.join(pkg_share, 'worlds/robocon_final_field.sdf')
#     use_sim_time = LaunchConfiguration('use_sim_time')

#     ignition_gazebo = IncludeLaunchDescription(os.path.join(
#         get_package_share_directory('ros_ign_gazebo'), 'launch', 'ign_gazebo.launch.py'),
#         launch_arguments={'ign_args': 'r -v 4 ' + world_path}.items()
#     )

#     # spawn a robot
#     spawn_entity = Node(
#         package='gazebo_ros',
#         executable='spawn_entity.py',
#         arguments=['-entity', 'tortoisebot', '-topic', 'robot_description'],
#         parameters= [{'use_sim_time': use_sim_time}],
#         output='screen'
#     )

#     # include rsp node
#     rsp_node = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         name='robot_state_publisher',
#         output='screen',
#         parameters=[{'use_sim_time': True}],
#         arguments=[os.path.join(pkg_share, 'urdf/mec_rob.xacro')]
#     ) 

#     return launch.LaunchDescription([

#         launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='False',
#                                 description='Flag to enable use_sim_time'),


#         spawn_entity,
#         rsp_node,
#         ignition_gazebo
#     ])
    

import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
import os
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='robot_two').find('robot_two')
    # gazebo_pkg_share = launch_ros.substitutions.FindPackageShare(package='robot_two_gazebo').find('robot_two_gazebo')
    default_model_path = os.path.join(pkg_share, 'urdf/mec_rob.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/sensors.rviz')
    world_path=os.path.join(pkg_share, 'worlds/robocon_final_field.sdf'),
    #sdf_path=os.path.join(pkg_share, 'urdf/robot_two', 'practice.world'),
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time},{'robot_description': ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),value_type=str)}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters= [{'use_sim_time': use_sim_time}],
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters= [{'use_sim_time': use_sim_time}],

    )
    spawn_entity = launch_ros.actions.Node(
    condition= IfCondition(use_sim_time),
    package='ros_ign_gazebo',
    executable='ign_gazebo.launch.py',
    arguments=['-entity', 'robot_two', '-topic', 'robot_description','-x', '5.0', '-y', '0.0', '-z' ,'8.0'],
    parameters= [{'use_sim_time': use_sim_time}],
    output='screen'
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                    description='Flag to enable use_sim_time'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.ExecuteProcess(condition= IfCondition(use_sim_time),cmd=['gazebo', '--verbose', '-s', 
                                            'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so',world_path], 
                                            output='screen'),


        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_entity,
        rviz_node,
    ])
    