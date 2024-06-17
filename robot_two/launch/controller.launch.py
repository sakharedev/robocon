from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    # Get the path to the controller.yaml file
    controller_yaml_path = os.path.join(
        get_package_share_directory('robot_two'),
        'launch',
        'controller.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'controller_yaml',
            default_value=controller_yaml_path,
            description='Path to the controller yaml file'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            name='controller_spawner',
            namespace='mec_rob',
            output='screen',
            arguments=[
                'front_left_joint_position_controller',
                'Revolute_57_position_controller',
                'Revolute_58_position_controller',
                'Revolute_59_position_controller',
                'Revolute_60_position_controller',
                'Revolute_61_position_controller',
                'Revolute_62_position_controller',
                'Revolute_63_position_controller',
                'Revolute_64_position_controller',
                'Revolute_65_position_controller',
                'front_right_joint_position_controller',
                'Revolute_67_position_controller',
                'Revolute_68_position_controller',
                'Revolute_69_position_controller',
                'Revolute_70_position_controller',
                'Revolute_71_position_controller',
                'Revolute_72_position_controller',
                'Revolute_73_position_controller',
                'Revolute_74_position_controller',
                'Revolute_75_position_controller',
                'back_left_joint_position_controller',
                'Revolute_77_position_controller',
                'Revolute_78_position_controller',
                'Revolute_79_position_controller',
                'Revolute_80_position_controller',
                'Revolute_81_position_controller',
                'Revolute_82_position_controller',
                'Revolute_83_position_controller',
                'Revolute_84_position_controller',
                'Revolute_85_position_controller',
                'back_right_joint_position_controller',
                'Revolute_87_position_controller',
                'Revolute_88_position_controller',
                'Revolute_89_position_controller',
                'Revolute_90_position_controller',
                'Revolute_91_position_controller',
                'Revolute_92_position_controller',
                'Revolute_93_position_controller',
                'Revolute_94_position_controller',
                'Revolute_95_position_controller',
                'joint_state_controller'
            ],
            parameters=[ParameterValue('config', LaunchConfiguration('controller_yaml'))]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            remappings=[
                ('/joint_states', '/mec_rob/joint_states')
            ]
        ),
    ])

