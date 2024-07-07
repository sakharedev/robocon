from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='object_detection',  # Replace with your package name
            executable='pub_front_left.py',  # Replace with your node executable name
            name='cam2image_left',
            namespace='camera_left',
            parameters=[
                {'reliability': 'reliable'},
                {'history': 'keep_last'},
                {'depth': 10},
                {'frequency': 30.0},
                {'show_camera': False},
                {'device_id': 4},
                {'width': 320},
                {'height': 240},
                {'burger_mode': False},
                {'frame_id': 'camera_frame_left'}
            ],
            output='screen'
        ),
        Node(
            package='object_detection',  # Replace with your package name
            executable='pub_front_right.py',  # Replace with your node executable name
            name='cam2image_right',
            namespace='camera_right',
            parameters=[
                {'reliability': 'reliable'},
                {'history': 'keep_last'},
                {'depth': 10},
                {'frequency': 30.0},
                {'show_camera': False},
                {'device_id': 2},
                {'width': 320},
                {'height': 240},
                {'burger_mode': False},
                {'frame_id': 'camera_frame_right'}
            ],
            output='screen'
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()
