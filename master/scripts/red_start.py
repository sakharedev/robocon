import subprocess
import os
from time import sleep

def main():
    # Set the ROS package directory
    package_dir = os.path.join(os.getenv('HOME'), '/home/mecha/robocon/mecha/robocon')

    # Define the sequence of launch files and scripts to execute
    launch_sequence = [
        {
            'description': 'Launching the Line Follower Sequence',
            'commands': [
                ['ros2', 'run', 'object_detection', 'pub_front_right.py', '--ros-args', '-p', 'device_id:=0'],
                ['ros2', 'run', 'line_follower', 'line_navigation.py'],
                # ['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', '/dev/ttyACM0'],
            ]
        },
        # {
        #     'description': 'Launching the ball Picking Sequence',
        #     'commands': [
        #         ['ros2', 'run', 'object_detection', 'DetectionAndDistance.py'],
        #         ['ros2', 'run', 'object_detection', 'pub_front_right.py'],
        #         ['ros2', 'run', 'object_detection', 'pub_front_left.py'],
        #         ['ros2', 'service', 'call', '/detect_ball_service', 'custom_interface/srv/StringServiceMessage', '"detect: "red_ball""']
        #     ]
        # },
    ]

    # Execute each command in the sequence
    for launch in launch_sequence:
        print(f"{launch['description']}...")
        for command in launch['commands']:
            process = subprocess.Popen(command)
            process.wait()  # Wait for the command to complete before proceeding
            sleep(10)  # Adjust sleep time as needed

if __name__ == "__main__":
    main()