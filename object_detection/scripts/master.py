#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import subprocess
import threading

class SequenceExecutorNode(Node):
    def __init__(self):
        super().__init__('sequence_executor_node')
        self.declare_parameter('sequence', '')
        self.create_timer(1.0, self.execute_sequence, callback_group=ReentrantCallbackGroup())
        self.processes = []

    def execute_sequence(self):
        sequence_input = self.get_parameter('sequence').get_parameter_value().string_value.lower()

        if not sequence_input:
            self.get_logger().info("No sequence specified. Use '--ros-args -p sequence:=<sequence_name>'")
            return

        if sequence_input == 'ball_picking':
            self.get_logger().info("Launching the Ball Picking Sequence")
            commands = [
                ['ros2', 'run', 'object_detection', 'DetectionAndDistance.py'],
                ['ros2', 'run', 'object_detection', 'pub_front_right.py', '--ros-args', '-p', 'device_id:=2'],
                ['ros2', 'run', 'object_detection', 'pub_front_left.py', '--ros-args', '-p', 'device_id:=4'],
                ['ros2', 'service', 'call', '/detect_ball_service', 'custom_interfaces/srv/StringServiceMessage', '"detect: "red_ball""']
            ]
        elif sequence_input == 'silo':
            self.get_logger().info("Launching the Ball Picking Sequence")
            commands = [
                ['ros2', 'run', 'object_detection', 'DetectionAndDistance.py'],
                ['ros2', 'run', 'object_detection', 'pub_front_right.py', '--ros-args', '-p', 'device_id:=2'],
                ['ros2', 'run', 'object_detection', 'pub_front_left.py', '--ros-args', '-p', 'device_id:=4']
            ]
        else:
            self.get_logger().error(f"Unknown sequence: {sequence_input}")
            return

        for command in commands:
            thread = threading.Thread(target=self.run_command, args=(command,))
            thread.start()

        # If it's the ball_picking sequence, we'll call the service after a delay
        if sequence_input == 'ball_picking':
            self.create_timer(10.0, self.call_ball_detection_service, callback_group=ReentrantCallbackGroup())

    def run_command(self, command):
        process = subprocess.Popen(command)
        self.processes.append(process)

    def call_ball_detection_service(self):
        self.get_logger().info("Calling ball detection service")
        subprocess.run(['ros2', 'service', 'call', '/detect_ball_service', 'custom_interface/srv/StringServiceMessage', '"detect: "red_ball""'])

    def on_shutdown(self):
        for process in self.processes:
            process.terminate()

def main(args=None):
    rclpy.init(args=args)
    node = SequenceExecutorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 