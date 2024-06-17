#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Control(Node):
    def __init__(self):
        super().__init__('control')
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.listner_callback, 10)

    def listner_callback(self, msg):
        print("Linear X: {} | Angular Z: {}".format(msg.linear.xt, msg.angular.z))

def main(args=None):
    rclpy.init(args=args)

    control = Control()

    rclpy.spin(control)

    control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()