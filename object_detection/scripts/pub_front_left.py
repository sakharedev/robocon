#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np

class Cam2Image(Node):

    def __init__(self):
        super().__init__('cam2image')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('reliability', 'reliable'),
                ('history', 'keep_last'),
                ('depth', 10),
                ('frequency', 30.0),
                ('show_camera', False),
                ('device_id', 4),
                ('width', 320),
                ('height', 240),
                ('burger_mode', False),
                ('frame_id', 'camera_frame')
            ]
        )

        self.reliability_policy = self.get_parameter('reliability').get_parameter_value().string_value
        self.history_policy = self.get_parameter('history').get_parameter_value().string_value
        self.depth = self.get_parameter('depth').get_parameter_value().integer_value
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self.show_camera = self.get_parameter('show_camera').get_parameter_value().bool_value
        self.device_id = self.get_parameter('device_id').get_parameter_value().integer_value
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.burger_mode = self.get_parameter('burger_mode').get_parameter_value().bool_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.is_flipped = False
        self.publish_number = 1
        self.bridge = CvBridge()

        if not self.burger_mode:
            self.cap = cv2.VideoCapture(self.device_id)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            if not self.cap.isOpened():
                self.get_logger().error("Could not open video stream")
                raise RuntimeError("Could not open video stream")

        qos_profile = self.create_qos_profile()
        self.pub = self.create_publisher(Image, '/camera_left/image', qos_profile)
        self.sub = self.create_subscription(Bool, 'flip_image', self.flip_callback, 10)
        self.timer = self.create_timer(1.0 / self.frequency, self.timer_callback)

    def create_qos_profile(self):
        reliability_policy = rclpy.qos.ReliabilityPolicy.RELIABLE if self.reliability_policy == 'reliable' else rclpy.qos.ReliabilityPolicy.BEST_EFFORT
        history_policy = rclpy.qos.HistoryPolicy.KEEP_ALL if self.history_policy == 'keep_all' else rclpy.qos.HistoryPolicy.KEEP_LAST
        return rclpy.qos.QoSProfile(
            history=history_policy,
            depth=self.depth,
            reliability=reliability_policy
        )

    def flip_callback(self, msg):
        self.is_flipped = msg.data
        self.get_logger().info(f"Set flip mode to: {'on' if self.is_flipped else 'off'}")

    def timer_callback(self):
        if self.burger_mode:
            frame = self.render_burger(self.width, self.height)
        else:
            ret, frame = self.cap.read()
            if not ret:
                return

        if self.is_flipped:
            frame = cv2.flip(frame, 1)

        if self.show_camera:
            cv2.imshow('cam2image', frame)
            cv2.waitKey(1)

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)
        self.get_logger().info(f"Publishing image #{self.publish_number}")
        self.publish_number += 1

    def render_burger(self, width, height):
        # Generate a simple image of a "burger"
        image = np.zeros((height, width, 3), dtype=np.uint8)
        cv2.putText(image, 'Burger', (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 3, (0, 255, 0), 2, cv2.LINE_AA)
        return image

def main(args=None):
    rclpy.init(args=args)
    node = Cam2Image()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not node.burger_mode:
            node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
