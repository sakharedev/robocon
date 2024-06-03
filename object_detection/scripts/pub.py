#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import cv2
from cv_bridge import CvBridge

class Cam2Image(Node):

    def __init__(self):
        super().__init__('cam2image')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('show_camera', False),
                ('depth', 10),
                ('freq', 30.0),
                ('reliability_policy', 'reliable'),
                ('history_policy', 'keep_last'),
                ('width', 320),
                ('height', 240),
                ('burger_mode', False),
                ('topic', 'image')
            ]
        )

        self.show_camera = self.get_parameter('show_camera').get_parameter_value().bool_value
        depth = self.get_parameter('depth').get_parameter_value().integer_value
        freq = self.get_parameter('freq').get_parameter_value().double_value
        reliability_policy = self.get_parameter('reliability_policy').get_parameter_value().string_value
        history_policy = self.get_parameter('history_policy').get_parameter_value().string_value
        width = self.get_parameter('width').get_parameter_value().integer_value
        height = self.get_parameter('height').get_parameter_value().integer_value
        self.burger_mode = self.get_parameter('burger_mode').get_parameter_value().bool_value
        topic = self.get_parameter('topic').get_parameter_value().string_value

        qos_profile = QoSProfile(
            depth=depth,
            reliability=ReliabilityPolicy.RELIABLE if reliability_policy == 'reliable' else ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST if history_policy == 'keep_last' else HistoryPolicy.KEEP_ALL
        )

        self.publisher_ = self.create_publisher(Image, topic, qos_profile)
        self.subscription = self.create_subscription(
            Bool,
            'flip_image',
            self.flip_callback,
            10
        )

        self.is_flipped = False
        self.bridge = CvBridge()

        if not self.burger_mode:
            self.cap = cv2.VideoCapture(2)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            if not self.cap.isOpened():
                self.get_logger().error('Could not open video stream')
                rclpy.shutdown()
                return

        self.timer = self.create_timer(1.0 / freq, self.timer_callback)
        self.frame_id = 1

    def flip_callback(self, msg):
        self.is_flipped = msg.data
        self.get_logger().info(f'Set flip mode to: {"on" if self.is_flipped else "off"}')

    def timer_callback(self):
        if self.burger_mode:
            frame = self.render_burger()
        else:
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error('Failed to capture image')
                return

        if self.is_flipped:
            frame = cv2.flip(frame, 1)

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.frame_id = str(self.frame_id)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing image #{self.frame_id}')
        self.frame_id += 1

        if self.show_camera:
            cv2.imshow('cam2image', frame)
            cv2.waitKey(1)

    def render_burger(self):
        # Replace with actual implementation if needed
        frame = cv2.imread('burger.jpg')
        return cv2.resize(frame, (self.width, self.height))

def main(args=None):
    rclpy.init(args=args)
    cam2image = Cam2Image()
    rclpy.spin(cam2image)
    cam2image.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
