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
                ('device_id_1', 0),
                ('device_id_2', 4),
                ('width', 320),
                ('height', 240),
                ('burger_mode', False),
                ('frame_id_1', 'camera_frame'),
                ('frame_id_2', 'camera_frame')
            ]
        )

        self.reliability_policy = self.get_parameter('reliability').get_parameter_value().string_value
        self.history_policy = self.get_parameter('history').get_parameter_value().string_value
        self.depth = self.get_parameter('depth').get_parameter_value().integer_value
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self.show_camera = self.get_parameter('show_camera').get_parameter_value().bool_value
        self.device_id_1 = self.get_parameter('device_id_1').get_parameter_value().integer_value
        self.device_id_2 = self.get_parameter('device_id_2').get_parameter_value().integer_value
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.burger_mode = self.get_parameter('burger_mode').get_parameter_value().bool_value
        self.frame_id_1 = self.get_parameter('frame_id_1').get_parameter_value().string_value
        self.frame_id_2 = self.get_parameter('frame_id_2').get_parameter_value().string_value

        self.is_flipped = False
        self.publish_number = 1
        self.bridge = CvBridge()

        if not self.burger_mode:
            self.cap1 = cv2.VideoCapture(self.device_id_1)
            self.cap1.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            if not self.cap1.isOpened():
                self.get_logger().error("Could not open video stream for device 1")
                raise RuntimeError("Could not open video stream for device 1")

            self.cap2 = cv2.VideoCapture(self.device_id_2)
            self.cap2.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            if not self.cap2.isOpened():
                self.get_logger().error("Could not open video stream for device 2")
                raise RuntimeError("Could not open video stream for device 2")

        qos_profile = self.create_qos_profile()
        self.pub1 = self.create_publisher(Image, '/camera_right/image', qos_profile)
        self.pub2 = self.create_publisher(Image, '/camera_left/image', qos_profile)
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
            frame1 = self.render_burger(self.width, self.height)
            frame2 = self.render_burger(self.width, self.height)
        else:
            ret1, frame1 = self.cap1.read()
            ret2, frame2 = self.cap2.read()
            if not ret1 or not ret2:
                return

        if self.is_flipped:
            frame1 = cv2.flip(frame1, 1)
            frame2 = cv2.flip(frame2, 1)

        if self.show_camera:
            cv2.imshow('Camera Right', frame1)
            cv2.imshow('Camera Left', frame2)
            cv2.waitKey(1)

        msg1 = self.bridge.cv2_to_imgmsg(frame1, encoding="bgr8")
        msg1.header.frame_id = self.frame_id_1
        msg1.header.stamp = self.get_clock().now().to_msg()
        self.pub1.publish(msg1)

        msg2 = self.bridge.cv2_to_imgmsg(frame2, encoding="bgr8")
        msg2.header.frame_id = self.frame_id_2
        msg2.header.stamp = self.get_clock().now().to_msg()
        self.pub2.publish(msg2)

        self.get_logger().info(f"Publishing image #{self.publish_number} from both cameras")
        self.publish_number += 1

    def render_burger(self, width, height):
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
            node.cap1.release()
            node.cap2.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
