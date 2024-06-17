#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from custom_interfaces.srv import StringServiceMessage
import cv2
from cv_bridge import CvBridge
import ament_index_python.packages as ament_index
import os

class ObjectDetectionNode(Node):

    def __init__(self, dummy=True):
        super().__init__('object_detection_node')

        self._dummy= dummy
        

        self.pkg_path = self.get_package_path("object_detection")
        self.scripts_path = os.path.join(self.pkg_path,"scripts")
        # cascade_file_path = os.path.join(self.scripts_path,'haarbanana.xml')

        # self.banana_cascade = cv2.CascadeClassifier(cascade_file_path)

        self.publisher = self.create_publisher(Image, 'image_detected_ball', 10)
        self.subscription = self.create_subscription(
            Image, '/camera_right/image', self.image_callback, 10)
        
        self.bridge = CvBridge()

        self.string_service = self.create_service(
            StringServiceMessage, 'detect_ball_service', self.string_service_callback)

        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.current_image = None
        self.detect_mode = None

        self.get_logger().info('READY ObjectDetectionNode')


    def get_package_path(self, package_name):
        try:
            package_share_directory = ament_index.get_package_share_directory(package_name)
            return package_share_directory
        except Exception as e:
            print(f"Error: {e}")
            return None

    def image_callback(self, msg):
        self.get_logger().info('Received an image.')
        self.current_image = msg

    def string_service_callback(self, request, response):
        # Handle the string service request
        self.get_logger().info(f'Received string service request: {request.detect}')
        
        self.detect_mode = request.detect  # Update the detection mode

        # Respond with success and a message
        response.success = True
        response.message = f'Received and processed: {request.detect}'
        return response

    def timer_callback(self):
        if self.detect_mode == "red_ball":
            self.detect_and_publish_red_ball()
        elif self.detect_mode == "blue_ball":
            self.detect_and_publish_blue_ball()

            
    def publish_image(self, image_msg):
        if image_msg is not None:
            self.publisher.publish(image_msg)


    def detect_and_publish_red_ball(self):
        if self.current_image is not None:
            frame = self.bridge.imgmsg_to_cv2(self.current_image, desired_encoding='bgr8')

            lower_red1 = (0, 50, 50)
            upper_red1 = (10, 255, 255)
            lower_red2 = (170, 50, 50)
            upper_red2 = (180, 255, 255)

            image_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # mask1 = cv2.inRange(image_hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(image_hsv, lower_red2, upper_red2)
            mask =  mask2

            cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            c_num = 0
            radius = 20
            for i, c in enumerate(cnts):
                ((x, y), r) = cv2.minEnclosingCircle(c)
                if r > radius:
                    print('OK=', str(r))
                    c_num += 1
                    cv2.circle(frame, (int(x), int(y)), int(r), (0, 255, 0), 2)
                    # prints x and y coordinates of the center of the circle
                    cv2.putText(frame, 'x={}, y={}'.format(int(x), int(y)), (int(x) - 10, int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                    # cv2.putText(frame, '#{}'.format(c_num), (int(x) - 10, int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                else:
                    print(r)

            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publish_image(image_msg)

        else:
            self.get_logger().error('Image Not Found')

    def detect_and_publish_blue_ball(self):
        if self.current_image is not None:
            frame = self.bridge.imgmsg_to_cv2(self.current_image, desired_encoding='bgr8')

            low_blue_raw = (110.0, 60.0, 50.0)
            high_blue_raw = (140.0, 120.0, 70.0)

            image_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            mask = cv2.inRange(image_hsv, low_blue_raw, high_blue_raw)

            cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            c_num = 0
            radius = 30
            for i, c in enumerate(cnts):
                ((x, y), r) = cv2.minEnclosingCircle(c)
                if r > radius:
                    c_num += 1
                    cv2.circle(frame, (int(x), int(y)), int(r), (0, 255, 0), 2)
                    cv2.putText(frame, '#{}'.format(c_num), (int(x) - 10, int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                else:
                    print(r)

            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publish_image(image_msg)

        else:
            self.get_logger().error('Image Not Found')

    def detect_and_publish_empty_grain(self):
        if self.current_image is not None:
            frame = self.bridge.imgmsg_to_cv2(self.current_image, desired_encoding='bgr8')

            #purple color
            low_empty_raw = (140.0, 80.0, 100.0)
            high_empty_raw = (130.0, 70.0, 110.0)

            image_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            mask = cv2.inRange(image_hsv, low_empty_raw, high_empty_raw)

            cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            c_num = 0
            radius = 30
            for i, c in enumerate(cnts):
                ((x, y), r) = cv2.minEnclosingCircle(c)
                if r > radius:
                    c_num += 1
                    cv2.circle(frame, (int(x), int(y)), int(r), (0, 255, 0), 2)
                    cv2.putText(frame, '#{}'.format(c_num), (int(x) - 10, int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                else:
                    print(r)

            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publish_image(image_msg)

        else:
            self.get_logger().error('Image Not Found')



def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode(dummy=False)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()