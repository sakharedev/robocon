#!/usr/bin/env python3

import os
import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from launch_ros.substitutions import FindPackageShare

from sensor_msgs.msg import Image

from custom_interfaces.srv import StringServiceMessage
import ament_index_python

class ObjectDetectionNode(Node):

    def __init__(self, dummy = True):
        super().__init__('object_detection')

        self._dummy = dummy
        self.pkg_path = self.get_package_path('object_detection')
        self.scripts_path = os.path.join(self.pkg_path, 'scripts')

        self.bridge = CvBridge()

        self.publisher = self.create_publisher(Image, 'image_detected_ball', 10)
        self.subscription = self.create_subscription(Image, '/image', self.image_callback, 10)

        self.string_service = self.create_service(StringServiceMessage, 'detect_ball_service', self.string_service_callback)

        self.get_logger().info('Object Detection Node has been started')

    def get_package_path(self, package_name):
        try:
            package_share_directory = ament_index_python.get_package_share_directory(package_name)
            return package_share_directory
        except Exception as e:
            print(f'Error: {e}')
            return None

    def image_callback(self, msg):
        self.get_logger().info('Recevied an Image')
        self.current_img = msg

    def string_service_callback(self, request, response):
        self.get_logger().info(f'Received String Service Request: {request.detect}')

        if request.detect == 'red_ball':
            if self._dummy:
                red_ball = self.generate_red_ball_detection_image()
                self.publish_image(red_ball)
            else:
                self.detect_and_publish_red_ball()

        elif request.detect == 'blue_ball':
            if self._dummy:
                blue_ball = self.generate_blue_ball_detection_image()
                self.publish_image(blue_ball)
            else:
                self.detect_and_publish_blue_ball()

        elif request.detect == 'empty_grain':
            if self._dummy:
                empty_grain = self.genereate_empty_grain_detection_image()
                self.publish_image(empty_grain)
            else:
                self.detect_and_publish_empty_grain()

        else :
            self.get_logger().info('Unknown Request')


        response.success = True
        response.message = f'Received and Processed: {request.detect}'
        return response


    def genereate_empty_grain_detection_image(self):
        self.img_path = os.path.join(self.scripts_path, 'empty_grain.jpeg')
        self.get_logger().warning('Empty Grain Patth=' + self.img_path)
        image = cv2.imread(self.img_path)

        if image is None:
            self.get_logger().error('Failed to load the Empty Grain image')
        else:
            self.get_logger().warning('SUCCESS to load the Empty Grain image')
        
        return self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
    
    def generate_red_ball_detection_image(self):
        self.img_path = os.path.join(self.scripts_path, 'red_ball.jpeg')
        self.get_logger().warning("Red Ball Path="+str(self.img_path))
        image = cv2.imread(self.img_path)

        if image is None:
            self.get_logger().error('Failed to load the Red ball Image')
        else:
            self.get_logger().warning('SUCCESS to load the Red Ball Image')
        
        return self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
    
    def generate_blue_ball_detection_image(self):
        self.img_path = os.path.join(self.scripts_path, 'blue_ball.jpeg')
        self.get_logger().warning('Blue Ball Path='+str(self.img_path))
        image = cv2.imread(self.img_path)

        if image is None:
            self.get_logger().error('Falied to load the Blue Ball Image')
        else:
            self.get_logger().warning('SUCCESS to load the Blue Ball Image')
        
        return self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
    
    def publish_image(self, image_msg):
        if image_msg is not None:
            self.publisher.publish(image_msg)


    def detect_and_publish_red_ball(self):
        if self.current_img is not None:
            frame = self.bridge.imgmsg_to_cv2(self.current_img, desired_encoding='bgr8')

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
            radius = 30
            for i, c in enumerate(cnts):
                ((x, y), r) = cv2.minEnclosingCircle(c)
                if r > radius:
                    print('OK=', str(r))
                    c_num += 1
                    cv2.circle(frame, (int(x), int(y)), int(r), (0, 255, 0), 2)
                    cv2.putText(frame, '#{}'.format(c_num), (int(x) - 10, int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                else:
                    print(r)

            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publish_image(image_msg)

        else:
            self.get_logger().error('Image Not Found')

    def detect_and_publish_blue_ball(self):
        if self.current_img is not None:
            frame = self.bridge.imgmsg_to_cv2(self.current_img, desired_encoding='bgr8')

            low_blue_raw = (113.0, 33.0, 175.0)
            high_blue_raw = (117.0, 94.0, 66.0)

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
        if self.current_img is not None:
            frame = self.bridge.imgmsg_to_cv2(self.current_img, desired_encoding='bgr8')

            low_empty_raw = (0.0, 0.0, 0.0)
            high_empty_raw = (179.0, 50.0, 100.0)

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
    node = ObjectDetectionNode(dummy = False)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()