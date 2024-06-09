#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from custom_interfaces.srv import StringServiceMessage
import cv2
from cv_bridge import CvBridge
import ament_index_python.packages as ament_index
import os


class DetectionAndDistance(Node):

    def __init__(self):
        super().__init__("detection_and_distance_node")

        self.pkg_path = self.get_package_path("object_detection")

        self.publisher_left_ = self.create_publisher(Image, "/left_detection", 10)
        self.publisher_right_ = self.create_publisher(Image, "/right_detection", 10)


        self.subscription_left_ = self.create_subscription(Image, "/camera_left/image", self.image_left_callback, 10)
        self.subscription_right_ = self.create_subscription(Image, "/camera_right/image", self.image_right_callback, 10)
        

        self.bridge = CvBridge()

        self.string_service_ = self.create_service(StringServiceMessage, "/detect_ball_service", self.string_service_callback)

        self.timer_ = self.create_timer(0.1, self.timer_callback)

        self.detect_mode = None
        self.burger_mode = False
        self.current_left_image_ = None
        self.current_right_image_ = None
    
    def get_package_path(self, package_name):
        try:
            package_share_directory = ament_index.get_package_share_directory(package_name)
            return package_share_directory
        except Exception as e:
            print(f"Error: {e}")
            return None
        
    def image_left_callback(self, msg):
        # self.get_logger().info("Received left Image.")
        self.current_left_image_ = msg

    def image_right_callback(self, msg):
        # self.get_logger().info("Received right Image.")
        self.current_right_image_ =msg

    def string_service_callback(self, request, response):
        self.get_logger().info(f"Received string service request: {request.detect}")

        self.detect_mode = request.detect 

        response.success = True
        response.message = f"Received and Processed: {request.detect}"
        return response
    
    def timer_callback(self):
        if self.detect_mode == "red_ball":
            self.detect_and_publish_red_ball_distance()
        elif self.detect_mode == "blue_ball":
            self.detect_and_published_blue_ball_distance()

    def publish_left_image(self, image_msg_left):
        if image_msg_left is not None:
            self.publisher_left_.publish(image_msg_left)

    def publish_right_image(self, image_msg_right):
        if image_msg_right is not None:
            self.publisher_right_.publish(image_msg_right)
        
    def detect_and_publish_red_ball_distance(self):
        # if self.current_left_image_ is not None and self.current_right_image_ is not None:
        #     left_frame_ = self.bridge.imgmsg_to_cv2(self.current_left_image_, desired_encoding="bgr8")
        #     right_frame_ = self.bridge.imgmsg_to_cv2(self.current_right_image_, desired_encoding="bgr8")

        #     lower_red = (170, 50, 50)
        #     upper_red = (180, 255, 255)

        #     image_hsv_left = cv2.cvtColor(left_frame_, cv2.COLOR_BGR2HSV)
        #     image_hsv_right = cv2.cvtColor(right_frame_, cv2.COLOR_BGR2HSV)

        #     mask_left = cv2.inRange(image_hsv_left, lower_red, upper_red)
        #     mask_right = cv2.inRange(image_hsv_right, lower_red, upper_red)

        #     cnts_left, _ = cv2.findContours(mask_left, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #     cnts_right, _ = cv2.findContours(mask_right, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        #     c_num = 0
        #     radius_upper = 57.5
        #     radius_lower = 37.5

        #     # Process contours in the left frame
        #     for i, c in enumerate(cnts_left):
        #         ((x, y), r) = cv2.minEnclosingCircle(c)
        #         if r > radius_lower and r < radius_upper:
        #             c_num += 1
        #             cv2.circle(left_frame_, (int(x), int(y)), int(r), (0, 255, 0), 2)
        #             cv2.putText(left_frame_, 'x={}, y={}, #{}'.format(int(x), int(y), c_num), (int(x) - 10, int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        #             print(f"Left Frame: Ball #{c_num}: x={int(x)}, y={int(y)}, radius={r}")

        #     # Reset contour number for the right frame
        #     c_num = 0

        #     # Process contours in the right frame
        #     for i, c in enumerate(cnts_right):
        #         ((x, y), r) = cv2.minEnclosingCircle(c)
        #         if r > radius_lower and r < radius_upper:
        #             c_num += 1
        #             cv2.circle(right_frame_, (int(x), int(y)), int(r), (0, 255, 0), 2)
        #             cv2.putText(right_frame_, 'x={}, y={}, #{}'.format(int(x), int(y), c_num), (int(x) - 10, int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        #             print(f"Right Frame: Ball #{c_num}: x={int(x)}, y={int(y)}, radius={r}")

                        
        #         else:
        #             print(r)
        if self.current_left_image_ is not None and self.current_right_image_ is not None:
            left_frame_ = self.bridge.imgmsg_to_cv2(self.current_left_image_, desired_encoding="bgr8")
            right_frame_ = self.bridge.imgmsg_to_cv2(self.current_right_image_, desired_encoding="bgr8")

            lower_red = (170, 50, 50)
            upper_red = (180, 255, 255)

            image_hsv_left = cv2.cvtColor(left_frame_, cv2.COLOR_BGR2HSV)
            image_hsv_right = cv2.cvtColor(right_frame_, cv2.COLOR_BGR2HSV)

            mask_left = cv2.inRange(image_hsv_left, lower_red, upper_red)
            mask_right = cv2.inRange(image_hsv_right, lower_red, upper_red)

            cnts_left, _ = cv2.findContours(mask_left, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts_right, _ = cv2.findContours(mask_right, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            c_num = 0
            min_radius = 37.5  # Lower limit of radius in pixels
            max_radius = 57.5  # Upper limit of radius in pixels

            left_ball_coordinates = None
            right_ball_coordinates = None

            # Process contours in the left frame
            for i, c in enumerate(cnts_left):
                ((x, y), r) = cv2.minEnclosingCircle(c)
                if min_radius <= r <= max_radius:
                    c_num += 1
                    cv2.circle(left_frame_, (int(x), int(y)), int(r), (0, 255, 0), 2)
                    cv2.putText(left_frame_, 'x={}, y={}, #{}'.format(int(x), int(y), c_num), (int(x) - 10, int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                    print(f"Left Frame: Ball #{c_num}: x={int(x)}, y={int(y)}, radius={r}")
                    left_ball_coordinates = (x, y)
                    break  # Assuming only one ball is detected

            # Reset contour number for the right frame
            c_num = 0

            # Process contours in the right frame
            for i, c in enumerate(cnts_right):
                ((x, y), r) = cv2.minEnclosingCircle(c)
                if min_radius <= r <= max_radius:
                    c_num += 1
                    cv2.circle(right_frame_, (int(x), int(y)), int(r), (0, 255, 0), 2)
                    cv2.putText(right_frame_, 'x={}, y={}, #{}'.format(int(x), int(y), c_num), (int(x) - 10, int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                    print(f"Right Frame: Ball #{c_num}: x={int(x)}, y={int(y)}, radius={r}")
                    right_ball_coordinates = (x, y)
                    break  # Assuming only one ball is detected

            # Calculate the distance to the ball using the disparity
            if left_ball_coordinates is not None and right_ball_coordinates is not None:
                x_left = left_ball_coordinates[0]
                x_right = right_ball_coordinates[0]
                disparity = abs(x_left - x_right)
                if disparity != 0:  # Avoid division by zero
                    baseline = 360  # in mm
                    camera_size = 0.0062  # in mm/px
                    focal_length = 3.6  # in mm
                    distance = (baseline * focal_length) / (disparity * camera_size)
                    print(f"Distance to the ball: {distance:.2f} mm")
                else:
                    print("Disparity is zero, cannot compute distance.")




            image_msg_left = self.bridge.cv2_to_imgmsg(left_frame_, encoding='bgr8')
            image_msg_right = self.bridge.cv2_to_imgmsg(right_frame_, encoding='bgr8')
            self.publish_left_image(image_msg_left)
            self.publish_right_image(image_msg_right)

        else:
            self.get_logger().error('Image Not Found')

    def detect_and_published_blue_ball_distance(self):
        if self.current_left_image_ is not None and self.current_right_image_ is not None:
            left_frame_ = self.bridge.imgmsg_to_cv2(self.current_left_image_, desired_encoding="bgr8")
            right_frame_ = self.bridge.imgmsg_to_cv2(self.current_right_image_, desired_encoding="bgr8")

            lower_blue = (110, 50, 50)
            upper_blue = (130, 255, 255)

            image_hsv_left = cv2.cvtColor(left_frame_, cv2.COLOR_BGR2HSV)
            image_hsv_right = cv2.cvtColor(right_frame_, cv2.COLOR_BGR2HSV)

            mask_left = cv2.inRange(image_hsv_left, lower_blue, upper_blue)
            mask_right = cv2.inRange(image_hsv_right, lower_blue, upper_blue)

            cnts_left, _ = cv2.findContours(mask_left, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts_right, _ = cv2.findContours(mask_right, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            c_num = 0
            min_radius = 37.5  # Lower limit of radius in pixels
            max_radius = 57.5  # Upper limit of radius in pixels

            left_ball_coordinates = None
            right_ball_coordinates = None

            # Process contours in the left frame
            for i, c in enumerate(cnts_left):
                ((x, y), r) = cv2.minEnclosingCircle(c)
                if min_radius <= r <= max_radius:
                    c_num += 1
                    cv2.circle(left_frame_, (int(x), int(y)), int(r), (0, 255, 0), 2)
                    cv2.putText(left_frame_, 'x={}, y={}, #{}'.format(int(x), int(y), c_num), (int(x) - 10, int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                    print(f"Left Frame: Ball #{c_num}: x={int(x)}, y={int(y)}, radius={r}")
                    left_ball_coordinates = (x, y)
                    break  # Assuming only one ball is detected

            # Reset contour number for the right frame
            c_num = 0

            # Process contours in the right frame
            for i, c in enumerate(cnts_right):
                ((x, y), r) = cv2.minEnclosingCircle(c)
                if min_radius <= r <= max_radius:
                    c_num += 1
                    cv2.circle(right_frame_, (int(x), int(y)), int(r), (0, 255, 0), 2)
                    cv2.putText(right_frame_, 'x={}, y={}, #{}'.format(int(x), int(y), c_num), (int(x) - 10, int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                    print(f"Right Frame: Ball #{c_num}: x={int(x)}, y={int(y)}, radius={r}")
                    right_ball_coordinates = (x, y)
                    break  # Assuming only one ball is detected

            # Calculate the distance to the ball using the disparity
            if left_ball_coordinates is not None and right_ball_coordinates is not None:
                x_left = left_ball_coordinates[0]
                x_right = right_ball_coordinates[0]
                disparity = abs(x_left - x_right)
                if disparity != 0:  # Avoid division by zero
                    baseline = 360  # in mm
                    camera_size = 0.0062  # in mm/px
                    focal_length = 3.6  # in mm
                    distance = (baseline * focal_length) / (disparity * camera_size)
                    print(f"Distance to the ball: {distance:.2f} mm")
                else:
                    print("Disparity is zero, cannot compute distance.")




            image_msg_left = self.bridge.cv2_to_imgmsg(left_frame_, encoding='bgr8')
            image_msg_right = self.bridge.cv2_to_imgmsg(right_frame_, encoding='bgr8')
            self.publish_left_image(image_msg_left)
            self.publish_right_image(image_msg_right)

        else:
            self.get_logger().error('Image Not Found')


def main(args=None):
    rclpy.init(args=args)
    node = DetectionAndDistance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not node.burger_mode:
            node.cap.release()

if __name__ == "__main__":
    main()

