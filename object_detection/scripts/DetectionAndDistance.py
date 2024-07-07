#!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from geometry_msgs.msg import Twist
# from std_msgs.msg import String
# from custom_interfaces.srv import StringServiceMessage
# import cv2
# from cv_bridge import CvBridge
# import ament_index_python.packages as ament_index
# import os
# import numpy as np

# class DetectionAndDistance(Node):

#     def __init__(self):
#         super().__init__("detection_and_distance_node")

#         self.pkg_path = self.get_package_path("object_detection")

#         self.publisher_left_ = self.create_publisher(Image, "/left_detection", 10)
#         self.publisher_right_ = self.create_publisher(Image, "/right_detection", 10)
#         self.cmd_vel_publisher_ = self.create_publisher(Twist, '/cmd_vel/ball', 10)
#         self.string_linear_actuator_publisher_ = self.create_publisher(String, "/linear_actuator", 10)


#         self.subscription_right_ = self.create_subscription(Image, "/camera/right", self.image_right_callback, 10)
#         self.subscription_left_ = self.create_subscription(Image, "/camera/left", self.image_left_callback, 10)

#         # self.subcription_pick_up = self.create_subscription(String, "/pick_up_start". self.)
        

#         self.bridge = CvBridge()

#         self.string_service_ = self.create_service(StringServiceMessage, "/detect_ball_service", self.string_service_callback)

#         self.timer_ = self.create_timer(0.1, self.timer_callback)

#         self.detect_mode = None
#         self.burger_mode = False
#         self.current_left_image_ = None
#         self.current_right_image_ = None

#         self.ball_distance = None
#         self.angle_degrees = None
    
#     def get_package_path(self, package_name):
#         try:
#             package_share_directory = ament_index.get_package_share_directory(package_name)
#             return package_share_directory
#         except Exception as e:
#             print(f"Error: {e}")
#             return None
        
#     def image_left_callback(self, msg):
#         self.current_left_image_ = msg

#     def image_right_callback(self, msg):
#         self.current_right_image_ = msg

#     def string_service_callback(self, request, response):
#         self.get_logger().info(f"Received string service request: {request.detect}")

#         self.detect_mode = request.detect 

#         response.success = True
#         response.message = f"Received and Processed: {request.detect}"
#         return response
    
#     def timer_callback(self):
#         if self.detect_mode == "red_ball":
#             self.detect_and_publish_red_ball_distance()
#         elif self.detect_mode == "blue_ball":
#             self.detect_and_published_blue_ball_distance()

#         if self.ball_distance is not None and self.angle_degrees is not None:
#             self.go_to_goal(self.ball_distance, self.angle_degrees)


#     def publish_left_image(self, image_msg_left):
#         if image_msg_left is not None:
#             self.publisher_left_.publish(image_msg_left)

#     def publish_right_image(self, image_msg_right):
#         if image_msg_right is not None:
#             self.publisher_right_.publish(image_msg_right)
        
#     def detect_and_publish_red_ball_distance(self):
#         if self.current_left_image_ is not None and self.current_right_image_ is not None:
#             left_frame_ = self.bridge.imgmsg_to_cv2(self.current_left_image_, desired_encoding="bgr8")
#             right_frame_ = self.bridge.imgmsg_to_cv2(self.current_right_image_, desired_encoding="bgr8")

#             lower_red = (170, 50, 50)
#             upper_red = (180, 255, 255)

#             image_hsv_left = cv2.cvtColor(left_frame_, cv2.COLOR_BGR2HSV)
#             image_hsv_right = cv2.cvtColor(right_frame_, cv2.COLOR_BGR2HSV)

#             mask_left = cv2.inRange(image_hsv_left, lower_red, upper_red)
#             mask_right = cv2.inRange(image_hsv_right, lower_red, upper_red)

#             cnts_left, _ = cv2.findContours(mask_left, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#             cnts_right, _ = cv2.findContours(mask_right, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#             min_radius = 20  # Lower limit of radius in pixels
#             max_radius = 150  # Upper limit of radius in pixels

#             largest_radius_left = 0
#             left_ball_coordinates = None

#             largest_radius_right = 0
#             right_ball_coordinates = None

#             for c in cnts_left:
#                 ((x, y), r) = cv2.minEnclosingCircle(c)
#                 if min_radius <= r <= max_radius and r > largest_radius_left:
#                     largest_radius_left = r
#                     left_ball_coordinates = (x, y)

#             if left_ball_coordinates is not None:
#                 cv2.circle(left_frame_, (int(left_ball_coordinates[0]), int(left_ball_coordinates[1])), int(largest_radius_left), (0, 255, 0), 2)
#                 cv2.putText(left_frame_, 'x={}, y={}, r={:.1f}'.format(int(left_ball_coordinates[0]), int(left_ball_coordinates[1]), largest_radius_left),
#                             (int(left_ball_coordinates[0]) - 10, int(left_ball_coordinates[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
#                 print(f"Left Frame: Nearest Ball: x={int(left_ball_coordinates[0])}, y={int(left_ball_coordinates[1])}, radius={largest_radius_left}")

#             for c in cnts_right:
#                 ((x, y), r) = cv2.minEnclosingCircle(c)
#                 if min_radius <= r <= max_radius and r > largest_radius_right:
#                     largest_radius_right = r
#                     right_ball_coordinates = (x, y)

#             if right_ball_coordinates is not None:
#                 cv2.circle(right_frame_, (int(right_ball_coordinates[0]), int(right_ball_coordinates[1])), int(largest_radius_right), (0, 255, 0), 2)
#                 cv2.putText(right_frame_, 'x={}, y={}, r={:.1f}'.format(int(right_ball_coordinates[0]), int(right_ball_coordinates[1]), largest_radius_right),
#                             (int(right_ball_coordinates[0]) - 10, int(right_ball_coordinates[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
#                 print(f"Right Frame: Nearest Ball: x={int(right_ball_coordinates[0])}, y={int(right_ball_coordinates[1])}, radius={largest_radius_right}")

#             if left_ball_coordinates is not None and right_ball_coordinates is not None:
#                 x_left = left_ball_coordinates[0]
#                 x_right = right_ball_coordinates[0]
#                 disparity = abs(x_left - x_right)
#                 if disparity != 0:
#                     baseline = 180  # in mm
#                     camera_size = 0.011  # in mm/px
#                     focal_length = 3.6  # in mm
#                     self.ball_distance = (baseline * focal_length) / (disparity * camera_size)

#                     image_center_x = left_frame_.shape[1] / 2  # assuming both images have the same dimensions
#                     angle_left = np.arctan((x_left - image_center_x) * camera_size / focal_length)
#                     angle_right = np.arctan((x_right - image_center_x) * camera_size / focal_length)
#                     angle = (angle_left + angle_right) / 2  # average angle

#                     self.angle_degrees = np.degrees(angle)

#                     cv2.putText(left_frame_, 'Distance: {:.2f} mm'.format(self.ball_distance), (10, left_frame_.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
#                     cv2.putText(left_frame_, 'Angle: {:.2f} degrees'.format(self.angle_degrees), (10, left_frame_.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

#                     cv2.putText(right_frame_, 'Distance: {:.2f} mm'.format(self.ball_distance), (10, right_frame_.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
#                     cv2.putText(right_frame_, 'Angle: {:.2f} degrees'.format(self.angle_degrees), (10, right_frame_.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

#                     print(f"Distance to the ball: {self.ball_distance:.2f} mm, Angle: {self.angle_degrees:.2f} degrees")
#                 else:
#                     self.ball_distance = None
#                     self.angle_degrees = None
#             else:
#                 self.ball_distance = None
#                 self.angle_degrees = None

#             image_msg_left = self.bridge.cv2_to_imgmsg(left_frame_, encoding='bgr8')
#             image_msg_right = self.bridge.cv2_to_imgmsg(right_frame_, encoding='bgr8')
#             self.publish_left_image(image_msg_left)
#             self.publish_right_image(image_msg_right)
#         else:
#             self.get_logger().error('Image Not Found')

#     def detect_and_published_blue_ball_distance(self):
#         if self.current_left_image_ is not None and self.current_right_image_ is not None:
#             left_frame_ = self.bridge.imgmsg_to_cv2(self.current_left_image_, desired_encoding="bgr8")
#             right_frame_ = self.bridge.imgmsg_to_cv2(self.current_right_image_, desired_encoding="bgr8")

#             lower_blue = (110, 50, 50)
#             upper_blue = (130, 255, 255)

#             image_hsv_left = cv2.cvtColor(left_frame_, cv2.COLOR_BGR2HSV)
#             image_hsv_right = cv2.cvtColor(right_frame_, cv2.COLOR_BGR2HSV)

#             mask_left = cv2.inRange(image_hsv_left, lower_blue, upper_blue)
#             mask_right = cv2.inRange(image_hsv_right, lower_blue, upper_blue)

#             cnts_left, _ = cv2.findContours(mask_left, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#             cnts_right, _ = cv2.findContours(mask_right, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#             min_radius = 20  # Lower limit of radius in pixels
#             max_radius = 70  # Upper limit of radius in pixels

#             largest_radius_left = 0
#             left_ball_coordinates = None

#             largest_radius_right = 0
#             right_ball_coordinates = None

#             for c in cnts_left:
#                 ((x, y), r) = cv2.minEnclosingCircle(c)
#                 if min_radius <= r <= max_radius and r > largest_radius_left:
#                     largest_radius_left = r
#                     left_ball_coordinates = (x, y)

#             if left_ball_coordinates is not None:
#                 cv2.circle(left_frame_, (int(left_ball_coordinates[0]), int(left_ball_coordinates[1])), int(largest_radius_left), (0, 255, 0), 2)
#                 cv2.putText(left_frame_, 'x={}, y={}, r={:.1f}'.format(int(left_ball_coordinates[0]), int(left_ball_coordinates[1]), largest_radius_left),
#                             (int(left_ball_coordinates[0]) - 10, int(left_ball_coordinates[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
#                 print(f"Left Frame: Nearest Ball: x={int(left_ball_coordinates[0])}, y={int(left_ball_coordinates[1])}, radius={largest_radius_left}")

#             for c in cnts_right:
#                 ((x, y), r) = cv2.minEnclosingCircle(c)
#                 if min_radius <= r <= max_radius and r > largest_radius_right:
#                     largest_radius_right = r
#                     right_ball_coordinates = (x, y)

#             if right_ball_coordinates is not None:
#                 cv2.circle(right_frame_, (int(right_ball_coordinates[0]), int(right_ball_coordinates[1])), int(largest_radius_right), (0, 255, 0), 2)
#                 cv2.putText(right_frame_, 'x={}, y={}, r={:.1f}'.format(int(right_ball_coordinates[0]), int(right_ball_coordinates[1]), largest_radius_right),
#                             (int(right_ball_coordinates[0]) - 10, int(right_ball_coordinates[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
#                 print(f"Right Frame: Nearest Ball: x={int(right_ball_coordinates[0])}, y={int(right_ball_coordinates[1])}, radius={largest_radius_right}")

#             if left_ball_coordinates is not None and right_ball_coordinates is not None:
#                 x_left = left_ball_coordinates[0]
#                 x_right = right_ball_coordinates[0]
#                 disparity = abs(x_left - x_right)
#                 if disparity != 0:
#                     baseline = 180  # in mm
#                     camera_size = 0.011  # in mm/px
#                     focal_length = 3.6  # in mm
#                     self.ball_distance = (baseline * focal_length) / (disparity * camera_size)

#                     image_center_x = left_frame_.shape[1] / 2  # assuming both images have the same dimensions
#                     angle_left = np.arctan((x_left - image_center_x) * camera_size / focal_length)
#                     angle_right = np.arctan((x_right - image_center_x) * camera_size / focal_length)
#                     angle = (angle_left + angle_right) / 2  # average angle

#                     self.angle_degrees = np.degrees(angle)

#                     cv2.putText(left_frame_, 'Distance: {:.2f} mm'.format(self.ball_distance), (10, left_frame_.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
#                     cv2.putText(left_frame_, 'Angle: {:.2f} degrees'.format(self.angle_degrees), (10, left_frame_.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

#                     cv2.putText(right_frame_, 'Distance: {:.2f} mm'.format(self.ball_distance), (10, right_frame_.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
#                     cv2.putText(right_frame_, 'Angle: {:.2f} degrees'.format(self.angle_degrees), (10, right_frame_.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

#                     print(f"Distance to the ball: {self.ball_distance:.2f} mm, Angle: {self.angle_degrees:.2f} degrees")
#                 else:
#                     self.ball_distance = None
#                     self.angle_degrees = None
#             else:
#                 self.ball_distance = None
#                 self.angle_degrees = None

#             image_msg_left = self.bridge.cv2_to_imgmsg(left_frame_, encoding='bgr8')
#             image_msg_right = self.bridge.cv2_to_imgmsg(right_frame_, encoding='bgr8')
#             self.publish_left_image(image_msg_left)
#             self.publish_right_image(image_msg_right)
#         else:
#             self.get_logger().error('Image Not Found')

#     def go_to_goal(self, ball_distance, angle_degrees):
#         # Command to control the robot
#         twist = Twist()

#         # Distance threshold and angle threshold to stop the robot
#         distance_threshold = 200.0  # in mm
#         angle_threshold = 5.0  # in degrees


#         if abs(angle_degrees) > angle_threshold:
#             # Rotate to align with the ball
#             twist.angular.z = -150.0 if angle_degrees < 0 else 150.0  # Adjust rotation speed as needed
#         else:
#             twist.angular.z = 0.0  # Go straight if within angle threshold
#             if ball_distance > distance_threshold:
#                 # Move forward
#                 twist.linear.x = 200.0  # Adjust speed as needed

#             else:
#                 twist.linear.x = 0.0  # Stop if within distance threshold
#                 twist.angular.z = 0.0  # Stop rotating

#         # Publish the twist message
#         self.cmd_vel_publisher_.publish(twist)

# def main(args=None):
#     rclpy.init(args=args)
#     node = DetectionAndDistance()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from custom_interfaces.srv import StringServiceMessage
import cv2
from cv_bridge import CvBridge
import ament_index_python.packages as ament_index
import os
import numpy as np

class DetectionAndDistance(Node):

    def __init__(self):
        super().__init__("detection_and_distance_node")

        self.pkg_path = self.get_package_path("object_detection")

        self.publisher_left_ = self.create_publisher(Image, "/left_detection", 10)
        self.publisher_right_ = self.create_publisher(Image, "/right_detection", 10)
        self.cmd_vel_publisher_ = self.create_publisher(Twist, '/cmd_vel/ball', 10)
        self.string_linear_actuator_publisher_ = self.create_publisher(Twist, "/cmd_vel/ball", 10)

        self.subscription_right_ = self.create_subscription(Image, "/camera/right", self.image_right_callback, 10)
        self.subscription_left_ = self.create_subscription(Image, "/camera/left", self.image_left_callback, 10)
        self.subscription_pick_up_state_ = self.create_subscription(String, "/pick_up_state", self.picked_up_callback, 10)

        self.bridge = CvBridge()

        self.string_service_ = self.create_service(StringServiceMessage, "/detect_ball_service", self.string_service_callback)

        self.timer_ = self.create_timer(0.1, self.timer_callback)

        self.detect_mode = None
        self.burger_mode = False
        self.current_left_image_ = None
        self.current_right_image_ = None

        self.ball_distance = None
        self.angle_degrees = None
        self.reached_goal = False
    
    def get_package_path(self, package_name):
        try:
            package_share_directory = ament_index.get_package_share_directory(package_name)
            return package_share_directory
        except Exception as e:
            print(f"Error: {e}")
            return None
        
    def image_left_callback(self, msg):
        self.current_left_image_ = msg

    def image_right_callback(self, msg):
        self.current_right_image_ = msg

    def picked_up_callback(self, data):
        self.pick_up_state = data
        if self.pick_up_state == "picked_up":
            pass


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

        if self.ball_distance is not None and self.angle_degrees is not None:
            self.go_to_goal(self.ball_distance, self.angle_degrees)


    def publish_left_image(self, image_msg_left):
        if image_msg_left is not None:
            self.publisher_left_.publish(image_msg_left)

    def publish_right_image(self, image_msg_right):
        if image_msg_right is not None:
            self.publisher_right_.publish(image_msg_right)
        
    def detect_and_publish_red_ball_distance(self):
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

            min_radius = 20  # Lower limit of radius in pixels
            max_radius = 250  # Upper limit of radius in pixels

            largest_radius_left = 0
            left_ball_coordinates = None

            largest_radius_right = 0
            right_ball_coordinates = None

            for c in cnts_left:
                ((x, y), r) = cv2.minEnclosingCircle(c)
                if min_radius <= r <= max_radius and r > largest_radius_left:
                    largest_radius_left = r
                    left_ball_coordinates = (x, y)

            if left_ball_coordinates is not None:
                cv2.circle(left_frame_, (int(left_ball_coordinates[0]), int(left_ball_coordinates[1])), int(largest_radius_left), (0, 255, 0), 2)
                cv2.putText(left_frame_, 'x={}, y={}, r={:.1f}'.format(int(left_ball_coordinates[0]), int(left_ball_coordinates[1]), largest_radius_left),
                            (int(left_ball_coordinates[0]) - 10, int(left_ball_coordinates[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                print(f"Left Frame: Nearest Ball: x={int(left_ball_coordinates[0])}, y={int(left_ball_coordinates[1])}, radius={largest_radius_left}")

            for c in cnts_right:
                ((x, y), r) = cv2.minEnclosingCircle(c)
                if min_radius <= r <= max_radius and r > largest_radius_right:
                    largest_radius_right = r
                    right_ball_coordinates = (x, y)

            if right_ball_coordinates is not None:
                cv2.circle(right_frame_, (int(right_ball_coordinates[0]), int(right_ball_coordinates[1])), int(largest_radius_right), (0, 255, 0), 2)
                cv2.putText(right_frame_, 'x={}, y={}, r={:.1f}'.format(int(right_ball_coordinates[0]), int(right_ball_coordinates[1]), largest_radius_right),
                            (int(right_ball_coordinates[0]) - 10, int(right_ball_coordinates[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                print(f"Right Frame: Nearest Ball: x={int(right_ball_coordinates[0])}, y={int(right_ball_coordinates[1])}, radius={largest_radius_right}")

            if left_ball_coordinates is not None and right_ball_coordinates is not None:
                x_left = left_ball_coordinates[0]
                x_right = right_ball_coordinates[0]
                disparity = abs(x_left - x_right)
                if disparity != 0:
                    baseline = 180  # in mm
                    camera_size = 0.011  # in mm/px
                    focal_length = 3.6  # in mm
                    self.ball_distance = (baseline * focal_length) / (disparity * camera_size)

                    image_center_x = left_frame_.shape[1] / 2  # assuming both images have the same dimensions
                    angle_left = np.arctan((x_left - image_center_x) * camera_size / focal_length)
                    angle_right = np.arctan((x_right - image_center_x) * camera_size / focal_length)
                    angle = (angle_left + angle_right) / 2  # average angle

                    self.angle_degrees = np.degrees(angle)

                    cv2.putText(left_frame_, 'Distance: {:.2f} mm'.format(self.ball_distance), (10, left_frame_.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    cv2.putText(left_frame_, 'Angle: {:.2f} degrees'.format(self.angle_degrees), (10, left_frame_.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

                    cv2.putText(right_frame_, 'Distance: {:.2f} mm'.format(self.ball_distance), (10, right_frame_.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    cv2.putText(right_frame_, 'Angle: {:.2f} degrees'.format(self.angle_degrees), (10, right_frame_.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

                    print(f"Distance to the ball: {self.ball_distance:.2f} mm, Angle: {self.angle_degrees:.2f} degrees")
                else:
                    self.ball_distance = None
                    self.angle_degrees = None
            else:
                self.ball_distance = None
                self.angle_degrees = None

            image_msg_left = self.bridge.cv2_to_imgmsg(left_frame_, encoding='bgr8')
            image_msg_right = self.bridge.cv2_to_imgmsg(right_frame_, encoding='bgr8')
            self.publish_left_image(image_msg_left)
            self.publish_right_image(image_msg_right)
        else:
            self.get_logger().error('Image Not Found')

            
            self.publish_left_image(self.bridge.cv2_to_imgmsg(left_frame_, encoding="bgr8"))
            self.publish_right_image(self.bridge.cv2_to_imgmsg(right_frame_, encoding="bgr8"))

    def detect_and_published_blue_ball_distance(self):
        if self.current_left_image_ is not None and self.current_right_image_ is not None:
            left_frame_ = self.bridge.imgmsg_to_cv2(self.current_left_image_, desired_encoding="bgr8")
            right_frame_ = self.bridge.imgmsg_to_cv2(self.current_right_image_, desired_encoding="bgr8")

            lower_blue = (100, 150, 0)
            upper_blue = (140, 255, 255)

            image_hsv_left = cv2.cvtColor(left_frame_, cv2.COLOR_BGR2HSV)
            image_hsv_right = cv2.cvtColor(right_frame_, cv2.COLOR_BGR2HSV)

            mask_left = cv2.inRange(image_hsv_left, lower_blue, upper_blue)
            mask_right = cv2.inRange(image_hsv_right, lower_blue, upper_blue)

            cnts_left, _ = cv2.findContours(mask_left, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts_right, _ = cv2.findContours(mask_right, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            min_radius = 20  # Lower limit of radius in pixels
            max_radius = 40  # Upper limit of radius in pixels

            largest_radius_left = 0
            left_ball_coordinates = None

            largest_radius_right = 0
            right_ball_coordinates = None

            for c in cnts_left:
                ((x, y), r) = cv2.minEnclosingCircle(c)
                if min_radius <= r <= max_radius and r > largest_radius_left:
                    largest_radius_left = r
                    left_ball_coordinates = (x, y)

            if left_ball_coordinates is not None:
                cv2.circle(left_frame_, (int(left_ball_coordinates[0]), int(left_ball_coordinates[1])), int(largest_radius_left), (0, 255, 0), 2)
                cv2.putText(left_frame_, 'x={}, y={}, r={:.1f}'.format(int(left_ball_coordinates[0]), int(left_ball_coordinates[1]), largest_radius_left),
                            (int(left_ball_coordinates[0]) - 10, int(left_ball_coordinates[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                print(f"Left Frame: Nearest Ball: x={int(left_ball_coordinates[0])}, y={int(left_ball_coordinates[1])}, radius={largest_radius_left}")

            for c in cnts_right:
                ((x, y), r) = cv2.minEnclosingCircle(c)
                if min_radius <= r <= max_radius and r > largest_radius_right:
                    largest_radius_right = r
                    right_ball_coordinates = (x, y)

            if right_ball_coordinates is not None:
                cv2.circle(right_frame_, (int(right_ball_coordinates[0]), int(right_ball_coordinates[1])), int(largest_radius_right), (0, 255, 0), 2)
                cv2.putText(right_frame_, 'x={}, y={}, r={:.1f}'.format(int(right_ball_coordinates[0]), int(right_ball_coordinates[1]), largest_radius_right),
                            (int(right_ball_coordinates[0]) - 10, int(right_ball_coordinates[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                print(f"Right Frame: Nearest Ball: x={int(right_ball_coordinates[0])}, y={int(right_ball_coordinates[1])}, radius={largest_radius_right}")

            if left_ball_coordinates is not None and right_ball_coordinates is not None:
                x_left = left_ball_coordinates[0]
                x_right = right_ball_coordinates[0]
                disparity = abs(x_left - x_right)
                if disparity != 0:
                    baseline = 180  # in mm
                    camera_size = 0.011  # in mm/px
                    focal_length = 3.6  # in mm
                    self.ball_distance = (baseline * focal_length) / (disparity * camera_size)

                    image_center_x = left_frame_.shape[1] / 2  # assuming both images have the same dimensions
                    angle_left = np.arctan((x_left - image_center_x) * camera_size / focal_length)
                    angle_right = np.arctan((x_right - image_center_x) * camera_size / focal_length)
                    angle = (angle_left + angle_right) / 2  # average angle

                    self.angle_degrees = np.degrees(angle)

                    cv2.putText(left_frame_, 'Distance: {:.2f} mm'.format(self.ball_distance), (10, left_frame_.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    cv2.putText(left_frame_, 'Angle: {:.2f} degrees'.format(self.angle_degrees), (10, left_frame_.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

                    cv2.putText(right_frame_, 'Distance: {:.2f} mm'.format(self.ball_distance), (10, right_frame_.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    cv2.putText(right_frame_, 'Angle: {:.2f} degrees'.format(self.angle_degrees), (10, right_frame_.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

                    print(f"Distance to the ball: {self.ball_distance:.2f} mm, Angle: {self.angle_degrees:.2f} degrees")
                else:
                    self.ball_distance = None
                    self.angle_degrees = None
            else:
                self.ball_distance = None
                self.angle_degrees = None
            
            self.publish_left_image(self.bridge.cv2_to_imgmsg(left_frame_, encoding="bgr8"))
            self.publish_right_image(self.bridge.cv2_to_imgmsg(right_frame_, encoding="bgr8"))

    def go_to_goal(self, distance, angle):
        cmd_vel_msg = Twist()

        goal_distance = 200.0  # desired distance to the ball in mm
        goal_angle = 0.0  # desired angle in degrees

        k_linear = 0.001  # proportional constant for linear velocity
        k_angular = 0.01  # proportional constant for angular velocity

        error_distance = distance - goal_distance
        error_angle = angle - goal_angle

        linear_speed = k_linear * error_distance
        angular_speed = k_angular * error_angle

        cmd_vel_msg.linear.x = linear_speed
        cmd_vel_msg.angular.z = angular_speed

        twist = Twist()

        # Distance threshold and angle threshold to stop the robot
        distance_threshold = 250.0  # in mm
        angle_threshold = 4.0  # in degrees


        if abs(self.angle_degrees) > angle_threshold:
            # Rotate to align with the ball
            twist.angular.z = -150.0 if self.angle_degrees < 0 else 150.0  # Adjust rotation speed as needed
        else:
            twist.angular.z = 0.0  # Go straight if within angle threshold
            if self.ball_distance > distance_threshold:
                # Move forward
                twist.linear.x = 200.0  # Adjust speed as needed

            else:
                twist.linear.x = 0.0  # Stop if within distance threshold
                twist.angular.z = 0.0  # Stop rotating

        # Publish the twist message
        self.cmd_vel_publisher_.publish(twist)

        # self.cmd_vel_publisher_.publish(cmd_vel_msg)

        if abs(error_distance) < 10 and abs(error_angle) < 5:
            # Robot is considered to have reached the goal if the error is within thresholds
            self.reached_goal = True
            self.get_logger().info('Reached goal, stopping and moving linear actuator.')
            self.move_linear_actuator()
        else:
            self.reached_goal = False

    def move_linear_actuator(self):
        # Publish the message to move the linear actuator
        actuator_msg = Twist()
        actuator_msg.linear.y = 65535.0
        self.string_linear_actuator_publisher_.publish(actuator_msg)

def main(args=None):
    rclpy.init(args=args)
    detection_and_distance_node = DetectionAndDistance()
    rclpy.spin(detection_and_distance_node)
    detection_and_distance_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
