#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cv_bridge import CvBridge
from custom_interfaces.srv import StringServiceMessage
import cv2
import numpy as np
from ultralytics import YOLO
import time

class DetectionAndDistance(Node):

    def __init__(self):
        super().__init__("detection_and_distance_node")

        self.publisher_left_ = self.create_publisher(Image, "/left_detection", 10)
        self.publisher_right_ = self.create_publisher(Image, "/right_detection", 10)
        self.cmd_vel_publisher_ = self.create_publisher(Twist, '/cmd_vel/ball', 10)
        self.linear_actuator_publisher_ = self.create_publisher(Twist, "/cmd_vel/ball", 10)

        self.subscription_right_ = self.create_subscription(Image, "/camera/right", self.image_right_callback, 10)
        self.subscription_left_ = self.create_subscription(Image, "/camera/left", self.image_left_callback, 10)
        self.subscription_state_ = self.create_subscription(String, "/state", self.state_callback, 10)

        self.subscription_silo_ = self.create_subscription(Image, "/camera/silo", self.silo_image_callback, 10)

        self.bridge = CvBridge()

        self.string_service_ = self.create_service(StringServiceMessage, "/detect_ball_service", self.string_service_callback)

        self.timer_ = self.create_timer(0.1, self.timer_callback)

        self.detect_mode = None
        self.burger_mode = False
        self.current_left_image_ = None
        self.current_right_image_ = None
        self.current_silo_image_ = None

        self.ball_distance = None
        self.angle_degrees = None
        self.reached_goal = False
        
        self.state = "ball_detection"
        
        # Initialize YOLO model
        self.model = YOLO("/home/mecha/robocon/src/robocon/object_detection/robocon-chudaapv14(silo).pt")

        # Calibration variables
        self.focal_length = None
        self.calibration_mode = False
        self.calibration_image = None

    def image_left_callback(self, msg):
        self.current_left_image_ = msg

    def image_right_callback(self, msg):
        self.current_right_image_ = msg

    def silo_image_callback(self, msg):
        self.current_silo_image_ = msg

    def state_callback(self, data):
        pass

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
            
            self.publish_left_image(self.bridge.cv2_to_imgmsg(left_frame_, encoding="bgr8"))
            self.publish_right_image(self.bridge.cv2_to_imgmsg(right_frame_, encoding="bgr8"))


    def string_service_callback(self, request, response):
        self.get_logger().info(f"Received string service request: {request.detect}")
        self.detect_mode = request.detect 
        response.success = True
        response.message = f"Received and Processed: {request.detect}"
        return response
    
    def timer_callback(self):
        if self.state == "ball_detection":
            self.ball_detection_state()
        elif self.state == "silo_tracking":
            self.silo_tracking_state()

    def ball_detection_state(self):
        if self.detect_mode == "red_ball":
            self.detect_and_publish_red_ball_distance()
        elif self.detect_mode == "blue_ball":
            self.detect_and_published_blue_ball_distance()

        if self.ball_distance is not None and self.angle_degrees is not None:
            self.go_to_goal(self.ball_distance, self.angle_degrees)

    def silo_tracking_state(self):
        if self.current_silo_image_ is not None:
            frame = self.bridge.imgmsg_to_cv2(self.current_silo_image_, desired_encoding="bgr8")

            if self.calibration_mode:
                self.calibrate_focal_length(frame)
            elif self.focal_length is not None:
                self.track_silo(frame)
            else:
                cv2.putText(frame, "Press 'c' to start calibration",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            display_frame = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_right_.publish(display_frame)

    def calibrate_focal_length(self, frame):
        pixel_width, _, obj_class, top_left, bottom_right = self.detect_object(frame)
        if pixel_width and obj_class == CALIBRATION_CLASS:
            cv2.rectangle(frame, top_left, bottom_right, (0, 255, 0), 2)
            width_text = f"Width: {pixel_width:.2f} pixels"
            cv2.putText(frame, width_text, (top_left[0], bottom_right[1] + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            self.focal_length = self.calibrate_focal_length(KNOWN_DISTANCE, KNOWN_WIDTH, pixel_width)
            self.calibration_image = frame.copy()
            self.get_logger().info(f"Calibration complete. Focal length: {self.focal_length}")
            self.calibration_mode = False
        else:
            cv2.putText(frame, f"Place a {CALIBRATION_CLASS} at {KNOWN_DISTANCE}m and press 'c'",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    def track_silo(self, frame):
        pixel_width, _, obj_class, top_left, bottom_right = self.detect_object(frame)
        if pixel_width is not None and obj_class == CALIBRATION_CLASS:
            distance = self.estimate_distance(self.focal_length, KNOWN_WIDTH, pixel_width)
            cv2.putText(frame, f"Distance to {obj_class}: {distance:.2f}m",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        else:
            cv2.putText(frame, f"Place a {CALIBRATION_CLASS} in view",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    def detect_object(self, frame):
        results = self.model(frame)
        detections = results[0].boxes.data
        if len(detections) > 0:
            x1, y1, x2, y2, conf, class_id = detections[0]
            width = x2 - x1
            height = y2 - y1
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            label = f"{self.model.names[int(class_id)]} {conf:.2f}"
            cv2.putText(frame, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            return width.item(), height.item(), self.model.names[int(class_id)], (int(x1), int(y1)), (int(x2), int(y2))
        return None, None, None, None, None

    def estimate_distance(self, focal_length, real_width, pixel_width):
        return (real_width * focal_length) / pixel_width

    def calibrate_focal_length(self, known_distance, real_width, pixel_width):
        return (pixel_width * known_distance) / real_width

    def go_to_goal(self, distance, angle):
        cmd_vel_msg = Twist()
        goal_distance = 200.0
        goal_angle = 0.0
        k_linear = 0.001
        k_angular = 0.01
        error_distance = distance - goal_distance
        error_angle = angle - goal_angle
        linear_speed = k_linear * error_distance
        angular_speed = k_angular * error_angle
        cmd_vel_msg.linear.x = linear_speed
        cmd_vel_msg.angular.z = angular_speed
        twist = Twist()
        distance_threshold = 250.0
        angle_threshold = 4.0
        if abs(self.angle_degrees) > angle_threshold:
            twist.angular.z = -150.0 if self.angle_degrees < 0 else 150.0
        else:
            twist.angular.z = 0.0
            if self.ball_distance > distance_threshold:
                twist.linear.x = 200.0
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
        self.cmd_vel_publisher_.publish(twist)
        if self.ball_distance < 250 and self.angle_degrees < 5:
            self.reached_goal = True
            self.get_logger().info('Reached goal, stopping and moving linear actuator.')
            self.move_linear_actuator()
            self.state = "silo_tracking"
        else:
            self.reached_goal = False

    def move_linear_actuator(self):
        actuator_msg = Twist()
        actuator_msg.linear.z = 65535.0
        self.cmd_vel_publisher_.publish(actuator_msg)
        time.sleep(10)
        actuator_msg.linear.z = 0.0
        actuator_msg.linear.y = 400.0
        self.cmd_vel_publisher_.publish(actuator_msg)

def main(args=None):
    rclpy.init(args=args)
    detection_and_distance_node = DetectionAndDistance()
    rclpy.spin(detection_and_distance_node)
    detection_and_distance_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
