#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge
import numpy as np

LINEAR_SPEED = 0.2  # Reduced for smoother movement
ANGULAR_SPEED = 0.5  # Reduced for smoother movement

# PID constants
KP = 0.01
KI = 0.0001
KD = 0.005

class Image_Subscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(Image, 'camera_right/image', self.listener_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()
        self.last_error = 0
        self.integral = 0

    def listener_callback(self, msg):
        current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv_image = cv2.cvtColor(current_image, cv2.COLOR_BGR2HSV)
        
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 30, 255])

        white_mask = cv2.inRange(hsv_image, lower_white, upper_white)
        
        roi_mask = self.create_roi_mask(white_mask.shape)
        masked_image = cv2.bitwise_and(white_mask, white_mask, mask=roi_mask)

        line = self.get_contour_data(masked_image)
        
        cmd = Twist()
        height, width = masked_image.shape
        
        if line:
            error = line['x'] - width // 2
            self.integral += error
            derivative = error - self.last_error
            
            angular_z = KP * error + KI * self.integral + KD * derivative
            
            cmd.linear.x = LINEAR_SPEED
            cmd.angular.z = -angular_z  # Negative because positive error means turn left
            
            self.last_error = error
            
            cv2.circle(current_image, (line['x'], line['y']), 5, (0, 0, 255), -1)
        else:
            # If line is lost, stop and rotate to find it
            cmd.linear.x = 0.0
            cmd.angular.z = ANGULAR_SPEED
            self.integral = 0  # Reset integral when line is lost

        self.publisher.publish(cmd)
        
        cv2.imshow('Processed Image', current_image)
        cv2.waitKey(1)

    def create_roi_mask(self, shape):
        mask = np.zeros(shape, dtype=np.uint8)
        height, width = shape
        polygon = np.array([
            (0, height),
            (width, height),
            (width, height // 2),
            (0, height // 2)
        ], dtype=np.int32)
        cv2.fillPoly(mask, [polygon], 255)
        return mask

    def get_contour_data(self, masked_image):
        contours, _ = cv2.findContours(masked_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                return {'x': cx, 'y': cy}
        return None

def main(args=None):
    rclpy.init(args=args)
    try:
        image_subscriber = Image_Subscriber()
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()