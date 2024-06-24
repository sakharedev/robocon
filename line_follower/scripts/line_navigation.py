#! /usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from geometry_msgs.msg import Twist

# import cv2
# from cv_bridge import CvBridge
# import numpy as np

# LINEAR_SPEED = 0.2

# # Proportional constant to be applied on speed while turning
# # (Multiplied by the error value)
# KP = 1.5/100

# class Image_Subscriber(Node):
#     def __init__(self):
#         super().__init__('image_subscriber')
#         self.subscription = self.create_subscription(Image, 'camera_right/image', self.listner_callback, 10)
#         self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.bridge = CvBridge()

#     def listner_callback(self, msg):
#         current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         hsv_image = cv2.cvtColor(current_image, cv2.COLOR_BGR2HSV)
#         cv2.imshow('Image', hsv_image)
#         sensitivity = 40
#         lower_white = np.array([0,0,255-sensitivity])
#         upper_white = np.array([255,sensitivity,255])

#         white_mask = cv2.inRange(hsv_image, lower_white, upper_white)
#         white_segment_image = cv2.bitwise_and(current_image, current_image, mask=white_mask)

#         line = self.get_contour_data(white_mask)
#         # if line:
#         #     cv2.circle(white_segment_image, (line['x'], line['y']), 5, (0, 0, 255), 7)
        
#         error = None

#         cmd = Twist()
#         _, width, _ = white_segment_image.shape
#         if line:
#             x = line['x']

#             error = x - width//2

#             cmd.linear.x = LINEAR_SPEED
#             cv2.circle(white_segment_image, (line['x'], line['y']), 5, (0, 0, 255), -7)

#         elif error is None:
#             error = 0
#             cmd.angular.z = 0.2

#         cmd.angular.z = float(error) * -KP
#         print("Error: {} | Angular Z: {}".format(error, cmd.angular.z))

#         self.publisher.publish(cmd)
            

#         cv2.imshow('White Segment Image', white_segment_image)
#         cv2.waitKey(1)

#     def get_contour_data(self, mask):
        
#         MIN_AREA_TRACK = 1000
#         # MAX_AREA_TRACK = 6000
#         contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#         line = {}

#         for contour in contours:
#             M = cv2.moments(contour)

#             if(M['m00'] > MIN_AREA_TRACK):
#                 cx = int(M['m10']/M['m00'])
#                 cy = int(M['m01']/M['m00'])
#                 line['x'] = cx
#                 line['y'] = cy

#         return line

# def main(args=None):
#     rclpy.init(args=args)
#     try:
#         image_subscriber = Image_Subscriber()
#         rclpy.spin(image_subscriber)
#     except KeyboardInterrupt:
#         pass

# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

import cv2
from cv_bridge import CvBridge
import numpy as np

LINEAR_SPEED = 0.2

# Proportional constant to be applied on speed while turning
# (Multiplied by the error value)
KP = 1.5 / 100

class Image_Subscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(Image, 'camera_right/image', self.listener_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv_image = cv2.cvtColor(current_image, cv2.COLOR_BGR2HSV)
        sensitivity = 75
        lower_white = np.array([0, 0, 255 - sensitivity])
        upper_white = np.array([255, sensitivity, 255])

        white_mask = cv2.inRange(hsv_image, lower_white, upper_white)
        white_segment_image = cv2.bitwise_and(current_image, current_image, mask=white_mask)

        line = self.get_contour_data(white_mask)
        
        cmd = Twist()
        _, width, _ = white_segment_image.shape
        error = None

        if line:
            x = line['x']
            error = x - width // 2

            cmd.linear.x = LINEAR_SPEED
            cv2.circle(white_segment_image, (line['x'], line['y']), 5, (0, 0, 255), -1)
        else:
            # Handle intersections or loss of line
            error = self.handle_intersection(white_mask, white_segment_image, width)

        cmd.angular.z = float(error) * -KP
        self.publisher.publish(cmd)
        cv2.imshow('White Segment Image', white_segment_image)
        cv2.waitKey(1)

    def get_contour_data(self, mask):
        MIN_AREA_TRACK = 1000
        # MAX_AREA_TRACK = 120
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        line = {}
        for contour in contours:
            M = cv2.moments(contour)
            if MIN_AREA_TRACK < M['m00']:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                line['x'] = cx
                line['y'] = cy

        return line

    def handle_intersection(self, mask, image, width):
        # Intersection detection logic
        search_top = int(3 * image.shape[0] / 4)
        search_bot = int(3 * image.shape[0] / 4 + 20)
        mask[0:search_top, 0:width] = 0
        mask[search_bot:image.shape[0], 0:width] = 0

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

                if cx > width // 2:
                    # Right turn logic
                    error = cx - width // 2
                else:
                    # Continue straight
                    error = 0

                cv2.circle(image, (cx, cy), 5, (0, 255, 0), -1)
                return error

        return 0

def main(args=None):
    rclpy.init(args=args)
    try:
        image_subscriber = Image_Subscriber()
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
