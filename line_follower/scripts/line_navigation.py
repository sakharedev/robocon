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
ANGULAR_SPEED = 200.0
LINEAR_SPEED = 20.0
flag = 0


# Proportional constant to be applied on speed while turning
# (Multiplied by the error value)
KP = 1.5 / 1000

class Image_Subscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(Image, 'camera_right/image', self.listener_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel/line', 10)
        self.bridge = CvBridge()


    def visualize_roi(self, image):
        # Create a copy of the image to draw on
        image_copy = image.copy()
        height, width = image.shape[:2]

        # Define points for the bottom center polygon
        # polygon = np.array([
        #     (int(width * 0.5), height),  # Bottom left
        #     (int(width * 0.5), height),  # Bottom right
        #     (width * 5 // 8, int(height * 0.5)),  # Top right
        #     (width * 3 // 8, int(height * 0.5))  # Top left
        # ], dtype=np.int32)

        polygon = np.array([
            (460, 479),  # Bottom left
            (152, 479),  # Bottom right
            (100, 234),  # Top right
            (500, 234)  # Top left
        ], dtype=np.int32)
        

        # Draw the polygon on the image
        cv2.polylines(image_copy, [polygon], isClosed=True, color=(0, 255, 0), thickness=5)

        # Save the image with the drawn polygon
        cv2.imwrite('roi_visualization.jpg', image_copy)
        cv2.imshow('ROI Visualization', image_copy) 

    def listener_callback(self, msg):
        current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv_image = cv2.cvtColor(current_image, cv2.COLOR_BGR2HSV)
        sensitivity = 60
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

        diff = (float(error) * -KP)
        self.get_logger().info(f"diff: {diff} Error: {(float(error))}")
        cmd.angular.z = ANGULAR_SPEED

        #right and left turn logic
        if ((float(error)) * (-KP) < (-45.0)):
            cmd.angular.z = ANGULAR_SPEED
        elif((float(error)) * (-KP) > 45.0):
            cmd.angular.z = -ANGULAR_SPEED
        else:
            cmd.linear.x = 0.0


        self.publisher.publish(cmd)
        cv2.imshow('White Segment Image', white_segment_image)
        self.visualize_roi(current_image)
        cv2.waitKey(1)

    # def get_contour_data(self, mask):
    #     MIN_AREA_TRACK = 1000
    #     # MAX_AREA_TRACK = 120
    #     contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    #     line = {}
    #     for contour in contours:
    #         M = cv2.moments(contour)
    #         if MIN_AREA_TRACK < M['m00']:
    #             cx = int(M['m10'] / M['m00'])
    #             cy = int(M['m01'] / M['m00'])
    #             line['x'] = cx
    #             line['y'] = cy

    #     return line
    def create_roi_mask(self, shape):
        mask = np.zeros(shape[:2], dtype=np.uint8)
        height, width = shape[:2]

        # Define points for the bottom center polygon
        # Adjust these points as needed to focus on the desired area
        # polygon = np.array([[
        #     (int(width * 0.5), height),  # Bottom left
        #     (int(width * 0.5), height),  # Bottom right
        #     (width * 5 // 8, int(height * 0.5)),  # Top right
        #     (width * 3 // 8, int(height * 0.5))  # Top left
        # ]], dtype=np.int32)

        polygon = np.array([
            (460, 479),  # Bottom left
            (152, 479),  # Bottom right
            (100, 234),  # Top right
            (500, 234)  # Top left
        ], dtype=np.int32)

        # Fill the polygon with white
        cv2.fillPoly(mask, [polygon], 255)
        return mask

    def get_contour_data(self, mask):
        # Ensure the shape is correctly obtained from the mask
        MIN_AREA_TRACK = 1000
        shape = mask.shape  # This should be a tuple like (height, width, channels)
        # Pass the correct shape to create_roi_mask
        roi_mask = self.create_roi_mask(shape)
        masked_image = cv2.bitwise_and(mask, mask, mask=roi_mask)

        contours, _ = cv2.findContours(masked_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

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
