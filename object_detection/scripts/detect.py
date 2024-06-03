#!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# import numpy as np
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2

# class ObjectDetectionNode(Node):
#     def __init__(self):
#         super().__init__('ObjectDetectionNode')
#         self.create_subscription(Image, '/image', self.image_callback, 10)
#         self.bridge = CvBridge()
# # this is a fuction which detects an object whose hsv codes are between the given values h = 358 to 290 , s = 82 to 49, v 92 to 34
#     # def image_callback(self, msg):
#     #     cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#     #     hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
#     #     # lower2 = np.array([160,100,20])
#     #     # upper2 = np.array([179,255,255])
#     #     lower2 = np.array([159, 50, 70])
#     #     upper2 = np.array([180, 255, 255])
#     #     mask = cv2.inRange(hsv, lower2, upper2)
#     #     # cv2.imshow('image', cv_image)
#     #     cv2.imshow('mask', mask)
#     #     cv2.imshow('frame', cv_image)
#     #     # print if detected and no if not detected
#     #     if cv2.countNonZero(mask) > 0:
#     #         print('Detected')
#     #     else:
#     #         print('No Object Detected')
            
#     #     cv2.waitKey(3)

#     def image_callback(self, msg):
#         cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
#         lower2 = np.array([159, 50, 70])
#         upper2 = np.array([180, 255, 255])
#         mask = cv2.inRange(hsv, lower2, upper2)

#         # Apply Hough Circle Transform
#         circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=0, maxRadius=0)

#         if circles is not None:
#             circles = np.uint16(np.around(circles))
#             for i in circles[0,:]:
#                 # draw the outer circle
#                 cv2.circle(cv_image,(i[0],i[1]),i[2],(0,255,0),2)
#                 # draw the center of the circle
#                 cv2.circle(cv_image,(i[0],i[1]),2,(0,0,255),3)
#             print('Round Object Detected')
#         else:
#             print('No Round Object Detected')

#         cv2.imshow('detected circles', cv_image)
#         cv2.waitKey(3)



# def main(args=None):
#     rclpy.init(args=args)
#     node = ObjectDetectionNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()



import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class BallDetectorNode(Node):
    def __init__(self):
        super().__init__('ball_detector_node')
        self.subscription = self.create_subscription(
            Image,
            '/image',
            self.image_callback,
            10
        )
        self.publisher_ = self.create_publisher(Image, 'ball_detected_image', 10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Convert BGR image to HSV
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define the red color range in HSV
        lower_red = np.array([0, 70, 50])
        upper_red = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv_image, lower_red, upper_red)

        lower_red = np.array([170, 70, 50])
        upper_red = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv_image, lower_red, upper_red)

        # Combine the masks
        mask = mask1 + mask2

        # Morphological operations to remove noise
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            # Check if the contour is a circle
            area = cv2.contourArea(contour)
            ((x, y), radius) = cv2.minEnclosingCircle(contour)
            circularity = 4 * np.pi * (area / (cv2.arcLength(contour, True) ** 2))
            if circularity > 0.7 and radius > 10:  # Adjust circularity and radius threshold as needed
                # Draw the circle on the original image
                cv2.circle(cv_image, (int(x), int(y)), int(radius), (0, 255, 0), 2)
                cv2.putText(cv_image, 'Red Ball', (int(x-radius), int(y-radius)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)


        # cv2.imshow('Ball Detector', cv_image)
        # cv2.imshow('Mask', mask)

        # print detected is detected and no if not detected
        if len(contours) > 0:
            print('Detected')
        else:
            print('No Ball Detected')
        # Publish the result
        result_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
        self.publisher_.publish(result_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BallDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
