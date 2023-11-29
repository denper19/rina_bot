# Sample code to read images from the camera in ROS2 
# Use for image processing and practice
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge # ros package for converting between ROS2 and OpenCV messages
import numpy as np
import matplotlib.pyplot as plt

class ImageReader(Node):

    def __init__(self):
        super().__init__('ImageReader')
        self.bridge = CvBridge() # init the CVbridge to be used later on
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.callback,
            1
        )
        self.subscription # prevents unused variable warning
    
    def callback(self, msg):

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        original_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        block_size=3
        ksize=3
        k=0.04
        threshold=0.01
        img = np.float32(original_image) / 255.0  # Convert image to float and normalize

        # Compute gradients using Sobel operator
        Ix = cv2.Sobel(img, cv2.CV_64F, 1, 0, ksize=ksize)
        Iy = cv2.Sobel(img, cv2.CV_64F, 0, 1, ksize=ksize)

        # Compute products of derivatives
        Ixx = Ix * Ix
        Iyy = Iy * Iy
        Ixy = Ix * Iy

        # Compute sums of products of derivatives at each pixel
        Sxx = cv2.boxFilter(Ixx, -1, (block_size, block_size))
        Syy = cv2.boxFilter(Iyy, -1, (block_size, block_size))
        Sxy = cv2.boxFilter(Ixy, -1, (block_size, block_size))

        # Calculate the determinant and trace of the covariance matrix
        det_M = (Sxx * Syy) - (Sxy ** 2)
        trace_M = Sxx + Syy

        # Harris Corner Response Function
        R = det_M - k * (trace_M ** 2)

        # Threshold the response function to identify corners
        corners = np.zeros_like(img)
        corners[R > threshold * R.max()] = 1.0

        # Display the original image and detected corners
        cv2.imshow('Original Image', img)
        cv2.imshow('Harris Corner Detection', corners)

        # Wait for a key press and close the windows
        cv2.waitKey(0)
        cv2.destroyAllWindows()



def main(args=None):

    rclpy.init(args=args)
    
    minimal_subscriber = ImageReader()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


