#!/usr/bin/env python3

#Important Code from HW4 (test1.py)
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageSubscriber(Node):
    """
    Create an ImageSubscriber class, which is a subclss of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('image_subscriber')
      
        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.

        self.subscription = self.create_subscription(
        Image, 
        '/camera1/image_raw', 
        self.listener_callback, 
        10)
        self.subscription # prevent unused variable warning

        # Create the publisher. This publisher will publish an Image
        # to the video_frames topic. The queue size is 10 messages.
        self.publisher_ = self.create_publisher(Image, 'output_image', 10)

      
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

    def find_com(): 
        #Write Center of Mass Code Here
        return 0

def main(args=None):
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_subscriber = ImageSubscriber()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()

if __name__ == '__main__':
    main()
