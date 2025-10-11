#!/usr/bin/env python3.10

#Important Code from HW4 (test1.py)
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from typing import Sequence
import math

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
        '/realsense/image_raw', #Changing from camera1 to realsense
        self.listener_callback, 
        10)
        self.subscription # prevent unused variable warning

        # Create the publisher. This publisher will publish an Image
        # to the video_frames topic. The queue size is 10 messages.
        self.publisher_ = self.create_publisher(Image, 'output_image', 10)

      
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

    #Function to actiavte when the video_frames topic is updated
    def listener_callback(self, data):
        """
        Callback function
        """
        #Display the message on the console
        self.get_logger().info('Receiving video frame')

        #Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data)

        #Find center of mass for each object
        contours, comcoords = self.centroids(current_frame)
        closest = np.zeros((2, len(contours)))
        opposite = closest.copy()
        cv2.drawContours(current_frame, contours, -1, (150,150,150), 1)

        # Creates comcoords, closest, and opposite 2D numpy lists of equal sizes, and displays the points
        for i, c in enumerate(contours):
            closest[:,i] = self.closest_point(c, comcoords[:,i])
            closepoint = closest[:,i]
            print(f"Closest Point before opp (Listener Callback): {closest[:,i]}")
            opposite[:,i] = self.opposite_closest_point(comcoords[:,i], closepoint, c)
            print(f"Closest Point after opp (Listener Callback): {closest[:,i]}")
            # print(f"Opposite Closest Point: {opposite[:,i]}")

            # Convert numpy arrays to tuples of ints
            com_pt = tuple(map(int, comcoords[:, i]))
            closest_pt = tuple(map(int, closest[:, i]))
            #closest_pt = tuple([closest[0, i],closest[1, i]])
            opposite_pt = tuple(map(int, opposite[:, i]))

            print(f"Closest Point after Casting: {closest_pt}")
            print(f"Opposite Closest Point after Casting: {opposite_pt}")

            cv2.circle(current_frame, com_pt, 5, (0,0,255), -1)
            cv2.circle(current_frame, closest_pt, 5, (0,255,0), -1)
            cv2.circle(current_frame, opposite_pt, 5, (255,0,0), -1)

            Grasp_Matrix = self.grasp_matrix(comcoords, closest_pt, opposite_pt)

            Grasp_Metric = self.calculate_MSV(Grasp_Matrix)

            print(f"Grasp Metric of Grasp Matrix: {Grasp_Metric}")

        #Show final result of image 
        cv2.imshow("Best Grasp Point of Top Surface", current_frame)
        cv2.waitKey(1)

        #Publish image 
        # The 'cv2_to_imgmsg' method converts an OpenCV
        # image to a ROS 2 image message
        self.publisher_.publish(self.br.cv2_to_imgmsg(current_frame, encoding="bgr8"))


    #Function to find the center of mass of the top surface of an object 
    def centroids(self, img): 
        #Creates Gray scale image
        grey_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        #Canny Edge Detection of Gray Scale Image
        edge = cv2.Canny(grey_img, 100, 200)

        #Decalres kernel which will be used to dilate image
        kernel = np.ones((5,5), np.uint8)

        #Dilate the Canny Edge Detection Image to Find the Contours
        dilated = cv2.dilate(edge, kernel, iterations=1) 

        #Blur the Dilated image to smooth out all noise
        blur = cv2.GaussianBlur(dilated, (5, 5), cv2.BORDER_DEFAULT)

        #Usage of OpenCV Function to find the Contours in the image
        contours, hierarchy = cv2.findContours(blur,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

        #Declares Center of Mass Coords Variable
        comcoords = np.zeros((2, len(contours)))

        #Iterate through contours to find Center of Mass Coordinates
        for i, c in enumerate(contours):
            Mom = cv2.moments(c)

        #Calculates/Finds Center of Mass Coords
        if (Mom["m00"]!= 0):
            cX = int(Mom["m10"] / Mom["m00"])
            cY = int(Mom["m01"] / Mom["m00"])
            comcoords[0,i] = cX
            comcoords[1,i] = cY

        #Else case if Center of Mass Coords are not defined
        else:
            comcoords[0,i] = 0
            comcoords[1,i] = 0

        return contours, comcoords

    #Finds the Unit Vector of the closest and opposite closest point using the center of mass coordinates
    def find_Unit_Vector(self, comcoord, closest_point, opposite_closest_point):
        #Calculates the Normal Vector for the Closest and Opposite Closest Point
        Normal_Vector_Closest = comcoord - closest_point
        Normal_Vector_Opposite_Closest = comcoord - opposite_closest_point

        #Calculates the Unit Vector for the Closest and Opposite Closest Point
        Unit_Vector_Closest = (Normal_Vector_Closest/np.linalg.norm(Normal_Vector_Closest))
        Unit_Vector_Opposite_Closest = (Normal_Vector_Opposite_Closest/np.linalg.norm(Normal_Vector_Opposite_Closest))

        return Unit_Vector_Closest, Unit_Vector_Opposite_Closest
    
    #Finds the Closest Point with the given contour and the center of mass coordinates 
    def closest_point(self, contour, comcoord):

        #Declares the minimum distance in order to compare it to find the closest point
        min_dist = float('inf')

        #Decalres a variable to hold the closest point
        Closest_Point = tuple([min_dist, min_dist])

        #Finds magmitude of closest point
        dmag = np.linalg.norm(Closest_Point) 

        #Iterate through loop of each point in the contour 
        for j in contour:
            
            #Squeeze J
            j = np.squeeze(j)

            #Resultant vector from current CoM to point
            newdist = comcoord - j

            #New magnitude of Closest Point
            ndmag = np.linalg.norm(newdist)

            #Check new point against last closest point
            if (ndmag < dmag):
                Closest_Point = j
                dmag = ndmag
        print("Closest Point Function Output: ")
        print(Closest_Point)

        return Closest_Point
    
    #Finds the Opposite Closest Point with the given contour, the closest point, and the center of mass coordinates 
    def opposite_closest_point(self, comcoords, closest_Point, contour):

        #Calculate the Normal Vector of the Closest Point
        Normal_vector = comcoords - closest_Point

        #Find the Unit Vector of the Normal Vector and Reverse Direction to become Opposite
        Unit_vector = (Normal_vector/np.linalg.norm(Normal_vector))*-1

        #Declares the minimum distance in order to compare it to find the closest point
        Smallest_diff = tuple([float('inf'),float('inf')])

        #Declares a variable to hold the opposite closest point
        Opposite_closest_point = tuple([0,0])

        #Iterate through loop of each point in the contour
        for i in contour:

           #Squeeze I 
            i = np.squeeze(i)

            #Calculate the new Normal Vector of the current Point
            Normal_vector_contour = comcoords - i

            #Find the Unit Vector of the current Normal Vector 
            Unit_vector_contour = (Normal_vector_contour/np.linalg.norm(Normal_vector_contour))

            #Find the difference between the two unit vectors 
            Unit_diff = abs(Unit_vector - Unit_vector_contour)
            
            #Check new point against last opposite closest point
            if(np.all(Unit_diff < Smallest_diff)):
                Smallest_diff = Unit_diff
                Opposite_closest_point = i

        print("opposite point to closest point:")
        print(Opposite_closest_point)

        return Opposite_closest_point
    
    #Determines the Grasp Matrix of an object given the object's center, the closest point on the object, and the opposite closest point on the object
    def grasp_matrix(self, comcoord, closest, opposite):

        # Define 3x1 points for contacts and center
        c1 = np.array([int(closest[0]), int(closest[1]), 0])
        c2 = np.array([int(opposite[0]), int(opposite[1]), 0])

        #Assemble the 1x3 Matrix of Center of Object Coordinates
        center = np.array([int(comcoord[0]), int(comcoord[1]), 0])
        
        #Create skew symmetric matrices
        skew1 = self.skew_symmetric_matrix(np.subtract(c1, center))
        skew2 = self.skew_symmetric_matrix(np.subtract(c2, center))

        #Declares a 3x3 Identity Matrix
        identity_matrix = np.eye(3)

        #Declares a 3x3 Matrix of 0's
        zero_matrix = np.zeros((3,3))

        # Create P matrices
        p1 = np.block([
                [identity_matrix, np.transpose(skew1)],
                [zero_matrix, identity_matrix]
                ])
        p2 = np.block([
                [identity_matrix, np.transpose(skew2)],
                [zero_matrix, identity_matrix]
                ])
        
        #Create rotation blockdiag matrices
        RtoContact1 = self.calculateR_Matrix(center, c1)
        RtoContact2 = self.calculateR_Matrix(center, c2)

        #Grasp matrices
        g1 = RtoContact1*p1
        g2 = RtoContact2*p2
    
        return np.vstack((g1,g2))
    
    #Calculate a Rotation Matrix with a given center point of an object, and the contact point of the object.
    def calculateR_Matrix(self, center_point, contact_point):
        #Calculate the Unit Vector for the given center point of an object, the contact point
        UnitVector, unused = self.find_Unit_Vector(center_point, contact_point, np.array([1, 0, 0]))

        #Subtract the Unit Vector from the Determined Vector
        Resultant = np.array([1,0, 0]) - UnitVector

        #Calculate the Rotation Matrix with the given Resultant's Values
        R_Bar_Frame = self.joint_rotation_matrix_aboutZ(Resultant[0], Resultant[1])

        #Declares a 3x3 Matrix of 0's
        zero_matrix = np.zeros((3,3))

        #Assemble Rotation Matrix Block
        R_Ci_N = np.block([
            [R_Bar_Frame, zero_matrix],
            [zero_matrix, R_Bar_Frame]
        ])

        return R_Ci_N
    
    #Calculate the Minimum Singular Value Grasp Metric for a given Grasp Matrix 
    def calculate_MSV(self, grasp_matrix):
        #Find all of the Singular Values through Singular Value Decomposition for a given Grasp Matrix 
        singular_values = np.linalg.svd(grasp_matrix, compute_uv=False)

        #Find the Minimum Singular Value from the total list of all Singular Value
        singular_values_minimum = singular_values[-1]

        return singular_values_minimum

#Main Function of Test Python File
def main(args=None):
  # Initialize the rclpy library
  rclpy.init(args=args)

  # Create the node
  image_subscriber = ImageSubscriber()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber)
  
  # Destroy the node explicitly
  image_subscriber.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()

if __name__ == '__main__':
    main()