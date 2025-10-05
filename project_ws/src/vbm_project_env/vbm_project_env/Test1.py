#!/usr/bin/env python3

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
        '/camera1/image_raw', 
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
        justcanny = current_frame

        #Find center of mass for each object
        centroids = self.centroids(current_frame)
        
        print("\t")
        print(centroids)

        #Show final result of image 
        cv2.imshow("Best Grasp Point of Top Surface", current_frame)
        cv2.waitKey(1)

        #Publish image 
        # The 'cv2_to_imgmsg' method converts an OpenCV
        # image to a ROS 2 image message
        self.publisher_.publish(self.br.cv2_to_imgmsg(current_frame, encoding="bgr8"))


    #Function to find the center of mass of the top surface of an object 
    def centroids(self, img): 
        grey_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        edge = cv2.Canny(grey_img, 100, 200)
        kernel = np.ones((5,5), cv2.BORDER_DEFAULT)
        dilated = cv2.dilate(edge, kernel, iterations=1) 
        blur = cv2.GaussianBlur(dilated, (5, 5), cv2.BORDER_DEFAULT)
        #ret, thresh = cv2.threshold(blur, 127, 255,cv2.THRESH_BINARY_INV)
        contours, hierarchy = cv2.findContours(blur,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        comcoords = np.zeros((2, len(contours)))
        for i, c in enumerate(contours):
            Mom = cv2.moments(c)
        if (Mom["m00"]!= 0):
            cX = int(Mom["m10"] / Mom["m00"])
            cY = int(Mom["m01"] / Mom["m00"])
            comcoords[0,i] = cX
            comcoords[1,i] = cY
            cv2.circle(blur, (cX, cY), 5, (0,0,0), -1)
            cv2.putText(blur, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        else:
            comcoords[0,i] = 0
            comcoords[1,i] = 0
        cv2.imshow("output_image", blur)

        return contours, comcoords
    
    def closest_point(self, img):
        result = self.centroids(self, img)
        comcoords = result[1]
        contours = result[0]

        closest_Point = contours[0]
        min_dist = float('inf')

        for (x, y) in contours: 
            dist = math.sqrt((x -comcoords[0])**2 + (y - comcoords[1]**2))
            if dist < min_dist:
                min_dist = dist
                closest_Point = (x, y)

        return closest_Point
    
    def opposite_closest_point(self, comcoords, closest_Point, contours):
        #Find opposite point robustly, even for vertical/rotated orientations
        new_contours = [p for p in contours if p != closest_Point]
        closest_point_slope = self.calculate_slope(comcoords, closest_Point)

        if closest_point_slope == "undefined":
            return self.opposite_closest_point_angle(comcoords, closest_Point, contours)
       
        min_slope_diff = float('inf')
        opposite_point = None

        for (x, y) in new_contours:
            Point = (x,y)

            slope = self.calculate_slope(comcoords, Point)
            slope_diff = abs(slope - closest_point_slope)

            if slope_diff < min_slope_diff:
                min_slope_diff = slope_diff
                opposite_point = (x, y)

        print(f"Found Opposite Point: {opposite_point}")
        return opposite_point
    
    def opposite_closest_point_angle(self, comcoords, closest_Point, contours):
        cx, cy = comcoords
        px, py = closest_Point
        vx, vy = px - cx, py - cy
        mag_v = math.hypot(vx, vy)
        if mag_v == 0:
            return None

        best_point = None
        min_dot = float('inf')

        for (x, y) in contours:
            ux, uy = x - cx, y - cy
            mag_u = math.hypot(ux, uy)
            if mag_u == 0:
                continue

            dot = (vx * ux + vy * uy) / (mag_v * mag_u)
            if dot < min_dot:
                min_dot = dot
                best_point = (x, y)

        return best_point

    def calculate_slope(self, comcoords, closest_Point):
        xCOMcoord, yCOMcoord = comcoords
        xClosest, yClosest = closest_Point

        try: 
            slope = (yClosest - yCOMcoord) / (xClosest - xCOMcoord)
            return slope
        
        except ZeroDivisionError:
            return "undefined"
    
    def calculate_angle(self, comcoords, point):
        #Compute robust angle between COM and point (handles vertical lines)

        xCOMcoord, yCOMcoord = comcoords
        xPoint, yPoint = point

        return math.atan2(yPoint - yCOMcoord, xPoint - xCOMcoord)
    
    def vector_subtraction(self, vector1: Sequence[float], vector2: Sequence[float]) -> Sequence[float]:
        v1 = np.asarray(vector1, dtype=float)
        v2 = np.asarray(vector2, dtype=float)
        return v1 - v2

    def joint_rotation_matrix_aboutZ(self, theta):
        c = np.cos(theta)
        s = np.sin(theta)
        return np.array([[c, -s, 0],
                         [s, c, 0],
                         [0, 0, 1]])
    
    def skew_symmetric_matrix(self, vector: Sequence[float]) -> np.array:
        r = np.asarray(vector, dtype=float).reshape(3)
        x, y, z = r
        return np.array([[0.0, -z, y],
                         [z, 0.0, -x],
                         [-y, x, 0.0]])
    
    def R_ci_N_Matrix(self, contact_points_locations, joints_list, contact_rotation_angle):
        num_contacts = len(contact_points_locations)
        num_joints_total = len(joints_list)
        num_joints_per_contact_point = num_joints_total // num_contacts

        R_contact_list = []

        for index, contact_i in enumerate(contact_points_locations):
            theta = contact_rotation_angle[index]

            # Compute rotation matrices for all joints of this contact
            j_Rotation_matrix_list = [self.joint_rotation_matrix_aboutZ(theta)
                                      for _ in range(num_joints_per_contact_point)]

            R_contact_matrix = np.eye(3)
            for R_joint in j_Rotation_matrix_list:
                R_contact_matrix = np.matmul(R_contact_matrix, R_joint)
            
            zero_matrix = np.zeros((3,3))
            R_Ci_N = np.block([
                [R_contact_matrix, zero_matrix],
                [zero_matrix, R_contact_matrix]
            ])

            print(f"\nRotation matrix for contact {index}:\n{R_Ci_N}")
            R_contact_list.append(R_Ci_N)
        return R_contact_list 
    
    #Function to calculate the grasp martrix of the entire object and contact points
    def grasp_matrix(self, object_center_location, contact_points_locations, joints_list, contact_rotation_angle):
        R_contact_list = self.R_ci_N_Matrix(contact_points_locations, joints_list, contact_rotation_angle)

        rows = []
        for i_index, point_vector in enumerate(contact_points_locations):
            r = self.vector_subtraction(object_center_location, point_vector)
            r_skew = self.skew_symmetric_matrix(r)

            identity_matrix = np.eye(3)
            zero_matrix = np.zeros((3,3))
            Pi = np.block([
                [identity_matrix, r_skew],
                [zero_matrix, identity_matrix]
            ])

            print(f"\nPi matrix for contact {i_index}:\n{Pi}")
            print(f"Corresponding rotation matrix:\n{R_contact_list[i_index]}")

            G_Matrix = np.matmul(Pi, R_contact_list[i_index])
            print(f"Grasp matrix for contact {i_index}:\n{G_Matrix}")
            rows.append(G_Matrix)

        Full_Grasp_Matrix_Transpose = np.vstack(rows)
        return Full_Grasp_Matrix_Transpose
    
    def calculate_MSV(self, grasp_matrix):
        singular_values = np.linalg.svd(grasp_matrix, compute_uv=False)

        singular_values_minimum = singular_values[-1]

        return singular_values_minimum

def main(args=None):
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  Closest_Point = (2,3)
  Center_of_Mass = (2.5,3)
  contours = [(2,1), (3,1), (2,2), (3,2), (2,3), (3,3), (2,4), (3,4), (2,5), (3,5)]

  # Create the node
  image_subscriber = ImageSubscriber()

  OCLP = image_subscriber.opposite_closest_point(Center_of_Mass, Closest_Point, contours)
  
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
