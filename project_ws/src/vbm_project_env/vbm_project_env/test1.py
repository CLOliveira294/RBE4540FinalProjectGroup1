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
        grey_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        edge = cv2.Canny(grey_img, 100, 200)
        kernel = np.ones((5,5), np.uint8)
        dilated = cv2.dilate(edge, kernel, iterations=1) 
        blur = cv2.GaussianBlur(dilated, (5, 5), cv2.BORDER_DEFAULT)
        #ret, thresh = cv2.threshold(blur, 127, 255,cv2.THRESH_BINARY_INV)
        contours, hierarchy = cv2.findContours(blur,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        comcoords = np.zeros((2, len(contours)))
        for i, c in enumerate(contours):
            Mom = cv2.moments(c)
        if (Mom["m00"]!= 0):
            cX = int(Mom["m10"] / Mom["m00"])
            cY = int(Mom["m01"] / Mom["m00"])
            comcoords[0,i] = cX
            comcoords[1,i] = cY
            #cv2.circle(blur, (cX, cY), 5, (0,0,0), -1)
            #cv2.putText(blur, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        else:
            comcoords[0,i] = 0
            comcoords[1,i] = 0

        # cv2.imshow("output_image", blur)
        # cv2.waitKey(1)

        return contours, comcoords
    
    def find_Unit_Vector(self, comcoord, closest_point, opposite_closest_point):
        Normal_Vector_Closest = comcoord - closest_point
        Normal_Vector_Opposite_Closest = comcoord - opposite_closest_point

        Unit_Vector_Closest = (Normal_Vector_Closest/np.linalg.norm(Normal_Vector_Closest))
        Unit_Vector_Opposite_Closest = (Normal_Vector_Opposite_Closest/np.linalg.norm(Normal_Vector_Opposite_Closest))

        return Unit_Vector_Closest, Unit_Vector_Opposite_Closest
    
    # Contour is a single contour
    # comcoord is a single center of mass
    def closest_point(self, contour, comcoord):
        min_dist = float('inf')

        Closest_Point = tuple([min_dist, min_dist])
        dmag = np.linalg.norm(Closest_Point) #Magnitude of Closest Point

        for j in contour:

            j = np.squeeze(j)

            # resultant vector from current CoM to point
            newdist = comcoord - j
            ndmag = np.linalg.norm(newdist)

            # Check new point against last closest point
            if (ndmag < dmag):
                Closest_Point = j
                dmag = ndmag
        print("Closest Point Function Output: ")
        print(Closest_Point)

        return Closest_Point
    
    def opposite_closest_point(self, comcoords, closest_Point, contour):
        #Find opposite point robustly, even for vertical/rotated orientations

        #new_contour = [p for p in contour if not np.array_equal(p, closest_Point)]

        Normal_vector = comcoords - closest_Point

        Unit_vector = (Normal_vector/np.linalg.norm(Normal_vector))*-1

        Smallest_diff = tuple([float('inf'),float('inf')])
        Opposite_closest_point = tuple([0,0])

        for i in contour:

            i = np.squeeze(i)

            Normal_vector_contour = comcoords - i

            Unit_vector_contour = (Normal_vector_contour/np.linalg.norm(Normal_vector_contour))

            Unit_diff = abs(Unit_vector - Unit_vector_contour)
            

            if(np.all(Unit_diff < Smallest_diff)):
                Smallest_diff = Unit_diff
                Opposite_closest_point = i
        # print(f"Opposite Closest Point: {Opposite_closest_point}")

        print("opposite point to closest point:")
        print(Opposite_closest_point)

        return Opposite_closest_point
    
        # closest_point_slope = self.calculate_slope(comcoords, closest_Point)

        # if closest_point_slope == "undefined":
        #     return self.opposite_closest_point_angle(comcoords, closest_Point, contours)
       
        # min_slope_diff = float('inf')
        # opposite_point = None

        # for p in new_contours:
        #     x, y = np.squeeze(p)
        #     Point = (x,y)

        #     slope = self.calculate_slope(comcoords, Point)
        #     slope_diff = abs(slope - closest_point_slope)

        #     if slope_diff < min_slope_diff:
        #         min_slope_diff = slope_diff
        #         opposite_point = (x, y)

        # print(f"Found Opposite Point: {opposite_point}")

    
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

    # def calculate_slope(self, comcoords, closest_Point):
    #     xCOMcoord, yCOMcoord = comcoords
    #     xClosest, yClosest = closest_Point

    #     try: 
    #         slope = (yClosest - yCOMcoord) / (xClosest - xCOMcoord)
    #         return slope
        
    #     except ZeroDivisionError:
    #         return "undefined"
    
    # def calculate_angle(self, comcoords, point):
    #     #Compute robust angle between COM and point (handles vertical lines)

    #     xCOMcoord, yCOMcoord = comcoords
    #     xPoint, yPoint = point

    #     return math.atan2(yPoint - yCOMcoord, xPoint - xCOMcoord)
    
    def vector_subtraction(self, vector1: Sequence[float], vector2: Sequence[float]) -> Sequence[float]:
        v1 = np.asarray(vector1, dtype=float)
        v2 = np.asarray(vector2, dtype=float)
        return v1 - v2

    def joint_rotation_matrix_aboutZ(self, c, s):

        return np.array([[c, -s, 0],
                         [s, c, 0],
                         [0, 0, 1]])
    
    def skew_symmetric_matrix(self, vector: Sequence[float]) -> np.array:
        r = np.asarray(vector, dtype=float).reshape(3)
        x, y, z = r
        return np.array([[0.0, -z, y],
                         [z, 0.0, -x],
                         [-y, x, 0.0]])
    
    # def R_ci_N_Matrix(self, contact_points_locations, joints_list, contact_rotation_angle):
    #     num_contacts = len(contact_points_locations)
    #     num_joints_total = len(joints_list)
    #     num_joints_per_contact_point = num_joints_total // num_contacts

    #     R_contact_list = []

    #     for index, contact_i in enumerate(contact_points_locations):
    #         theta = contact_rotation_angle[index]

    #         # Compute rotation matrices for all joints of this contact
    #         j_Rotation_matrix_list = [self.joint_rotation_matrix_aboutZ(theta)
    #                                   for _ in range(num_joints_per_contact_point)]

    #         R_contact_matrix = np.eye(3)
    #         for R_joint in j_Rotation_matrix_list:
    #             R_contact_matrix = np.matmul(R_contact_matrix, R_joint)
            
    #         zero_matrix = np.zeros((3,3))
    #         R_Ci_N = np.block([
    #             [R_contact_matrix, zero_matrix],
    #             [zero_matrix, R_contact_matrix]
    #         ])

    #         print(f"\nRotation matrix for contact {index}:\n{R_Ci_N}")
    #         R_contact_list.append(R_Ci_N)
    #     return R_contact_list 
    
    #Function to calculate the grasp martrix of the entire object and contact points
    # def grasp_matrix(self, object_center_location, contact_points_locations, joints_list, contact_rotation_angle):
    #     R_contact_list = self.R_ci_N_Matrix(contact_points_locations, joints_list, contact_rotation_angle)

    #     rows = []
    #     for i_index, point_vector in enumerate(contact_points_locations):
    #         r = self.vector_subtraction(object_center_location, point_vector)
    #         r_skew = self.skew_symmetric_matrix(r)

    #         identity_matrix = np.eye(3)
    #         zero_matrix = np.zeros((3,3))
    #         Pi = np.block([
    #             [identity_matrix, r_skew],
    #             [zero_matrix, identity_matrix]
    #         ])

    #         print(f"\nPi matrix for contact {i_index}:\n{Pi}")
    #         print(f"Corresponding rotation matrix:\n{R_contact_list[i_index]}")

    #         G_Matrix = np.matmul(Pi, R_contact_list[i_index])
    #         print(f"Grasp matrix for contact {i_index}:\n{G_Matrix}")
    #         rows.append(G_Matrix)

    #     Full_Grasp_Matrix_Transpose = np.vstack(rows)
    #     return Full_Grasp_Matrix_Transpose

    def grasp_matrix(self, comcoord, closest, opposite):

        # Define 3x1 points for contacts and center
        c1 = np.array([int(closest[0]), int(closest[1]), 0])
        c2 = np.array([int(opposite[0]), int(opposite[1]), 0])
        center = np.array([int(comcoord[0]), int(comcoord[1]), 0])
        
        #Create skew symmetric matrices
        skew1 = self.skew_symmetric_matrix(np.subtract(c1, center))
        skew2 = self.skew_symmetric_matrix(np.subtract(c2, center))

        # Create P matrices
        identity_matrix = np.eye(3)
        zero_matrix = np.zeros((3,3))
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
        print(RtoContact1)
        RtoContact2 = self.calculateR_Matrix(center, c2)

        #Grasp matrices
        g1 = RtoContact1*p1
        g2 = RtoContact2*p2
    
        return np.vstack((g1,g2))
    
    def calculateR_Matrix(self, center_point, contact_point):

        UnitVector, unused = self.find_Unit_Vector(center_point, contact_point, np.array([1, 0, 0]))

        Resultant = np.array([1,0, 0]) - UnitVector

        R_Bar_Frame = self.joint_rotation_matrix_aboutZ(Resultant[0], Resultant[1])
        zero_matrix = np.zeros((3,3))
        R_Ci_N = np.block([
            [R_Bar_Frame, zero_matrix],
            [zero_matrix, R_Bar_Frame]
        ])

        return R_Ci_N
    
    def calculate_MSV(self, grasp_matrix):
        singular_values = np.linalg.svd(grasp_matrix, compute_uv=False)

        singular_values_minimum = singular_values[-1]

        return singular_values_minimum

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