from typing import Sequence
import numpy as np

###This file can just run via the run button to the top-right
##I'm having issues with making grasp_matrix_node.py an executable file. 
# I can make it as such but then the terminal always tells me "No Executable Found"
def vector_subtraction(vector1: Sequence[float], vector2: Sequence[float]) -> Sequence[float]:
    """
    Input: two vectors vector (w/ 3 rows), sequence of type floats
    Output: a resulting vector 3x1, sequence of type floats
    """
    v1 = np.asarray(vector1, dtype=float)
    v2 = np.asarray(vector2, dtype=float)
    return v1 - v2


def joint_rotation_matrix_aboutZ(theta):
    """
    Input: a joint's angle | the angle between two legs with a bend in the middle
    Output: A rotation matrix R for the transition between 2 joints
    """
    c = np.cos(theta)
    s = np.sin(theta)
    joint_R = np.array([[c, -s, 0],
                  [s, c, 0],
                  [0, 0, 1]])
    
    return joint_R

def R_ci_N_Matrix(contact_points_locations, joints_list, contact_rotation_angle):
    """
    Input: 
    - A list of contact point vectors, each of 3x1,
    - List of joint_locations
    - List of Final Rotation angle transformations between 1st joint of each finger and the respective contact_point

    Output: 
    - a list of the full rotation matrices for all the contacts
    """

    num_contacts = len(contact_points_locations)

    # # the minimum number of joints per 1 finger
    # num_joints_per_contact_point = num_contacts - 1

    num_joints_total = len(joints_list)
    num_joints_per_contact_point = num_joints_total // num_contacts

    R_contact_list = []

    ### ROTATION MATRICES WORK: FOR EACH CONTACT POINT
    #Loop through the number of contact points to get a rotation matrix for each point
    for index, contact_i  in enumerate(contact_points_locations):

        # Assign angles to the joints, 
        theta = contact_rotation_angle[index]

        # for the number of joints for 1 contact point
        j_Rotation_matrix_list = []
        for i in range(num_joints_per_contact_point):
            #create a rotation matrix for each joint
            R = joint_rotation_matrix_aboutZ(theta)
            j_Rotation_matrix_list.append(R)

        #Create a 3x3 identity matrix
        R_contact_matrix = np.eye(3)

        #For each rotation matrices per joint, multiply it with the next R_matrix in the list of joint Rotation matrices
        # (Multiply all R_matrices of the joints together, to get the final R_matrix for each contact_point)
        for R_joint in j_Rotation_matrix_list:
            R_contact_matrix = np.matmul(R_contact_matrix, R_joint)
        
        zero_matrix = np.zeros((3,3))
        R_Ci_N = np.block([
            [R_contact_matrix, zero_matrix],
            [zero_matrix, R_contact_matrix]
        ])

        print()
        print("This is the rotation matrix for the ", index, " contact")
        print(R)
        # #Add the rotation matrix of this contact point expressed in N, with respect to Ci,
        #  to the list of contact_Rotation_matrices
        R_contact_list.append(R_Ci_N)
    return R_contact_list  

def skew_symmetric_matrix(vector: Sequence[float]) -> np.array:
    """
    Input: a vector (w/ 3 rows), sequence of type floats
    Output: a skew-symmetric matrix, a np.array
    """

    r = np.asarray(vector, dtype = float).reshape(3)
    x, y, z = r
    return np.array([[0.0, -z, y],
                     [z, 0.0, -x],
                     [-y, x, 0.0]])

def grasp_matrix(object_center_location, contact_points_locations, R_contact_list):
    """
    Input: 
    - the object center's location as a 3x1 vector, 
    - and a list of contact point vectors 3x1

    Output: 
    - a 12x6 grasp matrix
    """
    rows = []
    #Loops over the list of contact points:
    for i_index, point_vector in enumerate(contact_points_locations):
        #Gets the vector of the contact_point relative to the object_center
        r = vector_subtraction(object_center_location, point_vector)
        
        #Gets the skew-symmetric matrix of r
        r_skew = skew_symmetric_matrix(r)

        ### Pi MATRICES WORK:
        #Make an Identity Matrix:
        identity_matrix = np.eye(3)
        #Make a zero matrix:
        zero_matrix = np.zeros((3,3))

        #Make the twist point mapping matrix Pi
        Pi = np.block([
            [identity_matrix, r_skew],
            [zero_matrix, identity_matrix]
        ])
        print()
        print("This is Pi only: ")
        print(Pi)

        print()
        print("this is the corresponding rotation matrix")
        print(R_contact_list[i_index])

        #compute each G_i matrix
        G_Matrix = np.matmul(Pi, R_contact_list[i_index])
        
        print()
        print("this is the corresponding G matrix")
        print(G_Matrix)
        #add it to a list of rows, 
        # so that it can get turned into a vertically stacked matrix
        rows.append(G_Matrix)

    #Define the full grasp matrix as a vertically stacked matrix of the G_Matrix from each contact point
    Full_Grasp_Matrix_Transpose = np.vstack(rows)

    # G_original = np.transpose(Full_Grasp_Matrix_Transpose)

    return Full_Grasp_Matrix_Transpose

if __name__ == "__main__":

    #using the joints and contacts from HW 1 as an example
    joints_list = [[-0.015, 0, 0], [-0.0217, 0.0125, 0], [0.015, 0, 0], [0.0217, 0.025, 0]]
    contact_rotation_angle = [30.0, 60.0] #the difference in rotation angle between joint 1 and the end-effector
    contact_points_list = [[-0.015, 0.025, 0], [0.015, 0.025, 0]]
    R_angles = [30, 60]
    object_center = [0, 0.025, 0]
    R_contact_List = R_ci_N_Matrix(contact_points_list, joints_list, R_angles)

    grasp_M = grasp_matrix(object_center, contact_points_list, R_contact_List)

    print()
    print("This is the Full Grasp Matrix: ")
    print(grasp_M)


