from typing import Sequence
import numpy as np
import rclpy
from rclpy.node import Node

class GraspMatrixNode(Node):
    def __init__(self):
        super().__init__('grasp_matrix_node')

        # Example: run once when node starts
        self.get_logger().info('Grasp Matrix Node Started!')

        # Example inputs
        joints_list = [[-0.015, 0, 0], [-0.0217, 0.0125, 0], [0.015, 0, 0], [0.0217, 0.025, 0]]
        contact_points_list = [[-0.015, 0.025, 0], [0.015, 0.025, 0]]
        R_angles = [30, 60]
        object_center = [0, 0.025, 0]

        # Compute contact rotation matrices and full grasp matrix
        R_contact_List = self.R_ci_N_Matrix(contact_points_list, joints_list, R_angles)
        grasp_M = self.grasp_matrix(object_center, contact_points_list, R_contact_List)

        self.get_logger().info(f'Full Grasp Matrix:\n{grasp_M}')
        
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

    def skew_symmetric_matrix(self, vector: Sequence[float]) -> np.array:
        r = np.asarray(vector, dtype=float).reshape(3)
        x, y, z = r
        return np.array([[0.0, -z, y],
                         [z, 0.0, -x],
                         [-y, x, 0.0]])

    def grasp_matrix(self, object_center_location, contact_points_locations, R_contact_list):
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

def main(args=None):
    rclpy.init(args=args)
    node = GraspMatrixNode()
    rclpy.spin(node)  # Keeps node alive
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()