import ikpy.chain
import numpy as np


class SimpleStretchIK:
    """
    Class to solve inverse kinematics for a Stretch robot with a standard gripper.
    """

    def __init__(self, urdf_path):
        # Load the modified URDF that represents the robot's chain for IK calculations
        self.chain = ikpy.chain.Chain.from_urdf_file(urdf_path)

    def solve_ik(self, target_position, target_orientation=None, initial_configuration=None):
        # If no orientation is provided, use a default one that points the gripper forward
        if target_orientation is None:
            target_orientation = ikpy.utils.geometry.rpy_matrix(0, 0, 0)

        # Calculate the initial configuration of the robot
        #stowed_pose = [0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 3.4, 0.0, 0.0]
        #initial_configuration = self.chain.active_to_full(self.chain.inverse_kinematics(target_position), [0] * len(self.chain.links))
        #print(initial_configuration)
        # Solve inverse kinematics to find the joint angles that achieve the target pose
        joint_angles = self.chain.inverse_kinematics(target_position, target_orientation, initial_position=initial_configuration)
        
        return joint_angles

    def get_joint_names(self):
        # Retrieve the names of the joints in the chain
        return [link.name for link in self.chain.links if link.active]
    


# Usage example:
# urdf_path should be the path to the modified URDF file for the Stretch robot with the standard gripper.
# ik_solver = SimpleStretchIK(urdf_path)
# target_position = [x, y, z]  # Replace with the desired position
# target_orientation = ikpy.utils.geometry.rpy_matrix(roll, pitch, yaw)  # Replace with the desired orientation in roll-pitch-yaw
# joint_angles = ik_solver.solve_ik(target_position, target_orientation)
# print(joint_angles)
