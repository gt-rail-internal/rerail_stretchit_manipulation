#!/usr/bin/env python3
import ikpy.chain
import numpy as np
import json
from manipulation.utils import rotation_matrix_to_rpy
from typing import List

# NOTE: Adapted from: https://github.com/mlamsey/stretch_ik

class StretchPose:
    """
    Data structure for storing Stretch's pose. Indented to use with Stretch Original Gripper.
    """

    def __init__(self):
        # POSE KEY:
        # "0": "joint_base_translation"
        # "1": "joint_mast"
        # "2": "joint_lift"
        # "3": "joint_arm_l4" [do not use - fixed base link of wrist extension]
        # "4": "joint_arm_l3" [sum of l0-l3 is wrist_extension]
        # "5": "joint_arm_l2" [sum of l0-l3 is wrist_extension]
        # "6": "joint_arm_l1" [sum of l0-l3 is wrist_extension]
        # "7": "joint_arm_l0" [sum of l0-l3 is wrist_extension]
        # "8": "joint_wrist_yaw"
        # "9": "joint_wrist_yaw_bottom" [do not use - dummy link?]
        # "10": "joint_straight_gripper"
        # "11": "joint_grasp_center" [do not use - not actuated AFAIK]

        # default poses without roll, pitch and base link
        self._stowed_pose = [0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 3.4, 0.0, 0.0, 0.0]
        self._mid_pose = [0.0, 0.0, 0.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # TODO: Add a default pose which is our pre-ik pose.
        self.pre_ik_pose = [0.0, 0.0, 0.8990015196900174, 0.0, 0.05263223492577255, 0.05263223492577255, 0.05263223492577255, 0.05263223492577255, 0.03195793308095086, 0.0, 0.0, 0.0]
        # state
        self.pose = self.pre_ik_pose.copy()

    def get_pose_vector(self) -> list:
        """
        Returns Stretch's pose as a vector, as used by the IK solver.
        """
        return self.pose
    
    def get_pose_dict(self, joint_lookup: dict) -> dict:
        """
        Returns Stretch's pose as a dictionary, with keys corresponding to the joint names.

        Args:
            joint_lookup (dict): Dictionary mapping joint names to indices in the pose vector.
            [NOTE: joint_lookup = {link.name: i for i, link in enumerate(ikpy.chain.Chain().chain.links)}]

        Returns:
            dict: Stretch's pose
            {
                "translate_mobile_base": [x, y, z],
                "joint_lift": lift,
                "wrist_extension": wrist_extension,
                "joint_wrist_yaw": wrist_yaw,
            }
        """

        # XYZ
        base_translation = self.pose[joint_lookup["joint_base_translation"]]
        lift = self.pose[joint_lookup["joint_lift"]]
        arm_l3 = self.pose[joint_lookup["joint_arm_l3"]]
        arm_l2 = self.pose[joint_lookup["joint_arm_l2"]]
        arm_l1 = self.pose[joint_lookup["joint_arm_l1"]]
        arm_l0 = self.pose[joint_lookup["joint_arm_l0"]]
        arm_extension = sum([arm_l3, arm_l2, arm_l1, arm_l0])

        # Yaw
        joint_wrist_yaw = self.pose[joint_lookup["joint_wrist_yaw"]]
        
        stretch_pose = {
            "translate_mobile_base": base_translation,
            "joint_lift": lift,
            "wrist_extension": arm_extension,
            "joint_wrist_yaw": joint_wrist_yaw
        }

        return stretch_pose
    
    def set_pose(self, pose: list):
        """
        Sets Stretch's pose.

        Args:
            pose (list): [len=12] Stretch's pose vector, as used by the IK solver.
        """
        
        if len(pose) != 12:
            raise ValueError("StretchDexPose::set_pose: pose must be a list of length 12")
        self.pose = pose

class StretchIK:
    """
    Wrapper for IK solver. Uses ikpy.
    """

    def __init__(self, urdf_path: str) -> None:
        """
        Args:
            urdf_path (str): Path to Stretch's URDF file.
        """
        self.chain = ikpy.chain.Chain.from_urdf_file(urdf_path)
        self.joint_lookup = {link.name: i for i, link in enumerate(self.chain.links)}
    
    def print_chain(self):
        print(self.chain)
    

    def print_joint_lookup(self, bool_i_first=False) -> None:
        if bool_i_first:
            print(json.dumps({v: k for k, v in self.joint_lookup.items()}, indent=4))
        else:
            print(json.dumps(self.joint_lookup, indent=4))

    def solve_ik(self, target_pos: List,
                 target_ori: List[List]=None,
                 initial_pose: List=None,
                 bool_check_error=False) -> dict:
        """
        Solves IK for Stretch. Wrapper for ikpy's inverse_kinematics() function.

        Args:
            target_pos (list): [len=3] Target position in the base frame.
            target_ori (list[list]): [3x3] Target orientation in the base frame, as a rotation matrix.
            initial_pose (list): [len=12] Initial pose for the IK solver. If None, uses the current pose.

        Returns:
            dict: Stretch's pose (can be sent directly move_to_pose())
            {
                "translate_mobile_base": [x, y, z],
                "joint_lift": lift,
                "wrist_extension": wrist_extension,
                "joint_wrist_yaw": wrist_yaw,
            }
        """

        stretch_pose = StretchPose()
        
        if initial_pose is None:
            q0 = stretch_pose.get_pose_vector()
        else:
            q0 = initial_pose

        q_soln = self.chain.inverse_kinematics(target_position=target_pos,
                                            target_orientation=target_ori,
                                            orientation_mode="all",
                                            initial_position=q0)
        

        #--------------- Checks error if flag is passed --------------- #
        if bool_check_error:
            # compute error
            fk_soln = self.chain.forward_kinematics(q_soln)
            print(fk_soln)
            soln_pos = fk_soln[:3, 3]
            soln_pos_error = np.linalg.norm(target_pos - soln_pos)
            soln_ori = fk_soln[:3, :3]
            soln_rpy = rotation_matrix_to_rpy(soln_ori)
            target_rpy = rotation_matrix_to_rpy(target_ori)
            soln_ori_error = np.linalg.norm(np.array(target_rpy) - np.array(soln_rpy))
            print('#------------ Inverse Kinematics ------------#')
            print('Solution pose error: ', soln_pos_error)
            print('Solution ori error: ', soln_ori_error)
            print('----------------------------------------------')
        #-------------------------------------------------------------------------------#
        # get pose as dict that can be sent to move_to_pose()
        stretch_pose.set_pose(q_soln)
        fk_pose = stretch_pose.get_pose_dict(joint_lookup=self.joint_lookup)
        return fk_pose

# if __name__ == '__main__':
#     import rospkg
#     import os

#     # print joint names
#     pkg_path = rospkg.RosPack().get_path('stretch_ik')
#     urdf_path = os.path.join(pkg_path, "urdf/stretch.urdf")
#     ik_engine = StretchIK(urdf_path)
#     ik_engine.print_joint_lookup(bool_i_first=True)
