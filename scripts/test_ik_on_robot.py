import rospy
import hello_helpers.hello_misc as hm
from ik import StretchIK
import numpy as np
import time
from utils import quaternion_to_rotation_matrix

class StretchNode(hm.HelloNode):
    def __init__(self):
        super(StretchNode, self).__init__()

    def main(self, pose_dict):
        super(StretchNode, self).main('stretch_controller', 'stretch_namespace', wait_for_first_pointcloud=False)
        self.move_to_pose(pose_dict)

# Assuming you are running this in a ROS environment
if __name__ == '__main__':
    try:
        # Create an instance of the StretchNode class
        stretch_node = StretchNode()

        # Initialize IK engine
        urdf_path = "./stretch.urdf"
        ik_engine = StretchIK(urdf_path)

        target_pos = [0.190676, -0.5139, 0.5482]
        target_ori = np.array([[1., 0., 0.],
                    [0., 1., 0.],
                    [0., 0., 1.]])
        pose_dict = ik_engine.solve_ik(target_pos, target_ori)
        print('moving to the first position:')
        print(pose_dict)
        # Execute the main function of the stretch_node with the pose_dict
        stretch_node.main(pose_dict)
        
        
        # target_pos = [0.15927, -0.36004, 0.76639]
        # target_ori = np.array([[1., 0., 0.],
        #             [0., 1., 0.],
        #             [0., 0., 1.]])
        # pose_dict = ik_engine.solve_ik(target_pos, target_ori)
        # print('moving to the first position:')
        # print(pose_dict)
        # # Execute the main function of the stretch_node with the pose_dict
        # stretch_node.main(pose_dict)

        # time.sleep(8)

        # target_pos = [-0.21525, -0.39693, 0.23993]
        # target_ori = np.array([[0., 0., 0.],
        #             [0., 0., 0.],
        #             [0., 0., 0.985778]])
        # pose_dict = ik_engine.solve_ik(target_pos, target_ori)
        # print('moving to the second position:')
        # print(pose_dict)
        # # Execute the main function of the stretch_node with the pose_dict
        # stretch_node.main(pose_dict)


    except rospy.ROSInterruptException:
        pass
