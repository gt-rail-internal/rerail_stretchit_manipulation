import rospy
import hello_helpers.hello_misc as hm
from ik import StretchIK
from utils import quaternion_to_rotation_matrix
from stretch_fetch_grasp_bridge.srv import StretchGraspPose

# Define the StretchNode class inheriting from HelloNode
class StretchNode(hm.HelloNode):
    def __init__(self):
        super(StretchNode, self).__init__()  # Initialize the superclass
        rospy.wait_for_service('/stretch_grasp_pose_suggester')
        self.grasp_pose_suggestor_service = rospy.ServiceProxy('/stretch_grasp_pose_suggester', StretchGraspPose)

    def go_to_grasp_pose(self, trg_pose, trg_quat):
        # Convert target pose and quaternion to robot's grasp position
        target_ori = quaternion_to_rotation_matrix(trg_quat)
        pose_dict = ik_engine.solve_ik(trg_pose, target_ori)
        rospy.loginfo('Moving to grasp position...')
        rospy.loginfo("Joint state goals for grasping: %s", str(pose_dict))
        self.main(pose_dict)
        rospy.sleep(5)

    def lift_arm_pre_ik(self, post_ik_flag=False):
        # Lifts or retracts the arm before or after IK calculations
        pose_dict = {
            'translate_mobile_base': 0.0,
            'joint_lift': 0.899,
            'wrist_extension': 0.0,
            'joint_wrist_yaw': 3.4 if not post_ik_flag else 0.027
        }
        rospy.loginfo('Adjusting the arm position...')
        self.main(pose_dict)
        rospy.sleep(3)

    def go_to_pre_ik_pose(self, post_ik_flag=False):
        # Positions the arm before or after IK calculations
        pose_dict = {
            'translate_mobile_base': 0.0,
            'joint_lift': 0.899,
            'wrist_extension': 0.214,
            'joint_wrist_yaw': 0.027
        }
        rospy.loginfo('Adjusting the arm position...')
        self.main(pose_dict)
        rospy.sleep(3)

    def go_to_carrying_pose(self):
        # Moves the arm to a carrying position
        pose_dict = {
            'translate_mobile_base': 0.0,
            'joint_lift': 0.586,
            'wrist_extension': 0.0,
            'joint_wrist_yaw': 0.027
        }
        rospy.loginfo('Moving the arm to carrying position...')
        self.main(pose_dict)
        rospy.sleep(3)

    def go_to_stow_pose(self):
        # Stows the arm
        pose_dict = {
            'translate_mobile_base': 0.0,
            'joint_lift': 0.2,
            'wrist_extension': 0.0,
            'joint_wrist_yaw': 3.395
        }
        rospy.loginfo('Stowing the arm...')
        self.main(pose_dict)

    def close_gripper(self):
        # Closes the gripper
        gripper_pose = {'joint_gripper_finger_left': -0.126}
        self.move_to_pose(gripper_pose)

    def open_gripper(self):
        # Opens the gripper
        gripper_pose = {'joint_gripper_finger_left': 0.203}
        self.move_to_pose(gripper_pose)

    def grasp_obj(self):
        # Executes the grasping sequence
        rospy.loginfo('Starting to execute a manipulation task!!')
        self.lift_arm_pre_ik()

        response = self.grasp_pose_suggestor_service(0,0)
        if response.success:
            pose = response.grasp_pose.pose
            target_pos = [pose.position.x, pose.position.y, pose.position.z]
            quaternion_ori = [0.004, -0.014, -0.677, 0.7355]

            self.open_gripper()
            self.go_to_pre_ik_pose()
            self.go_to_grasp_pose(target_pos, quaternion_ori)
            self.close_gripper()
            self.go_to_pre_ik_pose(post_ik_flag=True)
            self.lift_arm_pre_ik(post_ik_flag=True)
            self.go_to_carrying_pose()

            rospy.loginfo('Manipulation task successfully executed!! Ready for navigation')
        else:
            rospy.logerr("Failed to get grasp pose")

    def main(self, pose_dict):
        # Main execution loop of the StretchNode
        super(StretchNode, self).main('stretch_controller', 'stretch_namespace', wait_for_first_pointcloud=False)
        self.move_to_pose(pose_dict)

# Main script execution
if __name__ == '__main__':
    try:
        stretch_node = StretchNode()  # Create an instance of StretchNode
        urdf_path = "./stretch.urdf"
        ik_engine = StretchIK(urdf_path)  # Initialize IK engine
        stretch_node.grasp_obj()  # Begin grasp sequence
    except rospy.ROSInterruptException:
        pass
