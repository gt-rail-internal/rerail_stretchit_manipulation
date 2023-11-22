#!/usr/bin/env python3
import rospy
import hello_helpers.hello_misc as hm
from ik import StretchIK
from utils import quaternion_to_rotation_matrix
from geometry_msgs.msg import Pose
from rerail_stretchit_manipulation.srv import RerailManip, RerailManipResponse


# Define the StretchNode class inheriting from HelloNode
class StretchNode(hm.HelloNode):
    def __init__(self):
        # super(StretchNode, self).__init__()  # Initialize the superclass
        hm.HelloNode.__init__(self)
        hm.HelloNode.main(self, 'stretch_controller', 'stretch_namespace', wait_for_first_pointcloud=False)
        
        # Service that can be called to execute the manipulation task
        self.service = rospy.Service('/rerail_stretchit_manipulation/manipulate', RerailManip, self.handle_grasp_pose)
        self.service

    def get_ik_pose_dict(self, trg_pose, trg_quat):
        # Convert target pose and quaternion to robot's grasp position
        target_ori = quaternion_to_rotation_matrix(trg_quat)
        rospy.loginfo('Computing IK on Grasp Goal...')
        pose_dict = ik_engine.solve_ik(trg_pose, target_ori)
        rospy.loginfo("Sucessful computing!! Joint state goals for grasping: %s", str(pose_dict))
        return pose_dict
    
    
    def go_to_grasp_pose(self, grasp_pose_dict):
        # Convert target pose and quaternion to robot's grasp position
        grasp_pose_dict['translate_mobile_base'] = 0.0
        rospy.loginfo('Moving the arm to grasp pose...')
        self.main(grasp_pose_dict)
        #self.move_to_pose(grasp_pose_dict)
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

    def go_to_pre_ik_pose(self, grasp_pose_dict, post_ik_flag=False):
        
        if post_ik_flag:
            # Positions the arm after grasping the objects
            pose_dict = {
                'translate_mobile_base': 0.0,
                'joint_lift': 0.899,
                'wrist_extension': 0.214,
                'joint_wrist_yaw': 0.027
            }
        else:
            # Positions the arm before or after IK calculations
            pose_dict = grasp_pose_dict.copy()
            pose_dict['joint_lift'] = 0.95


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

    def move_camera(self):
        # Points the camera to the surface
        camera_pose = {
            'joint_head_pan' : -1.7042984434017716,
            'joint_head_tilt' : -0.39189916300751343
        }
        self.move_to_pose(camera_pose)

    # Service callback function
    def handle_grasp_pose(self, req):
        #print(">>>>", req.target_pose)
        try:
            target_pos = [req.target_pose.position.x, req.target_pose.position.y, req.target_pose.position.z]
            quaternion_ori = [req.target_pose.orientation.x, req.target_pose.orientation.y, req.target_pose.orientation.z, req.target_pose.orientation.w]
            
            # Obtain Grasp Pose
            grasp_pose_dict = self.get_ik_pose_dict(target_pos, quaternion_ori)
            
            #rospy.loginfo('This is the grasp pose dict!!!: ' + str(grasp_pose_dict))
            # Execute Grasp Sequence
            self.move_camera()
            self.lift_arm_pre_ik()
            self.open_gripper()
            self.go_to_pre_ik_pose(grasp_pose_dict)
            self.go_to_grasp_pose(grasp_pose_dict)
            self.close_gripper()
            self.go_to_pre_ik_pose(grasp_pose_dict, post_ik_flag=True)
            self.lift_arm_pre_ik(post_ik_flag=True)
            self.go_to_carrying_pose()
            
            rospy.loginfo('Manipulation task successfully executed!! Ready for navigation')
            return RerailManipResponse(success=True, message="Grasp completed successfully")
        except Exception as e:
            rospy.logerr("Failed to execute grasp pose: %s", e)
            return RerailManipResponse(success=False, message=str(e))

    def main(self, pose_dict):
        # Moves the arm to a given pose        
        self.move_to_pose(pose_dict)

# Main script execution
if __name__ == '__main__':
    try:
        stretch_node = StretchNode()  # Create an instance of StretchNode
        rospy.loginfo('Initialized manip node!!')
        urdf_path = "./stretch.urdf"
        ik_engine = StretchIK(urdf_path)  # Initialize IK engine
        rospy.loginfo('Manipulation grasp service ready!!!')
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
