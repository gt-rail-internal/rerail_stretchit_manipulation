#!/usr/bin/env python3
import rospy
import time
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import hello_helpers.hello_misc as hm
from std_srvs.srv import Trigger, TriggerResponse

# Author and Maintainer: Juan Antonio Robledo (jroblar98@gmail.com)

class DeliverObjectCommand(hm.HelloNode):
  """
  A class that sends multiple joint trajectory goals to the stretch robot to deliver objects.
  All of this trajectories have hardcoded positions that can be modified.
  """
  def __init__(self):
    hm.HelloNode.__init__(self)
    hm.HelloNode.main(self, 'multipoint_command', 'multipoint_command', wait_for_first_pointcloud=False)

    # Create the ROS service
    self.delivery_service = rospy.Service('execute_delivery_task', Trigger, self.main)
    rospy.loginfo('execute_delivery_task service ready!!!')


  def lift_arm_to_safe_pose(self):
    """
    Function lifts the arm to a safe position pior delivery
    """
    lift_arm = JointTrajectoryPoint()
    lift_arm.positions = [0.942526617861438, 0.0, 3.4, -0.126]


    trajectory_goal = FollowJointTrajectoryGoal()
    trajectory_goal.trajectory.joint_names = ['joint_lift', 'wrist_extension', 'joint_wrist_yaw', 'joint_gripper_finger_left']
    trajectory_goal.trajectory.points = [lift_arm]
    trajectory_goal.trajectory.header.stamp = rospy.Time(0.0)
    trajectory_goal.trajectory.header.frame_id = 'base_link'

    self.trajectory_client.send_goal(trajectory_goal)
    rospy.loginfo('Lifting arm pre delivery = {0}'.format(trajectory_goal))
    self.trajectory_client.wait_for_result()


  def reach_delivery_pose(self):
    """
    Function that makes an action call and sends multiple joint trajectory goals
    to the joint_lift, wrist_extension, joint_wrist_yaw, and joint_gripper_finger_left.
    :param self: The self reference.
    """
    # extend arm
    point1 = JointTrajectoryPoint()
    point1.positions = [0.942526617861438, 0.513410513603503, 0.027, -0.126]

    # lower arm
    point2 = JointTrajectoryPoint()
    point2.positions = [0.888899662603034, 0.513410513603503, 0.027, -0.126]

    # release grasp
    point3 = JointTrajectoryPoint()
    point3.positions = [0.8398899662603034, 0.503410513603503, 0.026, 0.203]

    # Execute trajectory
    trajectory_goal = FollowJointTrajectoryGoal()
    trajectory_goal.trajectory.joint_names = ['joint_lift', 'wrist_extension', 'joint_wrist_yaw', 'joint_gripper_finger_left']
    trajectory_goal.trajectory.points = [point1, point2, point3]
    trajectory_goal.trajectory.header.stamp = rospy.Time(0.0)
    trajectory_goal.trajectory.header.frame_id = 'base_link'

    self.trajectory_client.send_goal(trajectory_goal)
    rospy.loginfo('Sent list of goals = {0}'.format(trajectory_goal))
    self.trajectory_client.wait_for_result()

  def return_to_safe_pose(self):
    """
    Function that makes an action call and sends multiple joint trajectory goals
    to the joint_lift, wrist_extension, joint_wrist_yaw, and joint_gripper_finger_left.
    :param self: The self reference.
    """


    # lift arm
    point4 = JointTrajectoryPoint()
    point4.positions = [0.942526617861438, 0.513410513603503, 0.027, 0.203]

    # retract the arm and hide wrist
    point5 = JointTrajectoryPoint()
    point5.positions = [0.942526617861438, 0.0, 3.395, 0.0]


    # Execute trajectory
    trajectory_goal = FollowJointTrajectoryGoal()
    trajectory_goal.trajectory.joint_names = ['joint_lift', 'wrist_extension', 'joint_wrist_yaw', 'joint_gripper_finger_left']
    trajectory_goal.trajectory.points = [point4, point5]
    trajectory_goal.trajectory.header.stamp = rospy.Time(0.0)
    trajectory_goal.trajectory.header.frame_id = 'base_link'

    self.trajectory_client.send_goal(trajectory_goal)
    rospy.loginfo('Sent list of goals = {0}'.format(trajectory_goal))
    self.trajectory_client.wait_for_result()


  def issue_stow_command(self):
    '''
    Function that makes an action call and sends stow position goal.
    :param self: The self reference.
    '''
    stow_point = JointTrajectoryPoint()
    stow_point.time_from_start = rospy.Duration(0.000)
    stow_point.positions = [0.2, 0.0, 3.4]

    trajectory_goal = FollowJointTrajectoryGoal()
    trajectory_goal.trajectory.joint_names = ['joint_lift', 'wrist_extension', 'joint_wrist_yaw']
    trajectory_goal.trajectory.points = [stow_point]
    trajectory_goal.trajectory.header.stamp = rospy.Time(0.0)
    trajectory_goal.trajectory.header.frame_id = 'base_link'

    self.trajectory_client.send_goal(trajectory_goal)
    rospy.loginfo('Sent stow goal = {0}'.format(trajectory_goal))
    self.trajectory_client.wait_for_result()

  def main(self, req):
    """
    ROS service callback to execute the delivery task.
    :param req: The service request (not used in this case).
    :return: A TriggerResponse indicating the success or failure of the task.
    """
    try:
        self.lift_arm_to_safe_pose()
        time.sleep(1)
        self.reach_delivery_pose()
        time.sleep(1)
        self.return_to_safe_pose()
        time.sleep(1)
        self.issue_stow_command()
        time.sleep(1)

        # If all movements were successful, return True
        return TriggerResponse(success=True, message="Delivery task executed successfully")
    except Exception as e:
        # If any error occurs during execution, return False
        rospy.logerr(f"Error during delivery task execution: {str(e)}")
        return TriggerResponse(success=False, message=f"Error during delivery task execution: {str(e)}")

if __name__ == '__main__':
  try:
    node = DeliverObjectCommand()
    rospy.loginfo('Ready to execute delivery task...')
    rospy.spin()
  except KeyboardInterrupt:
    rospy.loginfo('interrupt received, so shutting down')
