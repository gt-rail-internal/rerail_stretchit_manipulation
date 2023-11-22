#!/usr/bin/env python3

import rospy
import time
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import hello_helpers.hello_misc as hm


# TODO: Make this class to work with a ROS service.


class MoveCameraCommand(hm.HelloNode):
  """
  A class that sends a goal trajectory to Strecht's camera.
  """
  def __init__(self):
    hm.HelloNode.__init__(self)

  def issue_movecamera_command(self):
    """
    Function that makes an action call and sends a joint trajectory points to
    joint_head_pan and joint_head_tilt to move the camera.
    :param self: The self reference.
    """
    point0 = JointTrajectoryPoint()
    point0.positions = [-1.7042984434017716, -0.39189916300751343]


    trajectory_goal = FollowJointTrajectoryGoal()
    trajectory_goal.trajectory.joint_names = ['joint_head_pan', 'joint_head_tilt']
    trajectory_goal.trajectory.points = [point0]
    trajectory_goal.trajectory.header.stamp = rospy.Time(0.0)
    trajectory_goal.trajectory.header.frame_id = 'base_link'

    self.trajectory_client.send_goal(trajectory_goal)
    rospy.loginfo('Sent list of goals = {0}'.format(trajectory_goal))
    self.trajectory_client.wait_for_result()

  def main(self):
    """
    Function that initiates the movecamera_command function.
    :param self: The self reference.
    """
    hm.HelloNode.main(self, 'issue_command', 'issue_command', wait_for_first_pointcloud=False)
    rospy.loginfo('issuing move camera command...')
    self.issue_movecamera_command()
    time.sleep(2)


if __name__ == '__main__':
  try:
    node = MoveCameraCommand()
    node.main()
  except KeyboardInterrupt:
    rospy.loginfo('interrupt received, so shutting down')