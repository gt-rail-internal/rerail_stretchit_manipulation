#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger, TriggerRequest

if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('test_camera_service')

        # Create a service proxy for the execute_camera_movement service
        rospy.wait_for_service('execute_camera_movement')
        execute_camera_movement = rospy.ServiceProxy('execute_camera_movement', Trigger)

        # Create a TriggerRequest (empty request)
        request = TriggerRequest()

        # Call the service
        response = execute_camera_movement(request)

        # Check the response and print the result
        if response.success:
            rospy.loginfo("Camera movement executed successfully")
        else:
            rospy.logerr(f"Failed to execute camera movement: {response.message}")

    except rospy.ROSInterruptException:
        rospy.loginfo('Test script interrupted')
    except Exception as e:
        rospy.logerr(f"Test script error: {str(e)}")
