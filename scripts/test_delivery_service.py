#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger, TriggerRequest

if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('test_delivery_service')

        # Create a service proxy for the execute_delivery_task service
        rospy.wait_for_service('execute_delivery_task')
        execute_delivery_task = rospy.ServiceProxy('execute_delivery_task', Trigger)

        # Create a TriggerRequest (empty request)
        request = TriggerRequest()

        # Call the service
        response = execute_delivery_task(request)

        # Check the response and print the result
        if response.success:
            rospy.loginfo("Delivery task executed successfully")
        else:
            rospy.logerr(f"Failed to execute delivery task: {response.message}")

    except rospy.ROSInterruptException:
        rospy.loginfo('Test script interrupted')
    except Exception as e:
        rospy.logerr(f"Test script error: {str(e)}")
