#!/usr/bin/env python
"""
@package contact_detection
@file contact_detection_action_client.py
@author Anthony Remazeilles
@brief see README.md

Copyright Tecnalia Research and Innovation 2016
Distributed under the GNU GPL v3.
For full terms see https://www.gnu.org/licenses/gpl.txt
"""

import rospy

from geometry_msgs.msg import Point, WrenchStamped
from ar_signal_processing import SignalAnalysis
from ar_geometry_msg_conversion import wrench_to_array
import actionlib
import contact_detection.msg


def contact_client():
    client = actionlib.SimpleActionClient('/wrench_contact_detector',
                                          contact_detection.msg.DetectContactAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = contact_detection.msg.DetectContactGoal()
    goal.do_noise_calibration = True
    goal.frequency = 100
    goal.finish_on_contact = True

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('contact_detector_client')
        result = contact_client()
        print "Result: {}".format(result)
        
    except rospy.ROSInterruptException:
        print "program interrupted before completion"

