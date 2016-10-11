#!/usr/bin/env python
"""
@package contact_detection
@file ar_signal_processing.py
@author Anthony Remazeilles
@brief see README.md

Copyright Tecnalia Research and Innovation 2016
Distributed under the GNU GPL v3. For full terms see https://www.gnu.org/licenses/gpl.txt
"""

import rospy

from geometry_msgs.msg import Point, WrenchStamped
from ar_signal_processing import SignalAnalysis
from ar_geometry_msg_conversion import wrench_to_array
import actionlib
import contact_detection.msg

class WrenchContactDetectorNode(object):

    def __init__(self, name="wrench_contact_detector"):
        self._name = name
        self._is_init_ok = False
        self._analysis = SignalAnalysis(size=6, num_sample_init=50)

        self._is_init_ok = True

        # ###########################
        # initialising ros interface
        # for receiving the wrenches
        # self.sub_wrench =  rospy.Subscriber("wrench", WrenchStamped, self._wrench_callback)
        self._sub_wrench =  None

        self._action_server = actionlib.SimpleActionServer(self._name,
                                                           contact_detection.msg.DetectContactAction,
                                                           execute_cb=self._execute_cb, auto_start=False)
        self._action_server.start()

    def execute_cb(self, goal):
        """
        adress an action request
        :param goal the provided spec of the current action
        """
        rospy.loginfo("{} launching contact detector action with spec {}".format(self._name, goal))
        feedback = contact_detection.DetectContactFeedback()
        result = contact_detection.DetectContactResult()

        rate = rospy.Rate(goal.frequency)

        end_loop = False
        is_in_contact = False
        while not end_loop:
            feedback.is_in_contact = False
            self._action_server.publish_feedback(feedback)

            end_loop = self._action_server.is_preempt_requested() or rospy._is_shutdown() or is_in_contact
            rate.sleep()

        if self._action_server.is_preempt_requested():
            self._action_server.set_preempted()
        else:
            if is_in_contact:
                self._action_server.set_succeeded(feedback)
            else:
                self._action_server.set_aborted(feedback)


    def _wrench_callback(self, msg):
        # rospy.loginfo("{} received: {}".format(name, msg))

        output = self._analysis.process(wrench_to_array(msg.wrench))
        print "output is: {}".format(output)


if __name__ == '__main__':
    name = "wrench_contact_detector"
    rospy.init_node(name, anonymous=True)

    detector = WrenchContactDetectorNode(name)


    if detector._is_init_ok:
        rospy.spin()
    else:
        rospy.logerr("Prb in initialization. Bye")
